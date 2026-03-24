// ============================================================================
// scenes.cpp — 调试场景的实现
// ============================================================================
// 场景矩阵（面向 Null 后端刚体回归测试）：
//   BoxDrop     — 单体落地：重力 + 接触 + 停止
//   BoxPush     — 双体对撞：接触法线 + 速度冲量响应
//   BoxSlope    — 斜面滑落/停留：法线方向 + 摩擦静止判定
//   BoxSpinDrop — 旋转落地：角速度积分 + 旋转碰撞
//   BoxStack    — 纯 box 堆叠：多体接触 + 堆叠稳定性 + 睡眠
//   FrictionLab — 摩擦对比：高/低摩擦两块平台上同类 box 的滑动差异
//   RagdollPrep — 布娃娃预备：多形状（Sphere/Capsule），关节系统入口
//
// 所有预置场景是确定性的（无随机性）。随机性仅存在于 SpawnDynamicBox()。
// ============================================================================
#include "scenes.h"

#include <random>
#include <string>

namespace lab {
namespace {

// ResetLocalColliderPose — 将碰撞体的局部位姿重置为零偏移 + 单位旋转。
// 大多数简单对象碰撞体与 Transform 完全重合，统一在此归零避免重复代码。
// 调用方：MakeBox / MakeSphere / MakeCapsule 创建碰撞组件后立即调用。
inline void ResetLocalColliderPose(World& world, ObjectId id) {
    world.colliders.local_offset[id] = Vec3{0.0f, 0.0f, 0.0f};
    world.colliders.local_rotation[id] = IdentityQuat();
}

// SetCommonRigidBody — 为对象启用刚体组件并写入运动类型、质量、重力开关。
// 共用逻辑提取成函数，避免在 MakeBox/MakeSphere/MakeCapsule 中重复代码。
// 调用方：MakeBox / MakeSphere / MakeCapsule 在设置渲染和碰撞组件后调用。
inline void SetCommonRigidBody(World& world,
                               ObjectId id,
                               MotionType motion_type,
                               float mass) {
    world.bodies.enabled[id] = 1;
    world.bodies.motion_type[id] = motion_type;
    world.bodies.mass[id] = mass;
    world.bodies.use_gravity[id] = (motion_type == MotionType::Dynamic) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// MakeBox — 创建一个 Box 对象（变换 + 渲染 + 碰撞 + 刚体）
// ---------------------------------------------------------------------------
// 当前 authoring 层仍然让 render 与 collider 保持一致，
// 但它们已经落在不同 SoA 里，后续可以分别演化。
// 调用方：BuildBoxStack()（地面/围墙/堆叠盒子）、BuildRagdollPrep()（地面/躯干）、SpawnDynamicBox()。
ObjectId MakeBox(World& world,
                 const char* name,
                 Vec3 position,
                 Vec3 half_extents,
                 MotionType motion_type,
                 ColorRGBA8 color,
                 float mass = 1.0f) {
    const ObjectId id = world.CreateObject(name);
    world.transforms.position[id] = position;
    world.transforms.rotation[id] = IdentityQuat();
    world.transforms.scale[id] = Vec3{1.0f, 1.0f, 1.0f};

    // 渲染组件
    world.render.enabled[id] = 1;
    world.render.shape[id] = ShapeKind::Box;
    world.render.box_half_extents[id] = half_extents;
    world.render.color[id] = color;

    // 碰撞组件
    world.colliders.enabled[id] = 1;
    world.colliders.shape[id] = ShapeKind::Box;
    world.colliders.box_half_extents[id] = half_extents;
    ResetLocalColliderPose(world, id);

    // 刚体组件
    SetCommonRigidBody(world, id, motion_type, mass);
    return id;
}

// ---------------------------------------------------------------------------
// MakeSphere — 创建一个 Sphere 对象（变换 + 渲染 + 碰撞 + 刚体）
// ---------------------------------------------------------------------------
// 调用方：BuildRagdollPrep()（头部球体）。
ObjectId MakeSphere(World& world,
                    const char* name,
                    Vec3 position,
                    float radius,
                    MotionType motion_type,
                    ColorRGBA8 color,
                    float mass = 1.0f) {
    const ObjectId id = world.CreateObject(name);
    world.transforms.position[id] = position;
    world.transforms.rotation[id] = IdentityQuat();
    world.transforms.scale[id] = Vec3{1.0f, 1.0f, 1.0f};

    // 渲染组件
    world.render.enabled[id] = 1;
    world.render.shape[id] = ShapeKind::Sphere;
    world.render.sphere_radius[id] = radius;
    world.render.color[id] = color;

    // 碰撞组件
    world.colliders.enabled[id] = 1;
    world.colliders.shape[id] = ShapeKind::Sphere;
    world.colliders.sphere_radius[id] = radius;
    ResetLocalColliderPose(world, id);

    // 刚体组件
    SetCommonRigidBody(world, id, motion_type, mass);

    // 球体容易滚动永不停，提高角阻尼和摩擦系数
    world.bodies.friction[id]        = 0.8f;
    world.bodies.angular_damping[id] = 0.6f;
    return id;
}

// ---------------------------------------------------------------------------
// MakeCapsule — 创建一个 Capsule 对象（变换 + 渲染 + 碰撞 + 刚体）
// ---------------------------------------------------------------------------
// 胶囊需要额外指定 rotation，因为其默认沿 Y 轴，
// 有时需要旋转 90° 让它横过来（如手臂）。
// 调用方：BuildRagdollPrep()（上臂/前臂/大腿/小腿等肢体部件）。
ObjectId MakeCapsule(World& world,
                     const char* name,
                     Vec3 position,
                     float radius,
                     float half_height,
                     Quat rotation,
                     MotionType motion_type,
                     ColorRGBA8 color,
                     float mass = 1.0f) {
    const ObjectId id = world.CreateObject(name);
    world.transforms.position[id] = position;
    world.transforms.rotation[id] = rotation;
    world.transforms.scale[id] = Vec3{1.0f, 1.0f, 1.0f};

    // 渲染组件
    world.render.enabled[id] = 1;
    world.render.shape[id] = ShapeKind::Capsule;
    world.render.capsule_radius[id] = radius;
    world.render.capsule_half_height[id] = half_height;
    world.render.color[id] = color;

    // 碰撞组件
    world.colliders.enabled[id] = 1;
    world.colliders.shape[id] = ShapeKind::Capsule;
    world.colliders.capsule_radius[id] = radius;
    world.colliders.capsule_half_height[id] = half_height;
    ResetLocalColliderPose(world, id);

    // 刚体组件
    SetCommonRigidBody(world, id, motion_type, mass);

    // 胶囊也容易滚动，提高角阻尼
    world.bodies.friction[id]        = 0.7f;
    world.bodies.angular_damping[id] = 0.5f;
    return id;
}

// ===========================================================================
// BuildBoxDrop — 单体落地场景
// ===========================================================================
// 测试目的：重力积分 → 地面接触 → 速度衰减 → 停止/睡眠。
// 场景内容：大地面 + 一个动态 box 从 y=5 处自由下落。
void BuildBoxDrop(World& world) {
    world.Clear();

    MakeBox(world, "ground",
            Vec3{0.0f, -1.0f, 0.0f}, Vec3{20.0f, 1.0f, 20.0f},
            MotionType::Static, ColorRGBA8{180, 180, 180, 255}, 0.0f);

    MakeBox(world, "drop_box",
            Vec3{0.0f, 5.0f, 0.0f}, Vec3{0.5f, 0.5f, 0.5f},
            MotionType::Dynamic, ColorRGBA8{220, 100, 80, 255}, 1.0f);
}

// ===========================================================================
// BuildBoxPush — 双体对撞场景
// ===========================================================================
// 测试目的：水平方向接触法线 → 冲量交换 → 速度响应。
// 场景内容：大地面 + 两个动态 box 从相对方向以初速度相向运动。
void BuildBoxPush(World& world) {
    world.Clear();

    MakeBox(world, "ground",
            Vec3{0.0f, -1.0f, 0.0f}, Vec3{20.0f, 1.0f, 20.0f},
            MotionType::Static, ColorRGBA8{180, 180, 180, 255}, 0.0f);

    // 左侧 box：向右运动
    const ObjectId left = MakeBox(world, "push_left",
            Vec3{-3.0f, 0.5f, 0.0f}, Vec3{0.5f, 0.5f, 0.5f},
            MotionType::Dynamic, ColorRGBA8{80, 160, 220, 255}, 1.0f);
    world.body_state.linear_velocity[left] = Vec3{4.0f, 0.0f, 0.0f};

    // 右侧 box：向左运动
    const ObjectId right = MakeBox(world, "push_right",
            Vec3{3.0f, 0.5f, 0.0f}, Vec3{0.5f, 0.5f, 0.5f},
            MotionType::Dynamic, ColorRGBA8{220, 160, 80, 255}, 1.0f);
    world.body_state.linear_velocity[right] = Vec3{-4.0f, 0.0f, 0.0f};
}

// ===========================================================================
// BuildBoxSlope — 斜面滑落场景
// ===========================================================================
// 测试目的：倾斜法线方向 → 重力分解 → 摩擦与滑动。
// 场景内容：大地面 + 斜放的静态 box 作坡面 + 动态 box 放在坡面上方。
// 坡面绕 Z 轴旋转约 25°，长条形以便 box 有足够滑行空间。
void BuildBoxSlope(World& world) {
    world.Clear();

    MakeBox(world, "ground",
            Vec3{0.0f, -1.0f, 0.0f}, Vec3{20.0f, 1.0f, 20.0f},
            MotionType::Static, ColorRGBA8{180, 180, 180, 255}, 0.0f);

    // 坡面：绕 Z 轴旋转 25° ≈ 0.4363 rad，长条形 box
    const float slope_angle = 0.4363f;  // ~25°
    const ObjectId slope = MakeBox(world, "slope",
            Vec3{0.0f, 1.8f, 0.0f}, Vec3{4.0f, 0.15f, 1.5f},
            MotionType::Static, ColorRGBA8{140, 160, 140, 255}, 0.0f);
    world.transforms.rotation[slope] = FromAxisAngle(Vec3{0.0f, 0.0f, 1.0f}, slope_angle);

    // 动态 box：放在坡面上方偏高处，保证接触坡面
    // 坡面中心 y=1.8，倾斜后左高右低；box 放在坡面偏上侧
    MakeBox(world, "slider",
            Vec3{-1.5f, 3.5f, 0.0f}, Vec3{0.4f, 0.4f, 0.4f},
            MotionType::Dynamic, ColorRGBA8{210, 180, 70, 255}, 1.0f);
}

// ===========================================================================
// BuildBoxSpinDrop — 旋转落地场景
// ===========================================================================
// 测试目的：角速度积分 → 旋转状态下的碰撞 → 角速度衰减。
// 场景内容：大地面 + 一个带明确初始角速度的动态 box 从 y=4 落下。
void BuildBoxSpinDrop(World& world) {
    world.Clear();

    MakeBox(world, "ground",
            Vec3{0.0f, -1.0f, 0.0f}, Vec3{20.0f, 1.0f, 20.0f},
            MotionType::Static, ColorRGBA8{180, 180, 180, 255}, 0.0f);

    const ObjectId spinner = MakeBox(world, "spin_box",
            Vec3{0.0f, 4.0f, 0.0f}, Vec3{0.5f, 0.5f, 0.5f},
            MotionType::Dynamic, ColorRGBA8{170, 90, 210, 255}, 1.0f);
    // 明确的旋转初速度：绕 (1,0.5,0) 方向旋转，约 ~6 rad/s
    world.body_state.angular_velocity[spinner] = Vec3{5.0f, 3.0f, 0.0f};
}

// ===========================================================================
// BuildFrictionLab — 摩擦对比场景
// ===========================================================================
// 测试目的：摩擦系数差异 → 同初速度下不同滑动表现。
// 场景内容：大地面 + 左/右两块平台（高摩擦/低摩擦）+ 各一个动态 box 以相同初速推出。
void BuildFrictionLab(World& world) {
    world.Clear();

    MakeBox(world, "ground",
            Vec3{0.0f, -1.0f, 0.0f}, Vec3{20.0f, 1.0f, 20.0f},
            MotionType::Static, ColorRGBA8{180, 180, 180, 255}, 0.0f);

    // 左侧平台：高摩擦 (0.95)
    const ObjectId plat_hi = MakeBox(world, "platform_hi_friction",
            Vec3{-3.0f, 0.15f, 0.0f}, Vec3{3.0f, 0.15f, 1.5f},
            MotionType::Static, ColorRGBA8{180, 100, 100, 255}, 0.0f);
    world.bodies.friction[plat_hi] = 0.95f;

    // 右侧平台：低摩擦 (0.05)
    const ObjectId plat_lo = MakeBox(world, "platform_lo_friction",
            Vec3{3.0f, 0.15f, 0.0f}, Vec3{3.0f, 0.15f, 1.5f},
            MotionType::Static, ColorRGBA8{100, 100, 180, 255}, 0.0f);
    world.bodies.friction[plat_lo] = 0.05f;

    // 左侧 box：初速向右 3 m/s，高摩擦底板上应快速停止
    const ObjectId box_hi = MakeBox(world, "box_hi_friction",
            Vec3{-4.5f, 0.8f, 0.0f}, Vec3{0.4f, 0.4f, 0.4f},
            MotionType::Dynamic, ColorRGBA8{230, 130, 130, 255}, 1.0f);
    world.bodies.friction[box_hi] = 0.8f;
    world.body_state.linear_velocity[box_hi] = Vec3{3.0f, 0.0f, 0.0f};

    // 右侧 box：初速向右 3 m/s，低摩擦底板上应滑更远
    const ObjectId box_lo = MakeBox(world, "box_lo_friction",
            Vec3{1.5f, 0.8f, 0.0f}, Vec3{0.4f, 0.4f, 0.4f},
            MotionType::Dynamic, ColorRGBA8{130, 130, 230, 255}, 1.0f);
    world.bodies.friction[box_lo] = 0.1f;
    world.body_state.linear_velocity[box_lo] = Vec3{3.0f, 0.0f, 0.0f};
}

// ===========================================================================
// BuildBoxStack — 第一阶段主调试场景：纯 box 堆叠
// ===========================================================================
// 场景内容：
//   - 一块大地面（静态）
//   - 一组 L 形围墙（静态）
//   - 4 层金字塔形堆叠的动态盒子（4+3+2+1=10 个）
//
// 说明：
//   - 这里故意不再放 sphere / capsule
//   - 目标是让 NullPhysics 先在“纯 box”环境下把重力、接触、堆叠、睡眠跑顺
void BuildBoxStack(World& world) {
    world.Clear();

    // 地面：40x2x40 的静态盒子，表面位于 y=0
    MakeBox(world,
            "ground",
            Vec3{0.0f, -1.0f, 0.0f},
            Vec3{20.0f, 1.0f, 20.0f},
            MotionType::Static,
            ColorRGBA8{180, 180, 180, 255},
            0.0f);

    // L 形围墙（两面相邻）：后墙 + 左墙，留出前方和右方供相机观察
    MakeBox(world,
            "wall_back",
            Vec3{0.0f, 3.0f, -10.0f},
            Vec3{10.5f, 3.0f, 0.5f},
            MotionType::Static,
            ColorRGBA8{120, 140, 150, 255},
            0.0f);
    MakeBox(world,
            "wall_left",
            Vec3{-10.0f, 3.0f, 0.0f},
            Vec3{0.5f, 3.0f, 10.5f},
            MotionType::Static,
            ColorRGBA8{120, 140, 150, 255},
            0.0f);

    // 金字塔形堆叠盒子：4 层，每层比上一层少一个
    int counter = 0;
    for (int y = 0; y < 4; ++y) {
        const int width = 4 - y;  // 底层 4 个，顶层 1 个
        for (int x = 0; x < width; ++x) {
            const float fx = (static_cast<float>(x) - static_cast<float>(width - 1) * 0.5f) * 1.15f;
            const float fy = 0.5f + static_cast<float>(y) * 1.05f;

            const std::string name = "stack_box_" + std::to_string(counter++);
            MakeBox(world,
                    name.c_str(),
                    Vec3{fx, fy, 0.0f},
                    Vec3{0.5f, 0.5f, 0.5f},
                    MotionType::Dynamic,
                    ColorRGBA8{210, 135, 85, 255},
                    1.0f);
        }
    }
}

// ===========================================================================
// BuildRagdollPrep — 布娃娃预备实验室
// ===========================================================================
// 场景内容：
//   - 地面（静态）
//   - 一个简化人偶：躯干（Box）+ 头（Sphere）
//     + 左/右上臂、左/右前臂（Capsule，横置）
//     + 左/右大腿、左/右小腿（Capsule，竖置）
//
// 当前所有部件都是独立的动态刚体，没有关节连接。
// 后续增加关节系统后，这些部件会被约束连接起来。
void BuildRagdollPrep(World& world) {
    world.Clear();

    // 地面：40x2x40，表面位于 y=0
    MakeBox(world,
            "ground",
            Vec3{0.0f, -1.0f, 0.0f},
            Vec3{20.0f, 1.0f, 20.0f},
            MotionType::Static,
            ColorRGBA8{180, 180, 180, 255},
            0.0f);

    // L 形围墙（后墙 + 左墙）
    MakeBox(world,
            "wall_back",
            Vec3{0.0f, 3.0f, -10.0f},
            Vec3{10.5f, 3.0f, 0.5f},
            MotionType::Static,
            ColorRGBA8{120, 140, 150, 255},
            0.0f);
    MakeBox(world,
            "wall_left",
            Vec3{-10.0f, 3.0f, 0.0f},
            Vec3{0.5f, 3.0f, 10.5f},
            MotionType::Static,
            ColorRGBA8{120, 140, 150, 255},
            0.0f);

    // 躯干：用 Box 表示，质量 6kg
    MakeBox(world,
            "torso",
            Vec3{0.0f, 2.3f, 0.0f},
            Vec3{0.35f, 0.55f, 0.22f},
            MotionType::Dynamic,
            ColorRGBA8{235, 170, 110, 255},
            6.0f);

    // 头部：用 Sphere 表示
    MakeSphere(world,
               "head",
               Vec3{0.0f, 3.25f, 0.0f},
               0.28f,
               MotionType::Dynamic,
               ColorRGBA8{240, 205, 160, 255},
               2.0f);

    // 手臂旋转 90°——胶囊默认沿 Y 轴，旋转后变为水平横置
    const Quat arm_rotation = FromAxisAngle(Vec3{0.0f, 0.0f, 1.0f}, 1.5707963f);

    // 左上臂
    MakeCapsule(world,
                "upper_arm_l",
                Vec3{-0.78f, 2.65f, 0.0f},
                0.12f,
                0.35f,
                arm_rotation,
                MotionType::Dynamic,
                ColorRGBA8{85, 130, 235, 255},
                1.2f);
    // 左前臂
    MakeCapsule(world,
                "lower_arm_l",
                Vec3{-1.48f, 2.65f, 0.0f},
                0.11f,
                0.32f,
                arm_rotation,
                MotionType::Dynamic,
                ColorRGBA8{85, 130, 235, 255},
                1.0f);
    // 右上臂
    MakeCapsule(world,
                "upper_arm_r",
                Vec3{0.78f, 2.65f, 0.0f},
                0.12f,
                0.35f,
                arm_rotation,
                MotionType::Dynamic,
                ColorRGBA8{85, 130, 235, 255},
                1.2f);
    // 右前臂
    MakeCapsule(world,
                "lower_arm_r",
                Vec3{1.48f, 2.65f, 0.0f},
                0.11f,
                0.32f,
                arm_rotation,
                MotionType::Dynamic,
                ColorRGBA8{85, 130, 235, 255},
                1.0f);

    // 左大腿——竖置（IdentityQuat）
    MakeCapsule(world,
                "upper_leg_l",
                Vec3{-0.22f, 1.28f, 0.0f},
                0.13f,
                0.48f,
                IdentityQuat(),
                MotionType::Dynamic,
                ColorRGBA8{90, 210, 125, 255},
                2.4f);
    // 左小腿
    MakeCapsule(world,
                "lower_leg_l",
                Vec3{-0.22f, 0.38f, 0.0f},
                0.11f,
                0.42f,
                IdentityQuat(),
                MotionType::Dynamic,
                ColorRGBA8{90, 210, 125, 255},
                2.0f);
    // 右大腿
    MakeCapsule(world,
                "upper_leg_r",
                Vec3{0.22f, 1.28f, 0.0f},
                0.13f,
                0.48f,
                IdentityQuat(),
                MotionType::Dynamic,
                ColorRGBA8{90, 210, 125, 255},
                2.4f);
    // 右小腿
    MakeCapsule(world,
                "lower_leg_r",
                Vec3{0.22f, 0.38f, 0.0f},
                0.11f,
                0.42f,
                IdentityQuat(),
                MotionType::Dynamic,
                ColorRGBA8{90, 210, 125, 255},
                2.0f);
}

// ===========================================================================
// BuildCubeRain — 方块雨场景（扩展版）
// ===========================================================================
// 场景内容：
//   - 大地面（静态）
//   - 8 层 × 6×6 网格 = 288 个动态方块悬浮在空中
// 排布策略（防止初始重叠蹦飞）：
//   - X/Z 方向：间距 1.1f（边长 0.8f + 0.3f 间隙），同层绝不重叠
//   - Y 方向：层间距 1.2f（边长 0.8f + 0.4f 间隙），层间绝不重叠
//   - 奇数层在 X/Z 方向偏移半步（0.55f），如砖墙排列，视觉上错开
// 初速度：
//   - 随机微量线速度（±0.8 m/s 水平，±0.3 m/s 垂直）
//   - 随机微量角速度（±2.5 rad/s，模拟混沌旋转）
void BuildCubeRain(World& world) {
    world.Clear();

    // 地面
    MakeBox(world, "ground",
            Vec3{0.0f, -1.0f, 0.0f}, Vec3{30.0f, 1.0f, 30.0f},
            MotionType::Static, ColorRGBA8{180, 180, 180, 255}, 0.0f);

    // 固定种子保证场景确定性（按 R 重载结果相同）
    std::mt19937 rng{12345u};
    std::uniform_real_distribution<float> vel_h(-0.8f, 0.8f);   // 水平线速度
    std::uniform_real_distribution<float> vel_v(-0.3f, 0.3f);   // 垂直线速度（微量）
    std::uniform_real_distribution<float> ang_v(-2.5f, 2.5f);   // 角速度每轴
    std::uniform_int_distribution<int>    color_ch(80, 230);     // 颜色通道

    constexpr int   kCols      = 6;     // 每层列数
    constexpr int   kRows      = 6;     // 每层行数
    constexpr int   kLayers    = 8;     // 总层数
    constexpr float kHalfExt   = 0.4f;  // cube 半边长
    constexpr float kStepXZ    = 1.1f;  // X/Z 方向间距
    constexpr float kStepY     = 1.2f;  // 层间 Y 距离
    constexpr float kBaseY     = 8.0f;  // 最低层中心高度

    // 网格中心偏移（让整体以 XZ 原点为中心）
    const float origin_x = -(kCols - 1) * kStepXZ * 0.5f;
    const float origin_z = -(kRows - 1) * kStepXZ * 0.5f;

    int counter = 0;
    for (int layer = 0; layer < kLayers; ++layer) {
        // 奇数层在 X/Z 各偏移半步，形成砖墙错位
        const float stagger = (layer % 2 == 1) ? kStepXZ * 0.5f : 0.0f;
        const float base_y  = kBaseY + static_cast<float>(layer) * kStepY;

        for (int row = 0; row < kRows; ++row) {
            for (int col = 0; col < kCols; ++col) {
                const float px = origin_x + stagger + static_cast<float>(col) * kStepXZ;
                const float pz = origin_z + stagger + static_cast<float>(row) * kStepXZ;

                const ColorRGBA8 color{
                    static_cast<std::uint8_t>(color_ch(rng)),
                    static_cast<std::uint8_t>(color_ch(rng)),
                    static_cast<std::uint8_t>(color_ch(rng)),
                    255};

                const std::string name = "rain_cube_" + std::to_string(counter++);
                const ObjectId id = MakeBox(world,
                                            name.c_str(),
                                            Vec3{px, base_y, pz},
                                            Vec3{kHalfExt, kHalfExt, kHalfExt},
                                            MotionType::Dynamic,
                                            color,
                                            1.0f);

                // 随机微量线速度
                world.body_state.linear_velocity[id] =
                    Vec3{vel_h(rng), vel_v(rng), vel_h(rng)};
                // 随机微量角速度
                world.body_state.angular_velocity[id] =
                    Vec3{ang_v(rng), ang_v(rng), ang_v(rng)};
            }
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// BuildScene — 根据场景 ID 构建世界数据
// ---------------------------------------------------------------------------
// 调用对应的 BuildXxx() 函数，然后做一次 CapturePreviousTransforms()
// 以初始化上一帧快照（避免第一帧插值闪烁）。
void BuildScene(World& world, DemoSceneId scene_id) {
    switch (scene_id) {
        case DemoSceneId::BoxDrop:
            BuildBoxDrop(world);
            break;
        case DemoSceneId::BoxPush:
            BuildBoxPush(world);
            break;
        case DemoSceneId::BoxSlope:
            BuildBoxSlope(world);
            break;
        case DemoSceneId::BoxSpinDrop:
            BuildBoxSpinDrop(world);
            break;
        case DemoSceneId::BoxStack:
            BuildBoxStack(world);
            break;
        case DemoSceneId::FrictionLab:
            BuildFrictionLab(world);
            break;
        case DemoSceneId::RagdollPrep:
            BuildRagdollPrep(world);
            break;
        case DemoSceneId::CubeRain:
            BuildCubeRain(world);
            break;
        default:
            BuildBoxDrop(world);
            break;
    }

    // 初始化上一帧快照，确保插值从第一帧开始就正确
    world.CapturePreviousTransforms();
}

// ---------------------------------------------------------------------------
// DemoSceneName — 返回场景名称字符串
// ---------------------------------------------------------------------------
const char* DemoSceneName(DemoSceneId scene_id) {
    switch (scene_id) {
        case DemoSceneId::BoxDrop:
            return "1: Box Drop (gravity+contact)";
        case DemoSceneId::BoxPush:
            return "2: Box Push (collision impulse)";
        case DemoSceneId::BoxSlope:
            return "3: Box Slope (normal+friction)";
        case DemoSceneId::BoxSpinDrop:
            return "4: Spin Drop (angular velocity)";
        case DemoSceneId::BoxStack:
            return "5: Box Stack (stacking+sleep)";
        case DemoSceneId::FrictionLab:
            return "6: Friction Lab (hi/lo compare)";
        case DemoSceneId::RagdollPrep:
            return "7: Ragdoll Prep (joints future)";
        case DemoSceneId::CubeRain:
            return "8: Cube Rain (mass spawn + stagger)";
        default:
            return "Unknown Scene";
    }
}

// ---------------------------------------------------------------------------
// SpawnDynamicBox — 运行时生成一个动态方块
// ---------------------------------------------------------------------------
// 在场景中央上方生成一个随机颜色、随机初速的动态 Box。
// 只操作 World 数据层，物理后端会在下一次 SyncAuthoringToRuntime 时自动发现。
ObjectId SpawnDynamicBox(World& world) {
    static thread_local std::mt19937 rng{std::random_device{}()};

    std::uniform_real_distribution<float> pos_xy(-2.0f, 2.0f);   // 水平位置抖动
    std::uniform_real_distribution<float> pos_y(8.0f, 12.0f);    // 从高处下落
    std::uniform_real_distribution<float> vel_h(-1.5f, 1.5f);    // 水平初速
    std::uniform_real_distribution<float> vel_v(-2.0f, 0.0f);    // 微向下的初速
    std::uniform_real_distribution<float> half_ext(0.3f, 0.6f);  // 半尺寸随机
    std::uniform_int_distribution<int> color_ch(80, 230);        // 颜色通道

    static int spawn_counter = 0;
    const std::string name = "spawned_" + std::to_string(spawn_counter++);

    const Vec3 position{pos_xy(rng), pos_y(rng), pos_xy(rng)};
    const float he = half_ext(rng);
    const Vec3 half_extents{he, he, he};
    const ColorRGBA8 color{
        static_cast<std::uint8_t>(color_ch(rng)),
        static_cast<std::uint8_t>(color_ch(rng)),
        static_cast<std::uint8_t>(color_ch(rng)),
        255};

    const ObjectId id = MakeBox(world,
                                name.c_str(),
                                position,
                                half_extents,
                                MotionType::Dynamic,
                                color,
                                1.0f);

    // 设置随机初速度（偏小，不会飞出场地）
    world.body_state.linear_velocity[id] = Vec3{vel_h(rng), vel_v(rng), vel_h(rng)};

    // 初始化上一帧快照，避免第一帧插值闪烁
    world.transforms.previous_position[id] = position;
    world.transforms.previous_rotation[id] = IdentityQuat();
    world.transforms.previous_scale[id] = Vec3{1.0f, 1.0f, 1.0f};

    return id;
}

}  // namespace lab

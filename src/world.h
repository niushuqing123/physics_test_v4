// ============================================================================
// world.h — 世界数据定义（项目的核心层）
// ============================================================================
// World 是整个项目的"唯一真相源"（Single Source of Truth）。
// 所有模块——物理、渲染、场景——都通过 World 中的数据来沟通。
//
// 数据布局采用 SoA（Struct of Arrays）模式：
//   - 同类属性存储在连续的 vector 中，以 ObjectId 做索引
//   - 比传统 AoS（Array of Structs）更缓存友好，也更方便按需遍历
//
// 分层设计：
//   TransformSoA          → 空间变换（位置/旋转/缩放），含上一帧快照用于渲染插值
//   RenderSoA             → 纯渲染 authoring（显示形状、颜色、尺寸参数）
//   ColliderAuthoringSoA  → 纯碰撞 authoring（物理几何、局部偏移、局部旋转）
//   RigidBodyAuthoringSoA → 刚体"设定型"属性（质量、运动类型、重力开关）
//   RigidBodyStateSoA     → 刚体"运行时"状态（速度、睡眠、物理句柄）
//
// 这次改动的关键点：
//   - ShapeKind 仍然复用同一个枚举，便于 renderer / physics 共用形状名称
//   - 但从语义上，render 和 collider 已经正式拆开：
//       render    只负责“看起来是什么”
//       colliders 只负责“撞起来是什么”
//   - 当前场景辅助函数仍然会同时填写 render 和 colliders，
//     这是为了平滑过渡，不代表两者未来必须永远绑定。
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "common.h"

namespace lab {

// ---------------------------------------------------------------------------
// 枚举：形状类型与运动类型
// ---------------------------------------------------------------------------

// 形状枚举：可同时用于 render 与 collider，但两者的 authoring 数据已分离。
enum class ShapeKind : std::uint8_t {
    Box,      // 盒子——由 box_half_extents 定义半尺寸
    Sphere,   // 球体——由 sphere_radius 定义半径
    Capsule,  // 胶囊体——由 capsule_radius + capsule_half_height 定义
};

// 刚体的运动类型，决定物理后端如何处理该对象
enum class MotionType : std::uint8_t {
    Static,     // 静态：不会移动，质量无限大，用于地面/墙壁
    Dynamic,    // 动态：受力驱动，参与碰撞响应
    Kinematic,  // 运动学：由代码控制位置，不被物理力影响，但可以推动动态物体
};

// ---------------------------------------------------------------------------
// SoA 数据容器
// ---------------------------------------------------------------------------

// 变换数据——每个对象的空间信息
// 同时保存"上一帧"快照，供渲染插值使用
struct TransformSoA {
    // --- 当前帧 ---
    std::vector<Vec3> position;     // 世界空间位置（对象/渲染原点）
    std::vector<Quat> rotation;     // 世界空间旋转
    std::vector<Vec3> scale;        // 各轴缩放

    // --- 上一帧快照（用于渲染插值）---
    // CapturePreviousTransforms() 在每个物理 tick 开始前调用
    std::vector<Vec3> previous_position;
    std::vector<Quat> previous_rotation;
    std::vector<Vec3> previous_scale;
};

// 渲染组件数据——决定对象在画面上的外观
// 注意：它不再承担“碰撞体定义”的职责。
struct RenderSoA {
    std::vector<std::uint8_t> enabled;      // 是否可见（0/1）
    std::vector<ShapeKind> shape;           // 显示形状
    std::vector<Vec3> box_half_extents;     // Box 的半尺寸 (hx, hy, hz)
    std::vector<float> sphere_radius;       // Sphere 的半径
    std::vector<float> capsule_radius;      // Capsule 的截面半径
    std::vector<float> capsule_half_height; // Capsule 的半高（不含端球）
    std::vector<ColorRGBA8> color;          // 显示颜色 RGBA
};

// 碰撞组件数据——决定对象在物理里使用什么几何
// 这是本轮新增的 authoring SoA，用来把物理几何从 render 中独立出来。
struct ColliderAuthoringSoA {
    std::vector<std::uint8_t> enabled;      // 是否拥有碰撞组件（0/1）
    std::vector<ShapeKind> shape;           // 碰撞几何种类
    std::vector<Vec3> box_half_extents;     // Box 的半尺寸 (hx, hy, hz)
    std::vector<float> sphere_radius;       // Sphere 的半径
    std::vector<float> capsule_radius;      // Capsule 的截面半径
    std::vector<float> capsule_half_height; // Capsule 的半高（不含端球）

    // 相对于 Transform 原点的局部碰撞体位姿。
    // v1 先全部默认为零偏移 + 单位旋转，后续 ragdoll / 偏心体会用到。
    std::vector<Vec3> local_offset;
    std::vector<Quat> local_rotation;
};

// 刚体"创作/设定"数据——在场景构建时确定，运行时一般不变
struct RigidBodyAuthoringSoA {
    std::vector<std::uint8_t> enabled;      // 是否拥有刚体组件（0/1）
    std::vector<MotionType> motion_type;    // 运动类型
    std::vector<float> mass;                // 质量（kg），Static 时为 0
    std::vector<std::uint8_t> use_gravity;  // 是否受重力影响（0/1）

    // 质心偏移：COM 相对 Transform 原点（body-local 坐标）。
    // Null 后端：运行时 position_ = COM 世界位置，Sync 时用此偏移做 COM ↔ Transform 换算。
    // Jolt 后端：传入 OffsetCenterOfMassShape，Jolt 内部自动处理。
    std::vector<Vec3> center_of_mass_offset;

    std::vector<float> friction;            // 摩擦系数（0=无摩擦, 1=高摩擦）
    std::vector<float> linear_damping;      // 线性速度阻尼（模拟空气阻力）
    std::vector<float> angular_damping;     // 角速度阻尼（防止滚动体永动）
};

// 刚体"运行时"状态——由物理后端写入，渲染/调试读取
struct RigidBodyStateSoA {
    std::vector<Vec3> linear_velocity;      // 线速度 (m/s)
    std::vector<Vec3> angular_velocity;     // 角速度 (rad/s)
    std::vector<std::uint8_t> sleeping;     // 是否处于睡眠状态（0=活跃, 1=睡眠）
    std::vector<PhysicsBodyHandle> handle;  // 物理后端内部句柄，调试用
};

// ---------------------------------------------------------------------------
// World 类——核心数据容器
// ---------------------------------------------------------------------------

class World {
public:
    // 清空所有数据，用于场景重建前。
    // 调用方：BuildScene()（scenes.cpp）在构建新场景前先调用以清空旧数据。
    void Clear();

    // 创建一个新对象并分配其 ObjectId（即 SoA 索引）
    // 所有 SoA 数组同步 push_back，保证索引一致性。
    // 调用方：scenes.cpp 的 MakeBox/MakeSphere/MakeCapsule 和 SpawnDynamicBox()。
    ObjectId CreateObject(const std::string& name);

    // 当前对象总数（含已销毁的槽位）
    [[nodiscard]] std::size_t ObjectCount() const;
    // 检查给定 ID 是否有效且存活
    [[nodiscard]] bool IsValid(ObjectId id) const;

    // 将当前帧变换拷贝到 previous_xxx 数组，用于渲染插值。
    // 调用方：AdvancePhysicsOneTick()（main.cpp）在每个物理 tick 最开始调用。
    void CapturePreviousTransforms();

    // --- Transform 存取 ---
    [[nodiscard]] Transform GetTransform(ObjectId id) const;
    [[nodiscard]] Transform GetPreviousTransform(ObjectId id) const;
    // 获取用于显示的变换：若 interpolate 为 true，则在上一帧和当前帧之间按 alpha 插值。
    // 调用方：renderer.cpp 的 DrawWorld() 和 PickObject() 用于计算渲染位姿和射线检测。
    [[nodiscard]] Transform GetDisplayTransform(ObjectId id, float alpha, bool interpolate) const;
    void SetTransform(ObjectId id, const Transform& transform);

    // --- 组件查询 ---
    // 是否有渲染组件；调用方：renderer.cpp 的 DrawWorld() / PickObject()
    [[nodiscard]] bool HasRenderable(ObjectId id) const;
    // 是否有碰撞组件；调用方：物理后端在 RebuildFromWorld / SyncAuthoringToRuntime 时过滤对象
    [[nodiscard]] bool HasCollider(ObjectId id) const;
    // 是否有刚体组件；调用方：物理后端；CountBodies() 内部
    [[nodiscard]] bool HasRigidBody(ObjectId id) const;
    // 统计指定运动类型的刚体数量。
    // 调用方：renderer.cpp 的 DrawOverlay() 在 HUD 中展示 Static/Dynamic/Kinematic 对象数。
    [[nodiscard]] std::size_t CountBodies(MotionType type) const;

    // --- 公开 SoA 数据 ---
    // 设计上有意公开，方便各模块直接按 ObjectId 索引读写。
    std::vector<std::string> names;   // 对象名称（调试用）
    std::vector<std::uint8_t> alive;  // 存活标志（0=已销毁, 1=存活）

    TransformSoA transforms;          // 空间变换数据
    RenderSoA render;                 // 渲染组件数据（只负责显示）
    ColliderAuthoringSoA colliders;   // 碰撞组件数据（只负责物理几何）
    RigidBodyAuthoringSoA bodies;     // 刚体设定数据
    RigidBodyStateSoA body_state;     // 刚体运行时状态
};

// --- 调试用名称转换 ---
const char* ShapeKindName(ShapeKind shape);
const char* MotionTypeName(MotionType motion_type);

}  // namespace lab

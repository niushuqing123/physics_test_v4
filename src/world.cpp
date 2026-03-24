// ============================================================================
// world.cpp — World 类的实现
// ============================================================================
// 本文件实现了 World 类的所有方法：对象创建/销毁、变换存取、
// 组件查询、以及用于调试显示的名称转换。
//
// 关键实现细节：
//   - CreateObject() 保证所有 SoA 数组同步增长，ObjectId 即数组索引
//   - Clear() 清空所有数组，用于场景重建
//   - CapturePreviousTransforms() 利用 vector 赋值做整体快照
//   - 本轮新增的 colliders SoA 与 render / bodies 一样，统一在这里初始化
// ==========================================================================
#include "world.h"

namespace lab {

// ---------------------------------------------------------------------------
// Clear — 清空世界中所有对象和数据
// ---------------------------------------------------------------------------
// 在加载新场景前调用，确保所有 SoA 数组回到空状态。
// 调用方：scenes.cpp 的 BuildBoxStack() / BuildRagdollPrep() 开头第一行。
void World::Clear() {
    names.clear();
    alive.clear();

    // 清空变换数据（当前帧 + 上一帧快照）
    transforms.position.clear();
    transforms.rotation.clear();
    transforms.scale.clear();
    transforms.previous_position.clear();
    transforms.previous_rotation.clear();
    transforms.previous_scale.clear();

    // 清空渲染组件
    render.enabled.clear();
    render.shape.clear();
    render.box_half_extents.clear();
    render.sphere_radius.clear();
    render.capsule_radius.clear();
    render.capsule_half_height.clear();
    render.color.clear();

    // 清空碰撞组件
    colliders.enabled.clear();
    colliders.shape.clear();
    colliders.box_half_extents.clear();
    colliders.sphere_radius.clear();
    colliders.capsule_radius.clear();
    colliders.capsule_half_height.clear();
    colliders.local_offset.clear();
    colliders.local_rotation.clear();

    // 清空刚体设定数据
    bodies.enabled.clear();
    bodies.motion_type.clear();
    bodies.mass.clear();
    bodies.use_gravity.clear();
    bodies.center_of_mass_offset.clear();
    bodies.friction.clear();
    bodies.linear_damping.clear();
    bodies.angular_damping.clear();

    // 清空刚体运行时状态
    body_state.linear_velocity.clear();
    body_state.angular_velocity.clear();
    body_state.sleeping.clear();
    body_state.handle.clear();
}

// ---------------------------------------------------------------------------
// CreateObject — 创建一个新对象，返回其 ObjectId
// ---------------------------------------------------------------------------
// 所有 SoA 数组同步 push_back 一个默认值条目。
// 新对象默认：无 render / collider / rigidbody 组件。
// 调用方：scenes.cpp 的 MakeBox/MakeSphere/MakeCapsule 和 SpawnDynamicBox()。
ObjectId World::CreateObject(const std::string& name) {
    // ObjectId = 当前数组大小，保证与数组索引对齐
    const ObjectId id = static_cast<ObjectId>(names.size());

    names.push_back(name);
    alive.push_back(1);   // 新对象默认存活

    // 变换：默认在原点，无旋转，单位缩放
    transforms.position.push_back(Vec3{0.0f, 0.0f, 0.0f});
    transforms.rotation.push_back(IdentityQuat());
    transforms.scale.push_back(Vec3{1.0f, 1.0f, 1.0f});
    // 上一帧快照也初始化为相同值
    transforms.previous_position.push_back(Vec3{0.0f, 0.0f, 0.0f});
    transforms.previous_rotation.push_back(IdentityQuat());
    transforms.previous_scale.push_back(Vec3{1.0f, 1.0f, 1.0f});

    // 渲染组件：默认关闭，形状为 Box，灰色
    render.enabled.push_back(0);
    render.shape.push_back(ShapeKind::Box);
    render.box_half_extents.push_back(Vec3{0.5f, 0.5f, 0.5f});
    render.sphere_radius.push_back(0.5f);
    render.capsule_radius.push_back(0.25f);
    render.capsule_half_height.push_back(0.5f);
    render.color.push_back(ColorRGBA8{200, 200, 200, 255});

    // 碰撞组件：默认关闭，几何默认也给一个合法 Box，便于后续显式启用
    colliders.enabled.push_back(0);
    colliders.shape.push_back(ShapeKind::Box);
    colliders.box_half_extents.push_back(Vec3{0.5f, 0.5f, 0.5f});
    colliders.sphere_radius.push_back(0.5f);
    colliders.capsule_radius.push_back(0.25f);
    colliders.capsule_half_height.push_back(0.5f);
    colliders.local_offset.push_back(Vec3{0.0f, 0.0f, 0.0f});
    colliders.local_rotation.push_back(IdentityQuat());

    // 刚体设定：默认关闭，静态，1kg，受重力
    // 阻尼/摩擦默认值适合方块类物体；球和胶囊在场景层会覆盖角阻尼
    bodies.enabled.push_back(0);
    bodies.motion_type.push_back(MotionType::Static);
    bodies.mass.push_back(1.0f);
    bodies.use_gravity.push_back(1);
    bodies.center_of_mass_offset.push_back(Vec3{0.0f, 0.0f, 0.0f});
    bodies.friction.push_back(0.6f);        // 适中摩擦，防止无限滑动
    bodies.linear_damping.push_back(0.05f); // 轻微线性阻尼（几乎无感）
    bodies.angular_damping.push_back(0.1f); // 轻微角阻尼

    // 刚体运行时状态：零速度，未睡眠，无效句柄
    body_state.linear_velocity.push_back(Vec3{0.0f, 0.0f, 0.0f});
    body_state.angular_velocity.push_back(Vec3{0.0f, 0.0f, 0.0f});
    body_state.sleeping.push_back(0);
    body_state.handle.push_back(kInvalidBodyHandle);

    return id;
}

// ---------------------------------------------------------------------------
// ObjectCount — 返回已分配的对象槽位总数
// ---------------------------------------------------------------------------
std::size_t World::ObjectCount() const {
    return names.size();
}

// ---------------------------------------------------------------------------
// IsValid — 检查 ObjectId 是否有效且对象存活
// ---------------------------------------------------------------------------
bool World::IsValid(ObjectId id) const {
    return id < names.size() && alive[id] != 0;
}

// ---------------------------------------------------------------------------
// CapturePreviousTransforms — 保存当前帧变换到"上一帧"快照
// ---------------------------------------------------------------------------
// 在每个物理 tick 开始前调用。渲染时通过 GetDisplayTransform() 在
// previous 和 current 之间插值，实现平滑显示。
// 调用方：main.cpp 的 AdvancePhysicsOneTick() 在 SyncAuthoringToRuntime 之前调用。
void World::CapturePreviousTransforms() {
    transforms.previous_position = transforms.position;
    transforms.previous_rotation = transforms.rotation;
    transforms.previous_scale = transforms.scale;
}

// ---------------------------------------------------------------------------
// Transform 存取方法
// ---------------------------------------------------------------------------

// 获取当前帧的变换
Transform World::GetTransform(ObjectId id) const {
    return Transform{
        transforms.position[id],
        transforms.rotation[id],
        transforms.scale[id],
    };
}

// 获取上一帧的变换（用于插值）
Transform World::GetPreviousTransform(ObjectId id) const {
    return Transform{
        transforms.previous_position[id],
        transforms.previous_rotation[id],
        transforms.previous_scale[id],
    };
}

// 获取用于显示的变换
// 当 interpolate=true 时，在上一帧和当前帧之间按 alpha 做线性/Nlerp 插值
// alpha=0 → 上一帧状态，alpha=1 → 当前帧状态
Transform World::GetDisplayTransform(ObjectId id, float alpha, bool interpolate) const {
    if (!interpolate) {
        return GetTransform(id);
    }
    return Interpolate(GetPreviousTransform(id), GetTransform(id), Clamp(alpha, 0.0f, 1.0f));
}

// 设置变换（旋转会被归一化以防止数值漂移）
void World::SetTransform(ObjectId id, const Transform& transform) {
    transforms.position[id] = transform.position;
    transforms.rotation[id] = Normalize(transform.rotation);
    transforms.scale[id] = transform.scale;
}

// ---------------------------------------------------------------------------
// 组件查询
// ---------------------------------------------------------------------------

// 判断对象是否拥有可渲染组件
bool World::HasRenderable(ObjectId id) const {
    return IsValid(id) && render.enabled[id] != 0;
}

// 判断对象是否拥有碰撞组件。
// 调用方：物理后端在 RebuildFromWorld 和 SyncAuthoringToRuntime 时
//         遍历对象并通过此函数过滤出需要注册碰撞体的对象。
bool World::HasCollider(ObjectId id) const {
    return IsValid(id) && colliders.enabled[id] != 0;
}

// 判断对象是否拥有刚体组件
bool World::HasRigidBody(ObjectId id) const {
    return IsValid(id) && bodies.enabled[id] != 0;
}

// 统计某种运动类型的刚体数量（用于 HUD 显示）。
// 调用方：renderer.cpp 的 DrawOverlay() 展示 Static/Dynamic/Kinematic 对象数。
std::size_t World::CountBodies(MotionType type) const {
    std::size_t count = 0;
    for (ObjectId id = 0; id < static_cast<ObjectId>(names.size()); ++id) {
        if (HasRigidBody(id) && bodies.motion_type[id] == type) {
            ++count;
        }
    }
    return count;
}

// ---------------------------------------------------------------------------
// 调试用名称转换函数
// ---------------------------------------------------------------------------

// 形状种类 → 可读字符串
const char* ShapeKindName(ShapeKind shape) {
    switch (shape) {
        case ShapeKind::Box:
            return "Box";
        case ShapeKind::Sphere:
            return "Sphere";
        case ShapeKind::Capsule:
            return "Capsule";
        default:
            return "Unknown";
    }
}

// 运动类型 → 可读字符串
const char* MotionTypeName(MotionType motion_type) {
    switch (motion_type) {
        case MotionType::Static:
            return "Static";
        case MotionType::Dynamic:
            return "Dynamic";
        case MotionType::Kinematic:
            return "Kinematic";
        default:
            return "Unknown";
    }
}

}  // namespace lab

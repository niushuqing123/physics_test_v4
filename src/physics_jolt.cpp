// ============================================================================
// physics_jolt.cpp — Jolt Physics 后端实现（"正确答案"对照陪跑）
// ============================================================================
// 本文件将 Jolt Physics 引擎封装为 IPhysicsBackend 实现，
// 作为你手写物理引擎的参考对照。
//
// ── 坐标语义约定（Jolt 后端 ↔ World 之间的映射）──
//
// World 层：
//   transforms.position   = Transform 原点（渲染/编辑器原点）
//   transforms.rotation   = 世界旋转
//   bodies.center_of_mass_offset = COM 相对 Transform 原点的局部偏移
//   colliders.local_offset       = 碰撞几何中心相对 Transform 原点的局部偏移
//   colliders.local_rotation     = 碰撞几何相对 body frame 的局部旋转
//
// Jolt 层：
//   Body position  = body frame 原点（不是 COM！）
//   Body rotation  = body frame 旋转
//   Shape::GetCenterOfMass() = COM 在 body-local space 的位置
//   RotatedTranslatedShape   = 碰撞几何相对 body frame 的局部位姿
//   OffsetCenterOfMassShape  = 在不改变碰撞几何的前提下移动物理 COM
//
// 映射规则：
//   Body position  ↔ transforms.position      （1:1 对应）
//   Body rotation  ↔ transforms.rotation       （1:1 对应）
//   Shape local    ← colliders.local_offset/rotation  （RotatedTranslatedShape）
//   Shape COM      ← bodies.center_of_mass_offset     （OffsetCenterOfMassShape）
//
// 不做的事：
//   - 不把 Body position 设成 COM 位置（与 Null 后端的 position_ 语义不同，
//     但 SyncRuntimeToWorld 的最终结果一致——都回写到 transforms.position）
//   - 不在 SyncRuntimeToWorld 里做 COM↔Transform 换算（因为 Jolt 的 GetPosition
//     本身返回的就是 body frame 原点 = Transform 原点）
// ============================================================================
#include "physics_api.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

// 关闭 Jolt 内部的编译器警告
JPH_SUPPRESS_WARNINGS

namespace lab {
namespace {

// ===========================================================================
// Jolt 辅助工具：回调函数、层定义、过滤器
// ===========================================================================

// Jolt 的 Trace 输出回调——将 Jolt 内部的调试输出重定向到 stdout
static void JoltTraceImpl(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
}

#ifdef JPH_ENABLE_ASSERTS
// Jolt 的断言失败回调——打印断言信息并触发断点
static bool JoltAssertFailed(const char* expression, const char* message, const char* file, unsigned int line) {
    printf("[Jolt Assert] %s:%u: (%s) %s\n", file, line, expression, message ? message : "");
    return true;  // 返回 true 触发调试器断点
}
#endif

// ---------------------------------------------------------------------------
// Broadphase 层定义
// ---------------------------------------------------------------------------
// Jolt 使用两级过滤：先按 BroadPhaseLayer 粗筛，再按 ObjectLayer 细筛。
// 这里简单分两层：NON_MOVING（静态物体）和 MOVING（动态/运动学物体）。
// Broadphase layers
namespace BroadPhaseLayers {
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr unsigned int NUM_LAYERS = 2;
}

// ---------------------------------------------------------------------------
// Object 层定义
// ---------------------------------------------------------------------------
// ObjectLayer 决定了一个 Body 属于哪个物理层。
// NON_MOVING=0 用于静态物体，MOVING=1 用于动态和运动学物体。
namespace ObjectLayers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr unsigned int NUM_LAYERS = 2;
}

// ---------------------------------------------------------------------------
// BPLayerInterfaceImpl — 告诉 Jolt 如何把 ObjectLayer 映射到 BroadPhaseLayer
// ---------------------------------------------------------------------------
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    unsigned int GetNumBroadPhaseLayers() const override {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        switch (layer) {
            case ObjectLayers::NON_MOVING: return BroadPhaseLayers::NON_MOVING;
            case ObjectLayers::MOVING:     return BroadPhaseLayers::MOVING;
            default:                       return BroadPhaseLayers::MOVING;
        }
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        switch ((JPH::BroadPhaseLayer::Type)layer) {
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:     return "MOVING";
            default:                                                        return "UNKNOWN";
        }
    }
#endif
};

// ---------------------------------------------------------------------------
// ObjectVsBroadPhaseLayerFilterImpl — Object 层 vs BroadPhase 层的碰撞过滤
// ---------------------------------------------------------------------------
// 规则：非移动物体之间不碰撞（静态 vs 静态没有意义）。
class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer obj_layer, JPH::BroadPhaseLayer bp_layer) const override {
        switch (obj_layer) {
            case ObjectLayers::NON_MOVING:
                return bp_layer == BroadPhaseLayers::MOVING;
            case ObjectLayers::MOVING:
                return true;
            default:
                return false;
        }
    }
};

// ---------------------------------------------------------------------------
// ObjectLayerPairFilterImpl — 两个 Object 层之间的碰撞过滤
// ---------------------------------------------------------------------------
// 规则：NON_MOVING 只与 MOVING 碰撞，MOVING 与所有层碰撞。
class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer obj1, JPH::ObjectLayer obj2) const override {
        switch (obj1) {
            case ObjectLayers::NON_MOVING:
                return obj2 == ObjectLayers::MOVING;
            case ObjectLayers::MOVING:
                return true;
            default:
                return false;
        }
    }
};

// ---------------------------------------------------------------------------
// 类型转换辅助——lab 类型 ↔ Jolt 类型
// ---------------------------------------------------------------------------

inline JPH::Vec3 ToJolt(Vec3 v) { return JPH::Vec3(v.x, v.y, v.z); }   // lab::Vec3 → JPH::Vec3
inline JPH::Quat ToJolt(Quat q) { return JPH::Quat(q.x, q.y, q.z, q.w); } // lab::Quat → JPH::Quat
inline Vec3 FromJolt(JPH::Vec3 v) { return Vec3{v.GetX(), v.GetY(), v.GetZ()}; } // JPH::Vec3 → lab::Vec3
inline Quat FromJolt(JPH::Quat q) { return Quat{q.GetX(), q.GetY(), q.GetZ(), q.GetW()}; } // JPH::Quat → lab::Quat

inline bool IsNearlyZero(Vec3 v, float epsilon = 1e-6f) {
    return std::fabs(v.x) <= epsilon && std::fabs(v.y) <= epsilon && std::fabs(v.z) <= epsilon;
}

inline bool IsNearlyIdentity(Quat q, float epsilon = 1e-6f) {
    return std::fabs(q.x) <= epsilon &&
           std::fabs(q.y) <= epsilon &&
           std::fabs(q.z) <= epsilon &&
           std::fabs(q.w - 1.0f) <= epsilon;
}

// lab::MotionType → Jolt 的 EMotionType
inline JPH::EMotionType ToJoltMotionType(MotionType mt) {
    switch (mt) {
        case MotionType::Static:    return JPH::EMotionType::Static;
        case MotionType::Dynamic:   return JPH::EMotionType::Dynamic;
        case MotionType::Kinematic: return JPH::EMotionType::Kinematic;
        default:                    return JPH::EMotionType::Static;
    }
}

// 根据运动类型决定 Jolt ObjectLayer：Static → NON_MOVING，其余 → MOVING
inline JPH::ObjectLayer ToJoltObjectLayer(MotionType mt) {
    return (mt == MotionType::Static) ? ObjectLayers::NON_MOVING : ObjectLayers::MOVING;
}

// ---------------------------------------------------------------------------
// Jolt 全局初始化守卫
// ---------------------------------------------------------------------------
// Jolt 的类型注册和内存分配器只能初始化一次（进程级）。
// 使用静态标志确保不会重复调用。
static bool s_jolt_global_initialized = false;

static void EnsureJoltGlobalInit() {
    if (s_jolt_global_initialized) return;
    JPH::RegisterDefaultAllocator();
    JPH::Trace = JoltTraceImpl;
#ifdef JPH_ENABLE_ASSERTS
    JPH::AssertFailed = JoltAssertFailed;
#endif
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
    s_jolt_global_initialized = true;
}

// ===========================================================================
// JoltPhysicsBackend — IPhysicsBackend 的 Jolt 实现
// ===========================================================================
// 将 Jolt Physics 封装为统一的后端接口，作为手写物理的"正确答案"对照。
// 内部维护:
//   - PhysicsSystem: Jolt 的核心物理系统
//   - TempAllocator: 每帧临时内存分配器
//   - JobSystem: 多线程任务系统
//   - 双向映射表: lab ObjectId ↔ Jolt BodyID

class JoltPhysicsBackend final : public IPhysicsBackend {
public:
    // 返回后端名称
    const char* Name() const override {
        return "JoltPhysicsBackend";
    }

    // -----------------------------------------------------------------------
    // Initialize — 初始化 Jolt 物理系统
    // -----------------------------------------------------------------------
    // 1. 确保全局注册只执行一次
    // 2. 创建临时内存分配器（10MB）和线程池（1个工作线程）
    // 3. 初始化 PhysicsSystem，设置容量上限
    // 4. 从 World 创建所有物理体
    void Initialize(const World& world, const PhysicsConfig& config) override {
        EnsureJoltGlobalInit();

        config_ = config;

        temp_allocator_ = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);
        job_system_ = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 1);

        constexpr unsigned int kMaxBodies      = 4096;
        constexpr unsigned int kNumBodyMutexes = 0;  // auto
        constexpr unsigned int kMaxBodyPairs   = 4096;
        constexpr unsigned int kMaxContacts    = 4096;

        physics_system_ = std::make_unique<JPH::PhysicsSystem>();
        physics_system_->Init(
            kMaxBodies, kNumBodyMutexes, kMaxBodyPairs, kMaxContacts,
            bp_layer_interface_, obj_vs_bp_filter_, obj_layer_pair_filter_);

        physics_system_->SetGravity(ToJolt(config_.gravity));

        RebuildFromWorld(world);
    }

    // -----------------------------------------------------------------------
    // Shutdown — 销毁所有物理体并释放 Jolt 资源
    // -----------------------------------------------------------------------
    void Shutdown() override {
        // 先移除并销毁所有 Body
        if (physics_system_) {
            JPH::BodyInterface& bi = physics_system_->GetBodyInterface();
            for (auto& [obj_id, body_id] : object_to_body_) {
                bi.RemoveBody(body_id);
                bi.DestroyBody(body_id);
            }
        }
        object_to_body_.clear();
        body_to_object_.clear();

        physics_system_.reset();
        job_system_.reset();
        temp_allocator_.reset();
    }

    // -----------------------------------------------------------------------
    // RebuildFromWorld — 从 World 完全重建所有 Jolt Body
    // -----------------------------------------------------------------------
    // 场景切换/重载时调用。先销毁旧 Body，再根据 World 数据重建。
    void RebuildFromWorld(const World& world) override {
        // 先移除并销毁所有已有的 Body
        if (physics_system_) {
            JPH::BodyInterface& bi = physics_system_->GetBodyInterface();
            for (auto& [obj_id, body_id] : object_to_body_) {
                bi.RemoveBody(body_id);
                bi.DestroyBody(body_id);
            }
        }
        object_to_body_.clear();
        body_to_object_.clear();

        if (!physics_system_) return;

        // 遍历 World 中所有有效的刚体对象，逐一创建 Jolt Body
        for (ObjectId id = 0; id < static_cast<ObjectId>(world.ObjectCount()); ++id) {
            if (!world.IsValid(id) || !world.HasRigidBody(id)) continue;
            AddSingleBody(world, id);
        }
    }

    // -----------------------------------------------------------------------
    // SyncAuthoringToRuntime — 同步静态/运动学物体的位姿到 Jolt
    // -----------------------------------------------------------------------
    // 1. 自动发现 World 中新增的刚体对象，创建对应的 Jolt Body
    // 2. 对 Static/Kinematic，把 World 编辑器位姿推送给 Jolt
    //    SetPositionAndRotation 的参数是 Body position（= Transform 原点），
    //    不是 COM 位置，因此直接使用 transforms.position/rotation。
    void SyncAuthoringToRuntime(const World& world) override {
        if (!physics_system_) return;
        JPH::BodyInterface& bi = physics_system_->GetBodyInterface();

        // --- 自动发现新增物体 ---
        for (ObjectId id = 0; id < static_cast<ObjectId>(world.ObjectCount()); ++id) {
            if (!world.IsValid(id) || !world.HasRigidBody(id)) continue;
            if (object_to_body_.count(id)) continue;
            AddSingleBody(world, id);
        }

        // --- 同步已知物体的位姿（Transform 原点 → Body position）---
        for (auto& [obj_id, body_id] : object_to_body_) {
            const MotionType mt = world.bodies.motion_type[obj_id];
            if (mt == MotionType::Static || mt == MotionType::Kinematic) {
                bi.SetPositionAndRotation(
                    body_id,
                    ToJolt(world.transforms.position[obj_id]),
                    ToJolt(world.transforms.rotation[obj_id]),
                    JPH::EActivation::DontActivate);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Step — 调用 Jolt 执行一步物理仿真
    // -----------------------------------------------------------------------
    // 碰撞步数设为 1，因为外层主循环已经处理了固定步长细分。
    // Jolt 内部会执行：宽相→窄相→约束求解→积分→睡眠判定 等完整管线。
    void Step(float dt) override {
        if (!physics_system_) return;
        physics_system_->Update(dt, 1, temp_allocator_.get(), job_system_.get());
    }

    // -----------------------------------------------------------------------
    // SyncRuntimeToWorld — 将 Jolt 仿真结果写回 World
    // -----------------------------------------------------------------------
    // 读取每个 Body 的最新位置/旋转/速度/活跃状态，写入 World 对应槽位。
    //
    // 关键语义：
    //   bi.GetPosition()    = body frame 原点 = Transform 原点（不是 COM）
    //   bi.GetLinearVelocity()  = COM 的线速度（Jolt 约定）
    //   bi.GetAngularVelocity() = 刚体角速度
    //
    // 因此 GetPosition() 可以直接写入 transforms.position，无需做 COM 换算。
    // （对比 Null 后端：position_ 存的是 COM 世界位置，回写 transforms.position
    //  时需要用 BodyPositionToWorldTransform 做反向换算。）
    void SyncRuntimeToWorld(World& world) override {
        if (!physics_system_) return;
        JPH::BodyInterface& bi = physics_system_->GetBodyInterface();

        for (auto& [obj_id, body_id] : object_to_body_) {
            const MotionType mt = world.bodies.motion_type[obj_id];
            world.body_state.handle[obj_id] = static_cast<PhysicsBodyHandle>(body_id.GetIndexAndSequenceNumber());
            // 注意：linear_velocity 是 COM 速度，这与 Null 后端一致。
            world.body_state.linear_velocity[obj_id]  = FromJolt(bi.GetLinearVelocity(body_id));
            world.body_state.angular_velocity[obj_id] = FromJolt(bi.GetAngularVelocity(body_id));
            world.body_state.sleeping[obj_id] = bi.IsActive(body_id) ? 0 : 1;

            if (mt == MotionType::Dynamic || mt == MotionType::Kinematic) {
                // GetPosition() = body frame origin = Transform 原点（直接写入，无需换算）
                world.transforms.position[obj_id] = FromJolt(bi.GetPosition(body_id));
                world.transforms.rotation[obj_id] = FromJolt(bi.GetRotation(body_id));
            }
        }
    }

    // 返回 Jolt 后端基础统计（宽相/接触/求解器数据不暴露，以 0 或 -1 表示不可用）
    PhysicsDebugStats GetDebugStats() const override {
        PhysicsDebugStats s;
        s.num_bodies = static_cast<int>(object_to_body_.size());
        if (physics_system_) {
            const JPH::BodyInterface& bi = physics_system_->GetBodyInterface();
            (void)bi; // 睡眠计数需要遍历，暂时省略
        }
        // 其他字段保持 0（Jolt 内部数据不经由此接口暴露）
        return s;
    }

private:
    // -----------------------------------------------------------------------
    // CreateShapeForObject — 三步构造碰撞 Shape
    // -----------------------------------------------------------------------
    //  Step 1: 创建基础几何（Box/Sphere/Capsule），COM 默认在几何中心。
    //  Step 2: 若 collider.local_offset 或 local_rotation 非默认，
    //          用 RotatedTranslatedShape 把几何体偏到 body frame 里的指定位置。
    //  Step 3: 若 bodies.center_of_mass_offset 非零，
    //          用 OffsetCenterOfMassShape 把物理 COM 移到指定位置，
    //          而不改变碰撞几何的位置。
    //
    // 之后 AddSingleBody 用 Body position = transforms.position (Transform 原点)
    // → Jolt 物理 COM = body_pos + Rotate(body_rot, shape->GetCenterOfMass())
    //   其中 shape->GetCenterOfMass() == center_of_mass_offset（由 Step 3 保证）
    JPH::ShapeRefC CreateShapeForObject(const World& world, ObjectId id) {
        if (!world.HasCollider(id) || world.colliders.enabled[id] == 0) {
            return nullptr;
        }

        // --- Step 1: 基础几何 ---
        const ShapeKind shape_kind = world.colliders.shape[id];
        JPH::ShapeRefC base_shape;
        switch (shape_kind) {
            case ShapeKind::Box: {
                const Vec3 he = world.colliders.box_half_extents[id];
                base_shape = new JPH::BoxShape(JPH::Vec3(he.x, he.y, he.z));
                break;
            }
            case ShapeKind::Sphere: {
                base_shape = new JPH::SphereShape(world.colliders.sphere_radius[id]);
                break;
            }
            case ShapeKind::Capsule: {
                base_shape = new JPH::CapsuleShape(
                    world.colliders.capsule_half_height[id],
                    world.colliders.capsule_radius[id]);
                break;
            }
            default:
                return nullptr;
        }

        // --- Step 2: 碰撞几何的局部位姿（body frame 内偏移/旋转）---
        JPH::ShapeRefC collision_shape = base_shape;
        const Vec3 local_offset = world.colliders.local_offset[id];
        const Quat local_rotation = world.colliders.local_rotation[id];
        if (!IsNearlyZero(local_offset) || !IsNearlyIdentity(local_rotation)) {
            collision_shape = new JPH::RotatedTranslatedShape(
                ToJolt(local_offset), ToJolt(local_rotation), base_shape.GetPtr());
        }

        // --- Step 3: COM 偏移 ---
        // World 定义: center_of_mass_offset = COM 相对 Transform 原点，body-local 坐标。
        // Jolt 需要: shape->GetCenterOfMass() 返回相同语义的向量。
        // OffsetCenterOfMassShape(inner, delta) 使 GetCenterOfMass() = delta + inner->GetCenterOfMass()。
        // 因此 delta = desired_com - inner->GetCenterOfMass()。
        const Vec3 com_offset = world.bodies.center_of_mass_offset[id];
        if (!IsNearlyZero(com_offset)) {
            const JPH::Vec3 desired_com = ToJolt(com_offset);
            const JPH::Vec3 current_com = collision_shape->GetCenterOfMass();
            collision_shape = new JPH::OffsetCenterOfMassShape(
                collision_shape.GetPtr(), desired_com - current_com);
        }

        return collision_shape;
    }

    // -----------------------------------------------------------------------
    // AddSingleBody — 为 World 中的单个对象创建 Jolt Body 并加入物理世界
    // -----------------------------------------------------------------------
    // Body position = transforms.position = Transform 原点（不是 COM）。
    // Jolt 内部的物理 COM = body_pos + Rotate(body_rot, shape->GetCenterOfMass())。
    // shape->GetCenterOfMass() 已在 CreateShapeForObject 的 Step 3 中被设为
    // bodies.center_of_mass_offset，因此物理 COM 与 World authoring 完全一致。
    void AddSingleBody(const World& world, ObjectId id) {
        if (!physics_system_) return;
        JPH::BodyInterface& bi = physics_system_->GetBodyInterface();

        JPH::ShapeRefC shape = CreateShapeForObject(world, id);
        if (!shape) return;

        const MotionType mt = world.bodies.motion_type[id];
        // Body position = Transform 原点，Body rotation = Transform 旋转
        JPH::BodyCreationSettings settings(
            shape,
            ToJolt(world.transforms.position[id]),
            ToJolt(world.transforms.rotation[id]),
            ToJoltMotionType(mt),
            ToJoltObjectLayer(mt));

        if (mt == MotionType::Dynamic) {
            const float mass = world.bodies.mass[id];
            if (mass > 1e-6f) {
                settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
                settings.mMassPropertiesOverride.mMass = mass;
            }
            settings.mGravityFactor = world.bodies.use_gravity[id] ? 1.0f : 0.0f;
        }

        // 摩擦与阻尼——从 World authoring 数据读取，适用于所有运动类型
        settings.mFriction        = world.bodies.friction[id];
        settings.mLinearDamping   = world.bodies.linear_damping[id];
        settings.mAngularDamping  = world.bodies.angular_damping[id];

        settings.mLinearVelocity  = ToJolt(world.body_state.linear_velocity[id]);
        settings.mAngularVelocity = ToJolt(world.body_state.angular_velocity[id]);

        JPH::Body* body = bi.CreateBody(settings);
        if (!body) return;

        JPH::BodyID body_id = body->GetID();
        bi.AddBody(body_id,
                   mt == MotionType::Dynamic ? JPH::EActivation::Activate
                                             : JPH::EActivation::DontActivate);

        object_to_body_[id] = body_id;
        body_to_object_[body_id] = id;
    }

    PhysicsConfig config_{};  // 缓存的物理配置

    // --- Jolt 子系统 ---
    BPLayerInterfaceImpl              bp_layer_interface_;    // BroadPhase 层映射
    ObjectVsBroadPhaseLayerFilterImpl obj_vs_bp_filter_;      // Object vs BroadPhase 碰撞过滤
    ObjectLayerPairFilterImpl         obj_layer_pair_filter_; // Object 层对碰撞过滤

    std::unique_ptr<JPH::TempAllocatorImpl>    temp_allocator_;  // 每帧临时内存分配器（10MB）
    std::unique_ptr<JPH::JobSystemThreadPool>  job_system_;      // 多线程任务调度系统
    std::unique_ptr<JPH::PhysicsSystem>        physics_system_;  // Jolt 核心物理系统

    // --- 双向映射表：lab ObjectId ↔ Jolt BodyID ---
    // 用于在 SyncRuntimeToWorld 和 RebuildFromWorld 中快速查找对应关系
    std::unordered_map<ObjectId, JPH::BodyID>  object_to_body_;
    std::unordered_map<JPH::BodyID, ObjectId>  body_to_object_;
};

}  // namespace

// 工厂函数：创建 JoltPhysicsBackend 实例
std::unique_ptr<IPhysicsBackend> CreateJoltPhysicsBackend() {
    return std::make_unique<JoltPhysicsBackend>();
}

}  // namespace lab

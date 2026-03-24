// ============================================================================
// physics_api.h — 物理后端的稳定接口层
// ============================================================================
// 本文件定义了物理后端的抽象接口 IPhysicsBackend 以及物理配置结构体。
//
// 设计意图：
//   - 所有物理实现（NullPhysicsBackend、JoltPhysicsBackend、你自己的求解器）
//     都继承自 IPhysicsBackend，外部代码只持有 unique_ptr<IPhysicsBackend>
//   - 接口方法遵循一套固定的生命周期：
//     Initialize → (RebuildFromWorld ↔ SyncAuthoringToRuntime → Step → SyncRuntimeToWorld)* → Shutdown
//   - 切换物理后端只需替换工厂函数调用，不影响 World / Renderer / 主循环
//
// 数据流：
//   World (authoring) ──SyncAuthoringToRuntime──► Physics (runtime)
//   Physics (runtime) ──SyncRuntimeToWorld──────► World (state)
// ============================================================================
#pragma once

#include <memory>
#include <vector>

#include "world.h"

namespace lab {

// ---------------------------------------------------------------------------
// PhysicsDebugStats — 物理后端本帧的统计摘要
// ---------------------------------------------------------------------------
// 后端实现覆盖 GetDebugStats() 以填充此结构体；默认实现返回全零。
struct PhysicsDebugStats {
    int num_bodies = 0;                   // 总刚体数
    int num_sleeping = 0;                 // 休眠刚体数
    int num_broadphase_pairs = 0;         // 宽相候选对数
    int num_contacts = 0;                 // 实际接触点数
    int solver_velocity_iterations = 0;   // 速度求解迭代次数（仅 Null 后端有效）
    int solver_position_iterations = 0;   // 位置求解迭代次数（仅 Null 后端有效）
};

// ---------------------------------------------------------------------------
// PhysicsDebugDrawData — 物理后端本帧的可视化调试数据
// ---------------------------------------------------------------------------
// 后端实现覆盖 CollectDebugDrawData() 以填充此结构体；默认实现不写入任何数据。
// Renderer 根据 DebugDrawOptions 决定分类展示哪些数据。
struct PhysicsDebugDrawData {
    // 接触点信息（每个接触点一条记录）
    struct ContactPoint {
        Vec3 position;        // 接触点世界坐标
        Vec3 normal;          // 碰撞法线（从 a 指向 b，单位向量）
        float penetration = 0.0f;  // 穿透深度
    };
    // 刚体质心（COM）与 Transform 原点信息
    struct BodyCom {
        Vec3 transform_origin;  // Transform origin 世界坐标（对应 world.transforms.position）
        Vec3 com_world;         // 质心（COM）世界坐标
    };
    // Collider 局部坐标系在世界空间的位姿
    struct ColliderFrame {
        Vec3 center_world;    // Collider 中心世界坐标
        Quat rotation_world;  // Collider 旋转（world-space）
    };
    // 物理后端内部 AABB（可能与渲染 AABB 不同）
    struct PhysicsAabb {
        Vec3 min_world;
        Vec3 max_world;
    };

    std::vector<ContactPoint>  contacts;
    std::vector<BodyCom>       body_coms;
    std::vector<ColliderFrame> collider_frames;
    std::vector<PhysicsAabb>   aabbs;

    void Clear() {
        contacts.clear();
        body_coms.clear();
        collider_frames.clear();
        aabbs.clear();
    }
};

// ---------------------------------------------------------------------------
// PhysicsConfig — 物理仿真的全局配置参数
// ---------------------------------------------------------------------------
struct PhysicsConfig {
    float fixed_dt = 1.0f / 60.0f;       // 固定物理步长（秒），默认 60Hz
    int max_substeps_per_frame = 4;       // 每帧最大子步数，防止 spiral of death
    Vec3 gravity{0.0f, -9.81f, 0.0f};    // 全局重力加速度 (m/s²)
};

// ---------------------------------------------------------------------------
// IPhysicsBackend — 物理后端抽象接口
// ---------------------------------------------------------------------------
// 所有物理后端必须实现此接口。主循环通过此接口驱动物理仿真。
class IPhysicsBackend {
public:
    virtual ~IPhysicsBackend() = default;

    // 返回后端名称，用于 HUD 显示（如 "NullPhysicsBackend" / "JoltPhysicsBackend"）
    virtual const char* Name() const = 0;

    // 初始化物理后端。传入世界数据和配置，创建内部资源。
    // 调用一次，在首次 RebuildFromWorld 之前。
    // 调用方：main.cpp 的 ReloadScene() 在首次加载场景时调用（后续改用 RebuildFromWorld）。
    virtual void Initialize(const World& world, const PhysicsConfig& config) = 0;

    // 销毁内部资源，释放所有物理对象。
    // 调用方：main() 退出前；P 键切换后端时先 Shutdown 再重建。
    virtual void Shutdown() = 0;

    // 从 World 数据完全重建物理后端内部状态。
    // 用于场景切换/重载——先销毁所有旧的物理体，再根据 World 重新创建。
    // 调用方：main.cpp 的 ReloadScene() 在非首次场景切换时调用。
    virtual void RebuildFromWorld(const World& world) = 0;

    // 将 World 的"设定型"数据（authoring）同步到物理后端的运行时数据。
    // 主要同步静态/运动学物体的位置旋转，动态物体一般不覆盖（除非 teleport）。
    // 调用方：main.cpp 的 AdvancePhysicsOneTick()，在 Step() 之前调用。
    virtual void SyncAuthoringToRuntime(const World& world) = 0;

    // 执行一个固定步长的物理仿真推进。
    // dt 就是 PhysicsConfig::fixed_dt，由主循环的累加器控制调用频率。
    // 内部应包含：力的累计、宽相/窄相碰撞检测、约束求解、速度积分等。
    // 调用方：main.cpp 的 AdvancePhysicsOneTick()，每消耗一个累加器步长调用一次。
    virtual void Step(float dt) = 0;

    // 将物理后端的运行时状态写回 World。
    // 包括：动态物体的位置/旋转、线速度/角速度、睡眠状态、物理句柄等。
    // 调用方：AdvancePhysicsOneTick()（Step 之后）；ReloadScene() 初始化完成后也调用一次。
    virtual void SyncRuntimeToWorld(World& world) = 0;

    // ---------------------------------------------------------------------------
    // 调试接口（可选实现）
    // ---------------------------------------------------------------------------
    // 返回后端本帧的统计摘要（不实现则返回全零）。
    // 调用方：main.cpp 每帧在渲染前调用一次。
    virtual PhysicsDebugStats GetDebugStats() const { return {}; }

    // 将本帧可视化调试数据填入 out（不实现则什么都不写入）。
    // 调用方：main.cpp 每帧在渲染前调用一次，out 由调用方负责先 Clear()。
    virtual void CollectDebugDrawData(PhysicsDebugDrawData& out) const { (void)out; }
};

// ---------------------------------------------------------------------------
// 工厂函数——创建各物理后端实例
// ---------------------------------------------------------------------------

// 创建 NullPhysicsBackend v1（旧版实验实现）
std::unique_ptr<IPhysicsBackend> CreateNullPhysicsBackendV1();

// 创建 NullPhysicsBackend v2（当前实验实现）
std::unique_ptr<IPhysicsBackend> CreateNullPhysicsBackendV2();

// 创建 NullPhysicsBackend v3（新版实验实现）
std::unique_ptr<IPhysicsBackend> CreateNullPhysicsBackendV3();

// 创建 NullPhysicsBackend v3.1（增强版）
std::unique_ptr<IPhysicsBackend> CreateNullPhysicsBackendV3_1();

// 创建 NullPhysicsBackend v4（新版本）
std::unique_ptr<IPhysicsBackend> CreateNullPhysicsBackendV4();

// 创建 Jolt Physics 后端（作为"正确答案"对照陪跑）
std::unique_ptr<IPhysicsBackend> CreateJoltPhysicsBackend();

}  // namespace lab

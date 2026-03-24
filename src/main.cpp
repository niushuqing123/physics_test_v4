// ============================================================================
// main.cpp — 主循环、固定步长、输入处理、场景切换
// ============================================================================
// 本文件是程序的入口点，实现了：
//   1. AppState 结构体 — 集中管理所有应用状态
//   2. 固定时间步长物理循环 — 累加器 + 最大子步钳制
//   3. 键盘输入处理 — 场景切换、暂停/单步、调试开关
//   4. 每帧控制台调试输出 — 帧率、tick、对象状态
//
// 固定步长原理：
//   物理每次以 fixed_dt 推进，与帧率无关。
//   每帧将真实时间差累加到 accumulator，
//   当累加器 >= fixed_dt 时执行一次物理 Step。
//   单帧最多执行 max_substeps_per_frame 次以防止 spiral of death。
//   渲染时可用累加器余量做插值，实现平滑显示。
// ============================================================================
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>

#include <raylib.h>

#include "physics_api.h"
#include "renderer.h"
#include "scenes.h"
#include "world.h"

namespace {

using PhysicsBackendFactory = std::unique_ptr<lab::IPhysicsBackend> (*)();

struct PhysicsBackendOption {
    const char* name;
    PhysicsBackendFactory create;
};

constexpr std::array<PhysicsBackendOption, 6> kPhysicsBackendOptions{{
    {"JoltPhysicsBackend", &lab::CreateJoltPhysicsBackend},
    {"NullPhysicsBackendV1", &lab::CreateNullPhysicsBackendV1},
    {"NullPhysicsBackendV2", &lab::CreateNullPhysicsBackendV2},
    {"NullPhysicsBackendV3", &lab::CreateNullPhysicsBackendV3},
    {"NullPhysicsBackendV3_1", &lab::CreateNullPhysicsBackendV3_1},
    {"NullPhysicsBackendV4", &lab::CreateNullPhysicsBackendV4},
}};

std::size_t FindPhysicsBackendIndex(const lab::IPhysicsBackend& backend) {
    for (std::size_t i = 0; i < kPhysicsBackendOptions.size(); ++i) {
        if (std::strcmp(backend.Name(), kPhysicsBackendOptions[i].name) == 0) {
            return i;
        }
    }
    return 0;
}

std::unique_ptr<lab::IPhysicsBackend> CreatePhysicsBackendByIndex(std::size_t index) {
    return kPhysicsBackendOptions[index % kPhysicsBackendOptions.size()].create();
}

// ---------------------------------------------------------------------------
// AppState — 应用程序的所有运行时状态
// ---------------------------------------------------------------------------
// 集中管理，避免全局变量散落。
struct AppState {
    lab::World world;                            // 世界数据（唯一真相源）
    std::unique_ptr<lab::IPhysicsBackend> physics; // 物理后端（可替换）
    lab::Renderer renderer;                       // raylib 渲染器
    lab::PhysicsConfig physics_config{};          // 物理配置（步长、重力等）
    lab::DebugDrawOptions draw_options{};         // 调试绘制选项

    lab::DemoSceneId current_scene = lab::DemoSceneId::BoxDrop; // 当前场景
    bool physics_initialized = false;  // 物理后端是否已初始化
    bool paused = true;                // 是否暂停（默认暂停，方便观察初始状态）
    bool request_single_step = false;  // 是否请求单步推进

    double accumulator = 0.0;                    // 固定步长累加器（秒）
    std::uint64_t tick = 0;                      // 物理 tick 计数
    lab::ObjectId selected = lab::kInvalidObject; // 当前选中的对象

    lab::PhysicsDebugDrawData debug_draw_data{};  // 物理后端本帧的可视化调试数据
    lab::PhysicsDebugStats    debug_stats{};      // 物理后端本帧的统计摘要

    // 性能计时（EMA 平滑，避免数字抖动）
    float physics_step_ms_avg = 0.0f;  // 每次 Step() 的平均 wall-clock 耗时（ms）
    float render_fps_smooth   = 60.0f; // 渲染主线程帧率
};

// ---------------------------------------------------------------------------
// ReloadScene — 加载/重载指定场景
// ---------------------------------------------------------------------------
// 流程：
//   1. 构建场景数据到 World
//   2. 物理后端初始化（首次）或重建（后续）
//   3. 同步一次运行时状态到 World
//   4. 重置累加器、tick、选中等状态
// 调用方：main() 启动时；R 键重载；1/2 切换场景；P 键切换后端后均会调用。
void ReloadScene(AppState& app, lab::DemoSceneId scene_id) {
    app.current_scene = scene_id;
    lab::BuildScene(app.world, scene_id);  // 构建场景数据

    // 首次初始化 vs 后续重建
    if (!app.physics_initialized) {
        app.physics->Initialize(app.world, app.physics_config);
        app.physics_initialized = true;
    } else {
        app.physics->RebuildFromWorld(app.world);
    }

    // 立即同步一次，确保 World 中的 body_state 有初始值
    app.physics->SyncRuntimeToWorld(app.world);

    // 重置仿真状态
    app.accumulator = 0.0;
    app.tick = 0;
    app.request_single_step = false;
    app.selected = lab::kInvalidObject;
}

// ---------------------------------------------------------------------------
// AdvancePhysicsOneTick — 执行一次固定步长的物理更新，返回 Step() 的 wall-clock 耗时（ms）
// ---------------------------------------------------------------------------
// 流程：
//   1. 快照当前变换到 previous（用于渲染插值）
//   2. 同步 World → Physics（authoring → runtime）
//   3. 执行物理 Step（仅此段计时）
//   4. 同步 Physics → World（runtime → state）
// 调用方：主循环每消耗一个 fixed_dt 调用一次；N 键单步模式下也会触发一次。
float AdvancePhysicsOneTick(AppState& app) {
    app.world.CapturePreviousTransforms();
    app.physics->SyncAuthoringToRuntime(app.world);

    const auto t0 = std::chrono::high_resolution_clock::now();
    app.physics->Step(app.physics_config.fixed_dt);
    const auto t1 = std::chrono::high_resolution_clock::now();
    const float step_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    app.physics->SyncRuntimeToWorld(app.world);
    ++app.tick;
    return step_ms;
}

}  // namespace

// ===========================================================================
// main — 程序入口
// ===========================================================================
int main() {
    AppState app;

    // 创建物理后端——默认使用 Jolt，可在运行时按 P 键切换
    app.physics = CreatePhysicsBackendByIndex(0);

    // 初始化渲染窗口（1600x900）并加载默认场景
    app.renderer.Initialize(1600, 900, "C++ Physics Debug Platform");
    ReloadScene(app, app.current_scene);

    // ===================================================================
    // 主循环
    // ===================================================================
    while (!app.renderer.ShouldClose()) {
        // 获取帧时间，钳制最大 0.25s 防止断点后大时间差
        const float frame_dt = std::min(app.renderer.GetFrameTimeSeconds(), 0.25f);

        // ---------------------------------------------------------------
        // 键盘输入处理
        // ---------------------------------------------------------------

        if (IsKeyPressed(KEY_H)) {
            app.draw_options.show_help = !app.draw_options.show_help;           // H: 切换帮助信息
        }
        if (IsKeyPressed(KEY_G)) {
            app.draw_options.show_grid = !app.draw_options.show_grid;           // G: 切换地面网格
        }
        if (IsKeyPressed(KEY_X)) {
            app.draw_options.show_local_axes = !app.draw_options.show_local_axes; // X: 切换局部坐标轴
        }
        if (IsKeyPressed(KEY_B)) {
            app.draw_options.show_selected_aabb = !app.draw_options.show_selected_aabb; // B: 切换包围盒
        }
        if (IsKeyPressed(KEY_I)) {
            app.draw_options.interpolate_transforms = !app.draw_options.interpolate_transforms; // I: 切换插值
        }
        if (IsKeyPressed(KEY_SPACE)) {
            app.paused = !app.paused;  // Space: 暂停/恢复
        }
        if (IsKeyPressed(KEY_N)) {
            app.request_single_step = true;  // N: 单步推进
            app.paused = true;               // 单步后自动暂停
        }
        if (IsKeyPressed(KEY_R)) {
            ReloadScene(app, app.current_scene);  // R: 重载当前场景
        }
        // P 键：在后端列表中循环切换，切换后自动重载场景
        if (IsKeyPressed(KEY_P)) {
            const std::size_t current_backend_index = FindPhysicsBackendIndex(*app.physics);
            const std::size_t next_backend_index = (current_backend_index + 1) % kPhysicsBackendOptions.size();
            app.physics->Shutdown();
            app.physics_initialized = false;
            app.physics = CreatePhysicsBackendByIndex(next_backend_index);
            ReloadScene(app, app.current_scene);
        }
        if (IsKeyPressed(KEY_ONE)) {
            ReloadScene(app, lab::DemoSceneId::BoxDrop);       // 1: 单体落地
        }
        if (IsKeyPressed(KEY_TWO)) {
            ReloadScene(app, lab::DemoSceneId::BoxPush);       // 2: 双体对撞
        }
        if (IsKeyPressed(KEY_THREE)) {
            ReloadScene(app, lab::DemoSceneId::BoxSlope);      // 3: 斜面滑落
        }
        if (IsKeyPressed(KEY_FOUR)) {
            ReloadScene(app, lab::DemoSceneId::BoxSpinDrop);   // 4: 旋转落地
        }
        if (IsKeyPressed(KEY_FIVE)) {
            ReloadScene(app, lab::DemoSceneId::BoxStack);      // 5: 纯盒堆叠
        }
        if (IsKeyPressed(KEY_SIX)) {
            ReloadScene(app, lab::DemoSceneId::FrictionLab);   // 6: 摩擦对比
        }
        if (IsKeyPressed(KEY_SEVEN)) {
            ReloadScene(app, lab::DemoSceneId::RagdollPrep);   // 7: 布娃娃
        }
        if (IsKeyPressed(KEY_EIGHT)) {
            ReloadScene(app, lab::DemoSceneId::CubeRain);      // 8: 方块雨
        }
        // 物理调试可视化开关
        if (IsKeyPressed(KEY_C)) {
            app.draw_options.show_physics_contacts = !app.draw_options.show_physics_contacts;
        }
        if (IsKeyPressed(KEY_V)) {
            app.draw_options.show_physics_contact_normals = !app.draw_options.show_physics_contact_normals;
        }
        if (IsKeyPressed(KEY_J)) {
            app.draw_options.show_physics_body_coms = !app.draw_options.show_physics_body_coms;
        }
        if (IsKeyPressed(KEY_K)) {
            app.draw_options.show_physics_collider_frames = !app.draw_options.show_physics_collider_frames;
        }
        if (IsKeyPressed(KEY_O)) {
            app.draw_options.show_physics_aabbs = !app.draw_options.show_physics_aabbs;
        }

        // F 键：在场景中央上方生成一个随机动态方块（仅在物理运行时可用）
        if (IsKeyPressed(KEY_F) && !app.paused) {
            lab::SpawnDynamicBox(app.world);
        }

        // 更新相机（WASD + 右键看向 + 滚轮缩放）
        app.renderer.UpdateCamera(frame_dt);

        // ---------------------------------------------------------------
        // 鼠标拾取（左键点击选中物体，但不在右键看向时触发）
        // ---------------------------------------------------------------
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            const float alpha = (app.physics_config.fixed_dt > 0.0f)
                                    ? static_cast<float>(app.accumulator / app.physics_config.fixed_dt)
                                    : 0.0f;
            app.selected = app.renderer.PickObject(app.world, alpha, app.draw_options.interpolate_transforms);
        }

        // ---------------------------------------------------------------
        // 固定步长累加器逻辑
        // ---------------------------------------------------------------
        // 未暂停时累加真实的帧时间
        if (!app.paused) {
            app.accumulator += frame_dt;
        }
        // 单步模式：强制累加器至少有一个步长的量
        if (app.request_single_step) {
            app.accumulator = std::max(app.accumulator, static_cast<double>(app.physics_config.fixed_dt));
            app.request_single_step = false;
        }

        // 消耗累加器，执行对应次数的物理 Step，累加 Step 耗时以计算 EMA
        int substeps = 0;
        float step_time_sum = 0.0f;
        while (app.accumulator >= app.physics_config.fixed_dt &&
               substeps < app.physics_config.max_substeps_per_frame) {
            step_time_sum += AdvancePhysicsOneTick(app);
            app.accumulator -= app.physics_config.fixed_dt;
            ++substeps;
        }
        // 更新物理步时 EMA（alpha=0.1，约 10 帧平滑）
        if (substeps > 0) {
            const float new_step_ms = step_time_sum / static_cast<float>(substeps);
            app.physics_step_ms_avg = 0.9f * app.physics_step_ms_avg + 0.1f * new_step_ms;
        }
        // 更新渲染帧率 EMA
        if (frame_dt > 0.0f) {
            app.render_fps_smooth = 0.9f * app.render_fps_smooth + 0.1f * (1.0f / frame_dt);
        }

        // 固定步长框架里要明确抑制 spiral of death：
        // 当本帧追不上时，直接丢弃堆积时间，而不是无限补帧。
        // 这样可以避免帧率下降→物理步数更多→帧率更低的恰怨循环。
        if (substeps == app.physics_config.max_substeps_per_frame &&
            app.accumulator >= app.physics_config.fixed_dt) {
            app.accumulator = 0.0;
        }
        // 收集物理调试数据（每帧一次，反映最新一次 step 后的状态）
        app.debug_draw_data.Clear();
        app.physics->CollectDebugDrawData(app.debug_draw_data);
        app.debug_stats = app.physics->GetDebugStats();
        // 计算插值因子 alpha：累加器剩余 / 步长，范围 [0, 1)
        // 用于渲染时在上一帧和当前帧之间插值，实现平滑显示
        const float alpha = (app.physics_config.fixed_dt > 0.0f)
                                ? static_cast<float>(app.accumulator / app.physics_config.fixed_dt)
                                : 0.0f;

        // ---------------------------------------------------------------
        // 控制台调试输出：每 60 帧打印一次，避免刷屏
        // ---------------------------------------------------------------
        static int dbg_frame = 0;
        if (++dbg_frame >= 60) {
            dbg_frame = 0;
            printf("[frame] dt=%.3fms  fps=%.1f  tick=%llu  accum=%.3fms  substeps=%d",
                   frame_dt * 1000.0f,
                   1.0f / frame_dt,
                   static_cast<unsigned long long>(app.tick),
                   app.accumulator * 1000.0,
                   substeps);
            printf("  | bodies dynamic=%zu static=%zu kinematic=%zu",
                   app.world.CountBodies(lab::MotionType::Dynamic),
                   app.world.CountBodies(lab::MotionType::Static),
                   app.world.CountBodies(lab::MotionType::Kinematic));
            if (app.world.IsValid(app.selected)) {
                const lab::Vec3 pos = app.world.GetTransform(app.selected).position;
                const lab::Vec3 lv  = app.world.body_state.linear_velocity[app.selected];
                printf("  | sel[%u] pos=(%.2f,%.2f,%.2f) lv=(%.2f,%.2f,%.2f)",
                       app.selected, pos.x, pos.y, pos.z, lv.x, lv.y, lv.z);
            }
            printf("\n");
        }

        // ---------------------------------------------------------------
        // 渲染
        // ---------------------------------------------------------------
        BeginDrawing();
        ClearBackground(Color{232, 236, 241, 255});  // 淡灰蓝背景

        // 3D 世界绘制
        app.renderer.DrawWorld(app.world, app.selected, app.draw_options, alpha, app.debug_draw_data);

        // 2D HUD 叠加层
        app.renderer.DrawOverlay(app.world,
                                 app.selected,
                                 app.draw_options,
                                 app.paused,
                                 app.tick,
                                 app.physics->Name(),
                                 lab::DemoSceneName(app.current_scene),
                                 app.debug_stats,
                                 app.render_fps_smooth,
                                 app.physics_step_ms_avg);
        DrawFPS(GetScreenWidth() - 96, 12);  // 右上角 FPS 计数器
        EndDrawing();
    }

    // ===================================================================
    // 清理
    // ===================================================================
    if (app.physics) {
        app.physics->Shutdown();  // 释放物理后端资源
    }
    app.renderer.Shutdown();      // 关闭窗口、释放渲染资源
    return 0;
}

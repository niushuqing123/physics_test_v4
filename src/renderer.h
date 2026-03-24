// ============================================================================
// renderer.h — 渲染器（平台/可视化层）接口声明
// ============================================================================
// Renderer 是 raylib 的封装层，只负责：
//   1. 窗口管理与生命周期（初始化/关闭）
//   2. 3D 相机控制（FPS 飞行模式）
//   3. 世界对象的 3D 渲染（盒子/球体/胶囊体 + 线框 + 坐标轴 + AABB）
//   4. 鼠标拾取（射线-AABB 检测）
//   5. HUD 叠加层（信息面板、帮助文字）
//
// 设计边界：
//   - 不拥有任何游戏对象数据，只从 World 读取
//   - 不参与物理真值计算
//   - 以后换 SDL + OpenGL / WebGL 时，只需替换本文件和对应 .cpp
// ============================================================================
#pragma once

#include <cstdint>

#include <raylib.h>

#include "physics_api.h"  // 包含 PhysicsDebugStats / PhysicsDebugDrawData 及其所依赖的 world.h
#include "world.h"

namespace lab {

// ---------------------------------------------------------------------------
// DebugDrawOptions — 调试绘制选项
// ---------------------------------------------------------------------------
// 通过快捷键切换，控制辅助可视化元素的显示
struct DebugDrawOptions {
    bool show_grid = true;                       // G 键：是否显示地面网格
    bool show_wireframe = true;                  // 是否显示物体线框
    bool show_local_axes = true;                 // X 键：是否显示选中物体的局部坐标轴
    bool show_selected_aabb = true;              // B 键：是否显示选中物体的包围盒
    bool interpolate_transforms = false;         // I 键：是否启用渲染插值（平滑显示）
    bool show_help = true;                       // H 键：是否显示帮助信息
    // --- 物理调试可视化开关 ---
    bool show_physics_contacts = false;          // C 键：接触点（黄色小球）
    bool show_physics_contact_normals = false;   // V 键：接触法线（青色箭头线）
    bool show_physics_body_coms = false;         // J 键：质心与 Transform 原点（品红/白小球）
    bool show_physics_collider_frames = false;   // K 键：Collider 局部坐标系（三色轴）
    bool show_physics_aabbs = false;             // O 键：物理后端 AABB（绿色线框）
};

// ---------------------------------------------------------------------------
// Renderer — raylib 封装层
// ---------------------------------------------------------------------------
class Renderer {
public:
    // 初始化窗口、相机、模型资源和字体。
    // 调用方：main() 启动时调用一次。
    void Initialize(int width, int height, const char* title);
    // 销毁资源、关闭窗口。
    // 调用方：main() 主循环结束后调用一次。
    void Shutdown();

    // 查询窗口是否应该关闭（用户点了关闭按钮或按了 ESC）
    [[nodiscard]] bool ShouldClose() const;
    // 获取上一帧的时长（秒），用于帧率无关的更新
    [[nodiscard]] float GetFrameTimeSeconds() const;

    // 更新相机位置/朝向（WASD 移动、鼠标右键环视、滚轮缩放）。
    // 调用方：主循环每帧调用，在键盘输入处理之后、渲染之前。
    void UpdateCamera(float dt);

    // 鼠标拾取：发射从鼠标位置出发的射线，检测与场景中 AABB 的碰撞
    // 返回最近命中物体的 ObjectId，未命中返回 kInvalidObject
    // alpha 和 interpolate 控制是否使用插值后的变换计算 AABB
    // 调用方：主循环在鼠标左键按下时调用（不与右键看向同时触发）。
    [[nodiscard]] ObjectId PickObject(const World& world, float alpha, bool interpolate) const;

    // 绘制 3D 世界：所有可渲染对象 + 调试辅助元素
    // alpha 是当前帧在物理步长中的插值因子 [0, 1]
    // debug_data 包含物理后端本帧的可视化数据（接触点/法线/COM/AABB 等）
    // 调用方：主循环每帧在 BeginDrawing()/EndDrawing() 块内调用。
    void DrawWorld(const World& world,
                   ObjectId selected,
                   const DebugDrawOptions& options,
                   float alpha,
                   const PhysicsDebugDrawData& debug_data) const;

    // 绘制 2D 叠加层（HUD）：信息面板、物理统计、选中物体详情、帮助文字。
    // stats 包含物理后端本帧的统计摘要。
    // render_fps: EMA 平滑后的渲染帧率（帧/秒）
    // physics_step_ms: EMA 平滑后的每次 Step() wall-clock 耗时（毫秒）
    // 调用方：主循环每帧在 DrawWorld() 之后调用。
    void DrawOverlay(const World& world,
                     ObjectId selected,
                     const DebugDrawOptions& options,
                     bool paused,
                     std::uint64_t tick,
                     const char* physics_backend_name,
                     const char* scene_name,
                     const PhysicsDebugStats& stats,
                     float render_fps,
                     float physics_step_ms) const;

private:
    // 根据 yaw_ / pitch_ / camera_position_ 计算 camera_.target
    void SyncCamera();

    // --- raylib 3D 相机 ---
    Camera3D camera_{};
    Model unit_cube_{};             // 2x2x2 的单位立方体模型，Box 都用这个模型缩放绘制
    bool cube_model_ready_ = false; // 模型是否加载成功

    // --- FPS 飞行相机参数 ---
    float yaw_ = -2.35f;           // 水平旋转角（弧度），初始朝向场景中心
    float pitch_ = -0.45f;         // 俯仰角（弧度），正值朝上负值朝下
    Vector3 camera_position_{8.0f, 6.0f, 8.0f};  // 相机世界空间位置
    bool cursor_captured_ = false;  // 鼠标是否被右键捕获（隐藏光标）

    // --- UI 字体 ---
    Font ui_font_{};                // 等宽字体，用于 HUD 文字
    bool font_loaded_ = false;      // 字体是否加载成功
};

}  // namespace lab

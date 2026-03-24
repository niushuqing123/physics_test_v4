// ============================================================================
// renderer.cpp — 渲染器实现（raylib 封装层）
// ============================================================================
// 负责窗口管理、相机控制、世界对象 3D 渲染、鼠标拾取、HUD 叠加层等。
//
// 渲染流程（每帧）：
//   1. UpdateCamera()  — 处理输入，更新飞行相机
//   2. DrawWorld()     — BeginMode3D 内绘制所有 3D 对象 + 调试辅助
//   3. DrawOverlay()   — 2D 层绘制 HUD 信息面板和帮助文字
//
// 对象渲染方式：
//   - Box:     用一个 2x2x2 的 unit_cube_ 模型，乖以 half_extents 缩放
//   - Sphere:  用 raylib 的 DrawSphereEx 绘制
//   - Capsule: 用 raylib 的 DrawCapsule 绘制
// ============================================================================
#include "renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace lab {
namespace {

// --- 常量 ---
constexpr float kRadToDeg = 57.2957795131f;           // 弧度转角度的乘数
constexpr Color kWireColor = Color{30, 30, 30, 220};  // 线框默认颜色（深灰）

// --- lab → raylib 类型转换辅助 ---

Vector3 ToRl(Vec3 v) {
    return Vector3{v.x, v.y, v.z};
}

Color ToRl(ColorRGBA8 c) {
    return Color{c.r, c.g, c.b, c.a};
}

// 取向量三个分量绝对值的最大值（用于等比缩放场景）
float MaxAbs3(Vec3 v) {
    return std::max({std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)});
}

// ---------------------------------------------------------------------------
// ComputeAabb — 计算对象在世界空间中的轴对齐包围盒 (AABB)
// ---------------------------------------------------------------------------
// 用于鼠标拾取和调试显示。
// 不同形状的 AABB 计算方式不同：
//   Box:     考虑旋转后的投影范围
//   Sphere:  均匀向外扩展半径
//   Capsule: 两端点沿轴方向偏移后取包围
BoundingBox ComputeAabb(const World& world, ObjectId id, const Transform& transform) {
    const Vec3 center = transform.position;

    switch (world.render.shape[id]) {
        case ShapeKind::Box: {
            // 旋转 Box 的 AABB 计算：
            // 将三个局部轴方向旋转到世界空间，
            // 然后取每个轴在世界 x/y/z 上的投影绝对值之和，得到包围半尺寸
            const Vec3 h = MultiplyComponents(world.render.box_half_extents[id], transform.scale);
            const Vec3 x_axis = Rotate(transform.rotation, Vec3{1.0f, 0.0f, 0.0f});
            const Vec3 y_axis = Rotate(transform.rotation, Vec3{0.0f, 1.0f, 0.0f});
            const Vec3 z_axis = Rotate(transform.rotation, Vec3{0.0f, 0.0f, 1.0f});

            const Vec3 e{
                std::fabs(x_axis.x) * std::fabs(h.x) + std::fabs(y_axis.x) * std::fabs(h.y) + std::fabs(z_axis.x) * std::fabs(h.z),
                std::fabs(x_axis.y) * std::fabs(h.x) + std::fabs(y_axis.y) * std::fabs(h.y) + std::fabs(z_axis.y) * std::fabs(h.z),
                std::fabs(x_axis.z) * std::fabs(h.x) + std::fabs(y_axis.z) * std::fabs(h.y) + std::fabs(z_axis.z) * std::fabs(h.z),
            };

            return BoundingBox{ToRl(center - e), ToRl(center + e)};
        }
        case ShapeKind::Sphere: {
            // 球体 AABB：以中心为基准，均匀向外扩展缩放后的半径
            const float r = world.render.sphere_radius[id] * MaxAbs3(transform.scale);
            const Vec3 e{r, r, r};
            return BoundingBox{ToRl(center - e), ToRl(center + e)};
        }
        case ShapeKind::Capsule: {
            // 胶囊 AABB：沿局部 Y 轴方向找到两个端点，
            // 再向外扩展截面半径
            const float r = world.render.capsule_radius[id] * std::max(std::fabs(transform.scale.x), std::fabs(transform.scale.z));
            const float hh = world.render.capsule_half_height[id] * std::fabs(transform.scale.y);
            const Vec3 axis = Rotate(transform.rotation, Vec3{0.0f, 1.0f, 0.0f});
            const Vec3 offset = axis * hh;
            const Vec3 start = center - offset;
            const Vec3 end = center + offset;
            const Vec3 bmin{
                std::min(start.x, end.x) - r,
                std::min(start.y, end.y) - r,
                std::min(start.z, end.z) - r,
            };
            const Vec3 bmax{
                std::max(start.x, end.x) + r,
                std::max(start.y, end.y) + r,
                std::max(start.z, end.z) + r,
            };
            return BoundingBox{ToRl(bmin), ToRl(bmax)};
        }
        default:
            return BoundingBox{ToRl(center), ToRl(center)};
    }
}

// ---------------------------------------------------------------------------
// DrawLocalAxes — 在指定位置绘制局部坐标轴
// ---------------------------------------------------------------------------
// 红=X、绿=Y、蓝=Z，方便可视化对象的旋转方向
void DrawLocalAxes(const Transform& transform, float axis_length) {
    const Vec3 origin = transform.position;
    const Vec3 x_axis = Rotate(transform.rotation, Vec3{1.0f, 0.0f, 0.0f});
    const Vec3 y_axis = Rotate(transform.rotation, Vec3{0.0f, 1.0f, 0.0f});
    const Vec3 z_axis = Rotate(transform.rotation, Vec3{0.0f, 0.0f, 1.0f});

    DrawLine3D(ToRl(origin), ToRl(origin + x_axis * axis_length), RED);
    DrawLine3D(ToRl(origin), ToRl(origin + y_axis * axis_length), GREEN);
    DrawLine3D(ToRl(origin), ToRl(origin + z_axis * axis_length), BLUE);
}

}  // namespace

// ===========================================================================
// Renderer 实现
// ===========================================================================

// ---------------------------------------------------------------------------
// Initialize — 初始化窗口、相机、模型和字体
// ---------------------------------------------------------------------------
void Renderer::Initialize(int width, int height, const char* title) {
    // 创建 raylib 窗口，目标帧率 60fps
    InitWindow(width, height, title);
    SetTargetFPS(60);

    // 初始化 3D 透视相机
    camera_.up = Vector3{0.0f, 1.0f, 0.0f};  // Y 轴朝上
    camera_.fovy = 60.0f;                      // 垂直视场角 60°
    camera_.projection = CAMERA_PERSPECTIVE;   // 透视投影
    SyncCamera();  // 根据 yaw_/pitch_ 计算初始 target

    // 加载单位立方体模型（2x2x2），所有 Box 都用它缩放绘制
    unit_cube_ = LoadModelFromMesh(GenMeshCube(2.0f, 2.0f, 2.0f));
    cube_model_ready_ = IsModelValid(unit_cube_);

    // 加载 Consolas 等宽字体（Windows 系统字体），用于 HUD 文字
    ui_font_ = LoadFontEx("C:/Windows/Fonts/consola.ttf", 18, nullptr, 0);
    font_loaded_ = IsFontValid(ui_font_);
    if (font_loaded_) {
        // 双线性过滤让字体渲染更清晰
        SetTextureFilter(ui_font_.texture, TEXTURE_FILTER_BILINEAR);
    }

    EnableCursor();  // 初始状态显示鼠标光标
}

// ---------------------------------------------------------------------------
// Shutdown — 释放所有资源并关闭窗口
// ---------------------------------------------------------------------------
void Renderer::Shutdown() {
    if (cube_model_ready_) {
        UnloadModel(unit_cube_);
        cube_model_ready_ = false;
    }

    if (font_loaded_) {
        UnloadFont(ui_font_);
        font_loaded_ = false;
    }

    if (IsWindowReady()) {
        CloseWindow();
    }
}

// ---------------------------------------------------------------------------
// ShouldClose / GetFrameTimeSeconds — 窗口状态查询
// ---------------------------------------------------------------------------
bool Renderer::ShouldClose() const {
    return WindowShouldClose();
}

float Renderer::GetFrameTimeSeconds() const {
    return GetFrameTime();
}

// ---------------------------------------------------------------------------
// SyncCamera — 根据 yaw_/pitch_ 角度计算相机的看向点
// ---------------------------------------------------------------------------
// yaw_: 水平旋转角（0 = +X 方向）
// pitch_: 仰角（正值朝上，钳制在 ±83° 以防万向节死锁）
void Renderer::SyncCamera() {
    const float cos_pitch = std::cos(pitch_);
    const float sin_pitch = std::sin(pitch_);
    const float cos_yaw = std::cos(yaw_);
    const float sin_yaw = std::sin(yaw_);

    const Vector3 forward{
        cos_pitch * cos_yaw,
        sin_pitch,
        cos_pitch * sin_yaw,
    };

    camera_.position = camera_position_;
    camera_.target = Vector3{
        camera_position_.x + forward.x,
        camera_position_.y + forward.y,
        camera_position_.z + forward.z,
    };
}

// ---------------------------------------------------------------------------
// UpdateCamera — 处理输入，更新 FPS 飞行相机
// ---------------------------------------------------------------------------
// 操作：
//   - 右键按住：隐藏光标 + 鼠标控制朝向
//   - WASD: 水平移动（沿视角方向的 XZ 平面投影）
//   - Q/E:  垂直下降/上升
//   - Shift: 加速
//   - 滚轮: 沿视线方向缩放
void Renderer::UpdateCamera(float dt) {
    // --- 右键捕获鼠标 ---
    const bool right_held = IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
    if (right_held && !cursor_captured_) {
        DisableCursor();
        cursor_captured_ = true;
    } else if (!right_held && cursor_captured_) {
        EnableCursor();
        cursor_captured_ = false;
    }

    // --- 右键按住时用鼠标控制朝向 ---
    if (cursor_captured_) {
        const Vector2 delta = GetMouseDelta();
        yaw_ -= delta.x * 0.0035f;                               // 水平旋转
        pitch_ = Clamp(pitch_ - delta.y * 0.0030f, -1.45f, 1.45f); // 仰角钳制
    }

    // --- 计算移动方向基向量 ---
    // 只用视线的 XZ 分量作为前方，保证 WASD 永远在水平面移动
    const Vec3 view_dir = Normalize(Vec3{
        camera_.target.x - camera_.position.x,
        camera_.target.y - camera_.position.y,
        camera_.target.z - camera_.position.z,
    });
    Vec3 forward_flat = Normalize(Vec3{view_dir.x, 0.0f, view_dir.z});
    if (LengthSq(forward_flat) < 1e-6f) {
        forward_flat = Vec3{1.0f, 0.0f, 0.0f};
    }
    const Vec3 right = Normalize(Cross(forward_flat, Vec3{0.0f, 1.0f, 0.0f}));

    // --- 缩放速度设置 ---
    float move_speed = IsKeyDown(KEY_LEFT_SHIFT) ? 12.0f : 5.0f;  // Shift 加速
    move_speed *= dt;  // 帧率无关

    // --- WASD + QE 方向移动 ---
    Vec3 movement{0.0f, 0.0f, 0.0f};
    if (IsKeyDown(KEY_W)) {
        movement += forward_flat;
    }
    if (IsKeyDown(KEY_S)) {
        movement -= forward_flat;
    }
    if (IsKeyDown(KEY_D)) {
        movement += right;
    }
    if (IsKeyDown(KEY_A)) {
        movement -= right;
    }
    if (IsKeyDown(KEY_E)) {
        movement += Vec3{0.0f, 1.0f, 0.0f};
    }
    if (IsKeyDown(KEY_Q)) {
        movement -= Vec3{0.0f, 1.0f, 0.0f};
    }

    // 归一化后乘以速度，避免对角方向移动快于正方向
    if (LengthSq(movement) > 1e-6f) {
        movement = Normalize(movement) * move_speed;
        camera_position_.x += movement.x;
        camera_position_.y += movement.y;
        camera_position_.z += movement.z;
    }

    // --- 滚轮缩放：沿视线方向前进/后退 ---
    const float wheel = GetMouseWheelMove();
    if (std::fabs(wheel) > 1e-6f) {
        camera_position_.x += (camera_.target.x - camera_.position.x) * wheel * 0.75f;
        camera_position_.y += (camera_.target.y - camera_.position.y) * wheel * 0.75f;
        camera_position_.z += (camera_.target.z - camera_.position.z) * wheel * 0.75f;
    }

    SyncCamera();  // 更新相机看向点
}

// ---------------------------------------------------------------------------
// PickObject — 鼠标拾取：射线与 AABB 碰撞检测
// ---------------------------------------------------------------------------
// 从鼠标屏幕位置发射一条射线，遍历所有可渲染对象的 AABB，
// 返回最近命中对象的 ID。
ObjectId Renderer::PickObject(const World& world, float alpha, bool interpolate) const {
    const Ray ray = GetMouseRay(GetMousePosition(), camera_);  // raylib 提供的屏幕→世界射线
    float best_distance = std::numeric_limits<float>::max();
    ObjectId best_object = kInvalidObject;

    for (ObjectId id = 0; id < world.ObjectCount(); ++id) {
        if (!world.IsValid(id) || !world.HasRenderable(id)) {
            continue;
        }

        const Transform transform = world.GetDisplayTransform(id, alpha, interpolate);
        const BoundingBox box = ComputeAabb(world, id, transform);
        const RayCollision hit = GetRayCollisionBox(ray, box);
        // 记录最近的命中结果
        if (hit.hit && hit.distance < best_distance) {
            best_distance = hit.distance;
            best_object = id;
        }
    }

    return best_object;
}

// ---------------------------------------------------------------------------
// DrawWorld — 绘制 3D 世界（对象 + 调试辅助元素）
// ---------------------------------------------------------------------------
void Renderer::DrawWorld(const World& world, ObjectId selected, const DebugDrawOptions& options, float alpha,
                         const PhysicsDebugDrawData& debug_data) const {
    BeginMode3D(camera_);

    // 绘制地面参考网格
    if (options.show_grid) {
        DrawGrid(40, 1.0f);  // 40x40 单位网格
    }

    // --- 遍历并绘制所有可见对象 ---
    for (ObjectId id = 0; id < world.ObjectCount(); ++id) {
        if (!world.IsValid(id) || !world.HasRenderable(id)) {
            continue;
        }

        // 获取用于显示的变换（可能经过插值）
        const Transform transform = world.GetDisplayTransform(id, alpha, options.interpolate_transforms);
        const Color tint = ToRl(world.render.color[id]);
        const bool is_selected = (id == selected);

        switch (world.render.shape[id]) {
            case ShapeKind::Box: {
                // 用 unit_cube_ 模型缩放绘制 Box
                // DrawModelEx 需要轴-角表示的旋转，所以先将四元数转为轴角
                if (!cube_model_ready_) {
                    break;
                }
                const Vec3 axis_half = MultiplyComponents(world.render.box_half_extents[id], transform.scale);
                Vec3 axis;
                float angle_rad = 0.0f;
                QuatToAxisAngle(transform.rotation, axis, angle_rad);

                DrawModelEx(unit_cube_,
                            ToRl(transform.position),
                            ToRl(axis),
                            angle_rad * kRadToDeg,
                            ToRl(axis_half),
                            tint);

                if (options.show_wireframe || is_selected) {
                    DrawModelWiresEx(unit_cube_,
                                     ToRl(transform.position),
                                     ToRl(axis),
                                     angle_rad * kRadToDeg,
                                     ToRl(axis_half),
                                     is_selected ? YELLOW : kWireColor);
                }
                break;
            }
            case ShapeKind::Sphere: {
                // 球体渲染：用最大轴缩放作为等比半径
                const float radius = world.render.sphere_radius[id] * MaxAbs3(transform.scale);
                DrawSphereEx(ToRl(transform.position), radius, 12, 18, tint);
                if (options.show_wireframe || is_selected) {
                    DrawSphereWires(ToRl(transform.position), radius, 12, 18, is_selected ? YELLOW : kWireColor);
                }
                break;
            }
            case ShapeKind::Capsule: {
                // 胶囊渲染：计算实际半径和半高，然后用 Rotate 求出两端点
                const float radius = world.render.capsule_radius[id] * std::max(std::fabs(transform.scale.x), std::fabs(transform.scale.z));
                const float hh = world.render.capsule_half_height[id] * std::fabs(transform.scale.y);
                const Vec3 offset = Rotate(transform.rotation, Vec3{0.0f, 1.0f, 0.0f}) * hh;
                const Vector3 start = ToRl(transform.position - offset);
                const Vector3 end = ToRl(transform.position + offset);
                DrawCapsule(start, end, radius, 12, 12, tint);
                if (options.show_wireframe || is_selected) {
                    DrawCapsuleWires(start, end, radius, 12, 12, is_selected ? YELLOW : kWireColor);
                }
                break;
            }
            default:
                break;
        }
    }

    // --- 选中物体的额外可视化 ---
    if (world.IsValid(selected) && world.HasRenderable(selected)) {
        const Transform transform = world.GetDisplayTransform(selected, alpha, options.interpolate_transforms);
        if (options.show_selected_aabb) {
            DrawBoundingBox(ComputeAabb(world, selected, transform), YELLOW);
        }
        if (options.show_local_axes) {
            DrawLocalAxes(transform, 0.9f);
        }
    }

    // --- 物理后端调试可视化 ---

    // 物理 AABB（绿色线框）
    if (options.show_physics_aabbs) {
        for (const auto& aabb : debug_data.aabbs) {
            const BoundingBox bb{
                Vector3{aabb.min_world.x, aabb.min_world.y, aabb.min_world.z},
                Vector3{aabb.max_world.x, aabb.max_world.y, aabb.max_world.z},
            };
            DrawBoundingBox(bb, Color{0, 210, 90, 200});
        }
    }

    // 接触点（黄色小球）和接触法线（青色方向线）
    if (options.show_physics_contacts || options.show_physics_contact_normals) {
        for (const auto& cp : debug_data.contacts) {
            if (options.show_physics_contacts) {
                DrawSphere(ToRl(cp.position), 0.045f, YELLOW);
            }
            if (options.show_physics_contact_normals) {
                const Vec3 tip = cp.position + cp.normal * 0.28f;
                DrawLine3D(ToRl(cp.position), ToRl(tip), Color{0, 200, 220, 255});
            }
        }
    }

    // 质心（COM）与 Transform 原点（品红小球 + 白色小球 + 连线）
    if (options.show_physics_body_coms) {
        for (const auto& bc : debug_data.body_coms) {
            // Transform 原点：白色
            DrawSphere(ToRl(bc.transform_origin), 0.04f, WHITE);
            // COM：品红色
            DrawSphere(ToRl(bc.com_world), 0.055f, Color{220, 0, 200, 220});
            // 连线（仅在两点不重合时绘制）
            const Vec3 diff = bc.com_world - bc.transform_origin;
            if (Dot(diff, diff) > 1e-6f) {
                DrawLine3D(ToRl(bc.transform_origin), ToRl(bc.com_world), Color{200, 0, 180, 200});
            }
        }
    }

    // Collider 坐标系（小尺度三色轴 + 橙色中心点）
    if (options.show_physics_collider_frames) {
        for (const auto& cf : debug_data.collider_frames) {
            Transform ct{};
            ct.position = cf.center_world;
            ct.rotation = cf.rotation_world;
            ct.scale    = Vec3{1.0f, 1.0f, 1.0f};
            DrawLocalAxes(ct, 0.25f);
            DrawSphere(ToRl(cf.center_world), 0.03f, Color{255, 128, 0, 220});
        }
    }

    EndMode3D();  // 结束 3D 渲染
}

// ---------------------------------------------------------------------------
// DrawOverlay — 绘制 2D HUD 叠加层
// ---------------------------------------------------------------------------
// 包含：信息面板（场景名、后端名、状态、相机位置等）、物理统计、选中物体详情、帮助文字。
void Renderer::DrawOverlay(const World& world,
                           ObjectId selected,
                           const DebugDrawOptions& options,
                           bool paused,
                           std::uint64_t tick,
                           const char* physics_backend_name,
                           const char* scene_name,
                           const PhysicsDebugStats& stats,
                           float render_fps,
                           float physics_step_ms) const {
    const int line_h = 20;  // 每行文字高度（像素）

    // 统一文字绘制函数：有 TTF 字体时用 DrawTextEx（更清晰），否则回退到 raylib 内置位图字体
    auto Txt = [&](const char* text, int x, int y, int size, Color color) {
        if (font_loaded_) {
            DrawTextEx(ui_font_, text, Vector2{(float)x, (float)y}, (float)size, 1.0f, color);
        } else {
            DrawText(text, x, y, size, color);
        }
    };

    // --- 预计算信息面板高度 ---
    // end_y = 172（标题 + 6 行基础信息，含末尾 4px 间隔）
    //       + 3 * line_h（物理统计 2 行 + 性能帧率 1 行）
    //       + 按选中状态追加
    int end_y = 172 + 3 * line_h;
    if (world.IsValid(selected)) {
        end_y += line_h;                                            // 物体名
        if (world.HasRenderable(selected)) end_y += line_h;        // Render Shape
        if (world.HasCollider(selected))   end_y += 2 * line_h;    // Collider Shape + Collider offset
        if (world.HasRigidBody(selected))  end_y += 5 * line_h;    // Motion+Sleeping / Pos / COM offset / LinVel / AngVel
    } else {
        end_y += line_h;  // "Selected: none"
    }
    // end_y 指向下一行起始位置；用它计算面板所需总高度
    const int panel_height = (end_y - line_h + 18 + 10) - 12;

    // --- 绘制背景面板 ---
    DrawRectangle(12, 12, 520, panel_height, Color{245, 245, 245, 220});       // 半透明白色背景
    DrawRectangleLines(12, 12, 520, panel_height, Color{80, 80, 80, 255});     // 边框

    int y = 20;

    // --- 标题 ---
    Txt("Physics Debug Platform", 24, y, 22, BLACK);
    y += 28;

    // --- 场景信息 ---
    Txt(TextFormat("Scene: %s", scene_name), 24, y, 18, BLACK);
    y += line_h;
    Txt(TextFormat("Physics backend: %s", physics_backend_name), 24, y, 18, BLACK);
    y += line_h;
    // 物理状态和 Tick 计数
    Txt(TextFormat("State: %s   Tick: %llu", paused ? "Paused" : "Running", static_cast<unsigned long long>(tick)), 24, y, 18, BLACK);
    y += line_h;
    Txt(TextFormat("Objects: %i", static_cast<int>(world.ObjectCount())), 24, y, 18, BLACK);
    y += line_h;
    Txt(TextFormat("Camera: (%.2f, %.2f, %.2f)", camera_.position.x, camera_.position.y, camera_.position.z), 24, y, 18, BLACK);
    y += line_h;
    Txt(TextFormat("Interpolation: %s", options.interpolate_transforms ? "On" : "Off"), 24, y, 18, BLACK);
    y += line_h;

    // --- 物理统计（来自 GetDebugStats()，仅 Null 后端有求解器数据） ---
    Txt(TextFormat("Bodies: %d  Sleeping: %d  Contacts: %d",
                   stats.num_bodies, stats.num_sleeping, stats.num_contacts),
        24, y, 18, stats.num_bodies > 0 ? BLACK : DARKGRAY);
    y += line_h;
    Txt(TextFormat("Pairs: %d  VelIter: %d  PosIter: %d",
                   stats.num_broadphase_pairs,
                   stats.solver_velocity_iterations,
                   stats.solver_position_iterations),
        24, y, 18, stats.num_broadphase_pairs > 0 ? BLACK : DARKGRAY);
    y += line_h;
    // --- 帧率与物理性能（方便对比后端/场景开销）---
    // Render FPS: 渲染主线程帧率（EMA 平滑）
    // Step ms:    每次 Step() 的 wall-clock 耗时（EMA 平滑）
    // Phys ~FPS:  物理引擎理论最大频率（1000 / step_ms），与渲染帧率无关
    const float phys_theoretical_fps = (physics_step_ms > 0.0f) ? (1000.0f / physics_step_ms) : 0.0f;
    const Color perf_color = (physics_step_ms > 0.0f) ? BLACK : DARKGRAY;
    Txt(TextFormat("Render: %.1f fps  |  Step: %.3f ms  |  Phys ~%.0f fps",
                   render_fps, physics_step_ms, phys_theoretical_fps),
        24, y, 18, perf_color);
    y += line_h + 4;  // 4px 间距分隔统计区与选中物体区

    // --- 选中物体详情 ---
    if (world.IsValid(selected)) {
        const Transform t = world.GetTransform(selected);
        Txt(TextFormat("Selected: [%u] %s", selected, world.names[selected].c_str()), 24, y, 18, MAROON);
        y += line_h;

        if (world.HasRenderable(selected)) {
            Txt(TextFormat("Render Shape: %s", ShapeKindName(world.render.shape[selected])), 24, y, 18, BLACK);
            y += line_h;
        }

        if (world.HasCollider(selected)) {
            Txt(TextFormat("Collider Shape: %s", ShapeKindName(world.colliders.shape[selected])), 24, y, 18, BLACK);
            y += line_h;
            const Vec3 co = world.colliders.local_offset[selected];
            Txt(TextFormat("Coll off: (%.3f, %.3f, %.3f)", co.x, co.y, co.z), 24, y, 18, BLACK);
            y += line_h;
        }

        if (world.HasRigidBody(selected)) {
            const Vec3 lv = world.body_state.linear_velocity[selected];
            const Vec3 av = world.body_state.angular_velocity[selected];
            const bool sleeping = (world.body_state.sleeping[selected] != 0);
            const Vec3 com_off = world.bodies.center_of_mass_offset[selected];
            Txt(TextFormat("Motion: %s   Sleeping: %s   Handle: %u",
                           MotionTypeName(world.bodies.motion_type[selected]),
                           sleeping ? "Y" : "N",
                           world.body_state.handle[selected]),
                24, y, 18, BLACK);
            y += line_h;
            Txt(TextFormat("Pos: (%.3f, %.3f, %.3f)", t.position.x, t.position.y, t.position.z), 24, y, 18, BLACK);
            y += line_h;
            Txt(TextFormat("COM off: (%.3f, %.3f, %.3f)", com_off.x, com_off.y, com_off.z), 24, y, 18, BLACK);
            y += line_h;
            Txt(TextFormat("LinVel: (%.3f, %.3f, %.3f)", lv.x, lv.y, lv.z), 24, y, 18, BLACK);
            y += line_h;
            Txt(TextFormat("AngVel: (%.3f, %.3f, %.3f)", av.x, av.y, av.z), 24, y, 18, BLACK);
            y += line_h;
        }
    } else {
        Txt("Selected: none", 24, y, 18, DARKGRAY);
        y += line_h;
    }

    // --- 底部帮助文字（H 键切换）---
    if (options.show_help) {
        const int help_y = GetScreenHeight() - 164;
        DrawRectangle(12, help_y - 8, 780, 152, Color{245, 245, 245, 220});
        DrawRectangleLines(12, help_y - 8, 780, 152, Color{80, 80, 80, 255});
        Txt("Controls", 24, help_y, 18, BLACK);
        Txt("RMB look | WASD move | Q/E up/down | Wheel zoom", 24, help_y + 22, 16, BLACK);
        Txt("Space pause | N step | R reload | P backend | I interp | F spawn", 24, help_y + 44, 16, BLACK);
        Txt("H help | G grid | X axes | B aabb | LMB select", 24, help_y + 66, 16, BLACK);
        Txt("C contacts | V normals | J com | K coll-frame | O phys-aabb", 24, help_y + 88, 16, BLACK);
        Txt("1 drop | 2 push | 3 slope | 4 spin | 5 stack | 6 friction | 7 ragdoll | 8 rain", 24, help_y + 110, 16, BLACK);
    }
}

}  // namespace lab

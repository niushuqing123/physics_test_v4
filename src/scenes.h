// ============================================================================
// scenes.h — 调试场景定义 + 运行时物体生成
// ============================================================================
// 定义了预设的调试场景，用于测试物理后端的不同能力。
// 同时提供运行时物体生成工具函数，供 main.cpp 交互使用。
//
// 场景矩阵（面向 Null 后端刚体回归测试）：
//   BoxDrop     — 单体落地：重力 + 接触 + 停止
//   BoxPush     — 双体对撞：接触法线 + 速度冲量响应
//   BoxSlope    — 斜面滑落/停留：法线方向 + 摩擦静止判定
//   BoxSpinDrop — 旋转落地：角速度积分 + 旋转碰撞
//   BoxStack    — 纯 box 堆叠：多体接触 + 堆叠稳定性 + 睡眠
//   FrictionLab — 摩擦对比：高/低摩擦两块平台上同类 box 的滑动差异
//   RagdollPrep — 布娃娃预备：多形状（Sphere/Capsule），关节系统入口
//
// 添加新场景的步骤：
//   1. 在 DemoSceneId 枚举中添加新值
//   2. 在 scenes.cpp 中实现 BuildXxx() 函数
//   3. 在 BuildScene() 和 DemoSceneName() 的 switch 中添加分支
//   4. 在 main.cpp 中添加对应的快捷键（当前已用至 8）
// ============================================================================
#pragma once

#include "world.h"

namespace lab {

// 场景 ID 枚举
enum class DemoSceneId {
    BoxDrop,      // 单体落地（重力 + 接触 + 停止）
    BoxPush,      // 双体对撞（接触法线 + 冲量）
    BoxSlope,     // 斜面滑落（法线方向 + 摩擦）
    BoxSpinDrop,  // 旋转落地（角速度 + 旋转碰撞）
    BoxStack,     // 纯盒子堆叠（多体接触 + 睡眠）
    FrictionLab,  // 摩擦对比（高/低摩擦差异）
    RagdollPrep,  // 布娃娃预备（关节系统入口）
    CubeRain,     // 方块雨（大量cube空中分层生成 + 随机初速）
};

// 根据场景 ID 构建/重建世界数据。
// 调用方：main.cpp 的 ReloadScene() 每次场景切换/重载时调用。
void BuildScene(World& world, DemoSceneId scene_id);

// 返回场景的可读名称（用于 HUD 显示）。
// 调用方：main.cpp 将返回值传给 renderer.DrawOverlay() 显示在 HUD 场景名称栏。
const char* DemoSceneName(DemoSceneId scene_id);

// ---------------------------------------------------------------------------
// 运行时物体生成
// ---------------------------------------------------------------------------
// 在场景中央上方生成一个随机颜色/随机初速度的动态方块。
// 只操作 World 数据层，不直接与物理后端交互——
// 物理后端会在下一个 SyncAuthoringToRuntime 时自动发现并注册。
// 返回新创建对象的 ObjectId。
// 调用方：main.cpp 中 F 键按下时触发（仅在非暂停状态下有效）。
ObjectId SpawnDynamicBox(World& world);

}  // namespace lab

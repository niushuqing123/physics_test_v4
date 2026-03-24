# Physics Debug Platform

一个给“手写 3D 刚体物理”准备的最小 C++ 调试平台。

目标只有三个：

1. **世界数据全部自己定义**：Transform、Renderable、Collider、RigidBody authoring/state 都在 `src/world.h` 里，是自己的数据结构。
2. **外部库只负责窗口/输入/可视化**：当前选用 raylib 作为平台层，方便快速出 3D 画面与交互。
3. **物理模块可整体替换**：`src/physics_null.cpp` 是正在研发中的手写物理后端，同时 `src/physics_jolt.cpp` 作为 Jolt 后端提供“正确答案”对照。

---

## 为什么这个骨架长这样

### 1. 世界数据是主数据，渲染与物理都只是“消费者”

当前架构里，`World` 是唯一需要长期维护的核心数据容器：

- `TransformSoA`
- `RenderSoA`
- `ColliderAuthoringSoA`（碰撞几何，已从渲染语义中分离）
- `RigidBodyAuthoringSoA`
- `RigidBodyStateSoA`

这意味着：

- 以后移植到 Java / JS / 其他项目时，最容易迁走的是这一层；
- 物理模块内部可以有自己独立的数据布局，但不能反过来绑死世界层；
- 渲染模块不拥有游戏对象，只负责把 `World` 里的状态画出来。

### 2. 物理内部也保留独立 SoA

`NullPhysicsBackend` 里已经示范了一层和 `World` 解耦的运行时数据：

- `position_`
- `rotation_`
- `linear_velocity_`
- `angular_velocity_`
- `inverse_mass_`

以后替换算法时，外部接口不必变，只需要在 `Step()` 里逐步落下：

1. 外力/重力累计
2. broadphase
3. narrowphase / manifold
4. solver / joints / contacts
5. 速度积分与姿态更新
6. sleeping / ccd / 事件

### 3. 固定步长与渲染分离

主循环中已经给放好了：

- 可暂停
- 可单步
- 固定 `dt`
- 累加器
- 最大子步数钳制（避免 spiral of death）
- 可选渲染插值

所以研究半隐式欧拉、冲量求解、PGS、关节系统时，直接把 `AdvancePhysicsOneTick()` 这条链接上即可。

---

## 文件说明

```text
physics_debug_platform/
├─ CMakeLists.txt
├─ README.md
└─ src/
   ├─ common.h          # 基础类型与数学辅助
   ├─ world.h/.cpp      # 世界数据定义（核心）
   ├─ physics_api.h     # 物理后端稳定接口 + 调试可视化数据结构
   ├─ physics_null.cpp  # 手写物理后端（研发中，见下方说明）
   ├─ physics_jolt.cpp  # Jolt 物理后端（“正确答案”对照跑）
   ├─ renderer.h/.cpp   # raylib 边界层，只做可视化、相机、拾取、调试线框
   ├─ scenes.h/.cpp     # 7 个调试场景（见场景矩阵）
   └─ main.cpp          # 主循环、固定步长、输入、场景切换
```

---

## 当前已经能做什么

- 打开一个 3D 调试窗口
- 鼠标右键看向、WASD 飞行、滚轮缩放
- 7 个调试场景（数字键 1-7 切换）：
  - `1` Box Drop — 单体落地（重力 + 接触 + 停止）
  - `2` Box Push — 双体对撞（接触法线 + 冲量）
  - `3` Box Slope — 斜面滑落（法线方向 + 摩擦）
  - `4` Box Spin Drop — 旋转落地（角速度 + 旋转碰撞）
  - `5` Box Stack — 纯盒子堆叠（多体接触 + 睡眠）
  - `6` Friction Lab — 摩擦对比（高/低摩擦平台）
  - `7` Ragdoll Prep — 布娃娃预备（多形状，关节系统入口）
- 两个物理后端可 P 键切换：
  - **NullPhysicsBackend** — 手写物理（研发中，已有重力、速度/位置积分、Box-Box SAT 窄相、基础 solver、睡眠）
  - **JoltPhysicsBackend** — Jolt 引擎（“正确答案”对照）
- `Space` 暂停/恢复，`N` 单步，`R` 重载场景，`F` 生成方块
- 左键选中物体并查看 inspector
- `I` 渲染插值，`G/X/B/H` 网格/坐标轴/包围盒/帮助
- 物理调试可视化：`C` 接触点、`V` 法线、`J` 质心、`K` collider 坐标系、`O` 物理 AABB

---

## Null 后端当前进度

`NullPhysicsBackend`（`src/physics_null.cpp`）是这个项目的主战场。
它**不是空壳**，已经包含一个最小刚体实验骨架：

- 重力、速度/位置积分、旋转积分（欧拉近似）
- 宽相：AABB 过滤（从 OBB 导出，旋转感知）
- 窄相：Box-Box SAT（15 轴 + 退化处理 + 面轴优先）
- 速度约束求解（线性法向冲量 + 库仑摩擦）
- 位置修正（Baumgarte 稳定化）
- 基础睡眠判定
- 与 World 的正确同步（COM offset / Transform origin 换算）

**尚未实现（下一阶段主线）：**

- manifold clipping（多点接触）
- 角冲量 / 世界逆惯量张量
- persistent manifold / warm starting
- 关节系统
- GJK/EPA / 通用凸体
- CCD

---

## 架构上的关键边界

### World 层（长期稳定层）

- 自己定义的数据格式
- 跨模块沟通唯一真相源
- 最值得未来复用/移植的一层

### Physics 层（可重写层）

- 算法试验区
- 内部数据布局可为性能重排
- 通过 `SyncAuthoringToRuntime()` / `SyncRuntimeToWorld()` 与世界层对接

### Renderer 层（工具层）

- 不拥有世界数据
- 不参与物理真值
- 只负责展示与拾取

这个边界能保证：

- 以后换 raylib / SDL + OpenGL / WebGL，不会重写物理；
- 以后换物理实现，不会重写世界数据；
- 以后移植算法时，最重要的数据定义不会和库 API 缠死。

---

## 构建

### 方式 1：系统里已经装了 raylib

```bash
cmake -S . -B build
cmake --build build
```

### 方式 2：让 CMake 自动抓 raylib

默认 `PDP_FETCH_RAYLIB=ON`，会在本机没有 raylib 时用 `FetchContent` 下载官方 5.5 版本。

```bash
cmake -S . -B build
cmake --build build
```

如果不希望构建时联网：

```bash
cmake -S . -B build -DPDP_FETCH_RAYLIB=OFF
cmake --build build
```

---

## 真正应该改哪几个地方

### 必改 1：`src/physics_null.cpp`

这是主战场。当前已有一个最小刚体骨架，下一步是 manifold / 角冲量 / 关节。

### 常改 2：`src/world.h`

这里决定了未来移植时的数据边界长什么样。

### 偶尔改 3：`src/scenes.cpp`

做新的测试场景、压力场景、关节场景。

### 尽量少改：`src/renderer.*`

它只是工具，不要把业务逻辑和物理逻辑堆进去。

---

## 最后一句建议

这次不要追求“先写一个完整游戏引擎”，而要追求：

- 一套可信的主循环
- 一套自己完全掌控的数据定义
- 一个可以暂停、单步、观察状态的 3D 壳子
- 一个可以被慢慢替换成真实求解器的 physics backend

这个项目就是按这个目标来搭的。

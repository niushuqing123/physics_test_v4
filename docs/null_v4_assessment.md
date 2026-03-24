# Null 后端 v4 评估笔记

## 1. 当前代码和 Jolt 流程对照

### 已完成
- 速度积分、阻尼、角速度积分：`physics_null_v3_normal_only_candidate.cpp:1137-1167`
- 宽相 OBB->AABB + O(n^2) pair：`physics_null_v3_normal_only_candidate.cpp:1169-1187`
- Box-Box SAT 15 轴：`physics_null_v3_normal_only_candidate.cpp:333-391`
- 面接触裁剪 + 最多 4 点：`physics_null_v3_normal_only_candidate.cpp:551-595`
- 每接触点 1 法线 + 2 切线约束：`physics_null_v3_normal_only_candidate.cpp:1190-1215,1405-1449`
- 基础睡眠：`physics_null_v3_normal_only_candidate.cpp:1483-1509`

### 只做到“半步”的地方
- Warm start 只保留法线 λ，切向 λ 已禁用：`physics_null_v3_normal_only_candidate.cpp:1225-1268`
- persistent manifold 只是 pair-key + 世界点近邻匹配，不是 feature-id manifold：`physics_null_v3_normal_only_candidate.cpp:202-206,1225-1268,1320-1336`
- 位置修正只按 `representative_point` 做单点 pseudo impulse，不是按每个接触点修：`physics_null_v3_normal_only_candidate.cpp:1367-1396,1452-1480`

### 还没做到 / 当前不会正确匹配 Jolt 的地方
- Scene 7 的 Sphere/Capsule 在 Null 里被直接 boxify：`physics_null_v3_normal_only_candidate.cpp:220-238`
- Scene 7 的 ragdoll 预备场景本身就含 Sphere/Capsule：`scenes.cpp:346-485`
- 当前速度求解顺序是 **先法线后摩擦**：`physics_null_v3_normal_only_candidate.cpp:1405-1449`
- `合成.md` 里整理的 Jolt 简化流程写的是 **先摩擦后法线**：`合成.md:54-67,1132-1177`
- 当前 `restitution = 0.0f`，所以肉眼看到的“弹跳”不是弹性恢复，而是接触/位置修正数值效应：`physics_null_v3_normal_only_candidate.cpp:1198-1208`

## 2. 现象和代码根因对应

### A. Scene 7 的“圆柱体/胶囊体”抖动、磨蹭
这是**几何不匹配**，不是纯求解器问题。

- 场景 authoring 用了 Sphere/Capsule：`scenes.cpp:393-485`
- Null 后端把它们变成 box：`physics_null_v3_normal_only_candidate.cpp:226-238`

因此它不可能表现出和 Jolt 真 Sphere/Capsule 一样的刚性接触。

### B. point-face / line-face 的“跳跳糖”“电火花”
这是**接触特征不稳定 + 单点位置修正过于敏感**。

- SAT 仍然靠最小重叠轴选 normal，edge axis 只加了很小的偏置 `1e-4`：`physics_null_v3_normal_only_candidate.cpp:366-373`
- 一旦落到 edge-edge 分支，就只生成单个 midpoint：`physics_null_v3_normal_only_candidate.cpp:1356-1360,597-629`
- 位置修正不是逐 contact point，而是整流形只用一个 `representative_point`：`physics_null_v3_normal_only_candidate.cpp:1394-1396,1466-1476`

这几件事叠在一起，边/角接触比面接触更容易抖。

### C. v3.1 的 irregular pile / 乱堆方块慢慢蠕动
这和 v3 的“静止偏置抑制”有关。

- v3.1 在 `InitializeContact()` 里，当法向相对速度较小且 penetration 也不大时，会把 velocity bias 直接压到 0：`physics_null_v3_normal_only_candidate.cpp:1201-1208`
- 对金字塔这类规则 face-face 堆叠影响不大；对 irregular pile 里大量 point/edge 接触，会让残余穿透/剪切更容易变成长期 creep。

## 3. 和 v1 / v2 的关键分叉

- v1 的位置修正是**纯平移**，而且按每个 contact 做：`physics_null_v1.cpp:1214-1245`
- v2/v3 改成了**带角向的 pseudo impulse**，但只在 manifold representative point 上修：`physics_null_v2.cpp:1276-1304`

这正好解释了：
- v2/v3 的宏观运动、旋转接触比 v1 更像刚体
- 但 v2/v3 的近静止 edge contact 更容易“活”起来

## 4. 我在沙盒里做的回归实验（headless）

> 说明：下面是基于当前上传源码做的本地 headless stress test，不依赖 renderer / Jolt。

### BoxStack（scene 5）
- v1：全部睡死，后段几乎 0 漂移
- v2：全部睡死，后段几乎 0 漂移
- v3.1：全部睡死，后段几乎 0 漂移

### Deterministic irregular pile（自建 30 个随机旋转 box）
- v1：30/30 睡死，后段 0 漂移
- v2：30/30 睡死，后段 0 漂移
- v3.1：只有 25/30 睡死，后段平均漂移约 0.027，最大约 0.327

### 针对 v3.1 的小改动切片
- 只恢复 v2 的 velocity bias（不再对 near-rest contact 直接置 0）→ irregular pile 变稳
- 只恢复 v2 的 wake/broadphase 行为 → 没有变稳，反而更活
- 把 SAT 的 edge-axis 偏置从 `1e-4` 提到 `0.05` → irregular pile 明显变稳
- 把求解顺序改成 friction-first / normal-last → irregular pile 也有改善

## 5. 结论

### 该不该做 v4？
**值得做，但只能做一个“窄范围 v4”。**

### 最值得做的 v4 范围
1. 保留 v3.1 的 **normal-only warm start**
2. 增强 SAT/contact feature 稳定性（face preference / axis hysteresis）
3. 把求解顺序改成 **先摩擦后法线**，和 `合成.md` 对齐
4. 把“near-rest bias 直接置 0”改成**平滑衰减或保底 bias**，不要二元开关

### 不建议在 v4 做的事
- 不做 Sphere/Capsule 真形状支持
- 不做 full tangent warm start
- 不做 GJK/EPA
- 不做 joint / island / CCD

这些都会明显抬高复杂度，不适合“未来还要迁 Java / JS 的单文件求解器”。

### 如果决定不做 v4
那就不要以 v3.1 收尾，应该以 **v2 作为收紧基线**。v2 仍然是目前“效果 / 复杂度 / 可迁移性”最平衡的版本。

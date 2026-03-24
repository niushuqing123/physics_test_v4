# Null v4 说明与后续布娃娃兼容性判断

## 这版 v4 实际保留/放弃了什么

这版不是把上一轮评估里所有候选改动一股脑都塞进去，而是先做了回归筛选。
实际结果是：

- **保留** `v3.1` 的 **normal-only warm start**
- **保留** `v3.1` 的近静止 bias 抑制逻辑（没有重新把 bias 大幅放回去）
- **加入** 更强的 **face preference / edge-edge 选择惩罚**
- **加入** 基于上一帧 pair normal 的 **轻量 SAT hysteresis**
- **加入** `friction-first / normal-last` 的速度求解顺序
- **没有加入** 全面恢复 bias 的方案，因为它在 BoxStack 上会明显回退

## 为什么没有沿用“平滑 bias floor”方案

实际回归里，只要把近静止 bias 重新抬起来（哪怕是平滑抬），
`BoxStack` 都会从稳定睡死退化成长期微动。

也就是说：
- `v3.1` 稳金字塔的重要原因之一，就是它对 near-rest face contact 足够克制
- irregular pile 需要的不是“把 bias 全局加回来”
- 更像是需要 **更稳定的接触特征选择** 和 **更连续的约束方向**

所以 v4 最终走的是：
**先稳定 contact feature，再调求解顺序；不先重开 bias 闸门。**

## 这版 v4 的核心改动点

### 1. SAT 选轴更偏向 face manifold

当 face axis 和 edge-edge axis 的 overlap 很接近时，
这版会对 edge-edge 轴施加一个额外选择偏置，避免太容易从稳定的 face manifold
跳到脆弱的单点 edge-edge contact。

### 2. 复用上一帧的 pair normal 作为轻量 hysteresis

这不是完整 feature-id manifold，也不是工业级 cache。
只是用上一帧缓存的 normal 在 SAT 选轴时做一个很小的优先级提示。
这样做的目的，是让 near-rest contact 更不容易帧间翻法线。

### 3. 速度求解改为 friction-first / normal-last

这一步和 `合成.md` 的 Jolt 摘要一致：
每次 velocity iteration 里，先解切向摩擦，再解法向非穿透。

在当前实现里，这一步单独上时收益有限；
但和上面的 SAT feature 稳定化组合后，irregular pile 的停稳结果更好。

## 和布娃娃路线的关系

我判断：**当前路线天然兼容后续 ragdoll，不需要推翻。**

原因不是因为现在已经有 joint，而是因为底层关键语义已经在对的轨道上：

- `position_` 表示 **COM 世界位置**
- `rotation_` 是刚体参考帧旋转
- `center_of_mass_offset_local_` 已经接通
- `collider_local_offset_ / collider_local_rotation_` 已经接通
- 接触求解已经使用 **点速度**、**世界逆惯量**、**角向冲量**

这些正是后续做：
- ball-socket
- hinge
- cone / swing-twist 之类简化关节

所需要的共同底座。

### 对布娃娃真正还缺什么

当前还缺的是：

1. **joint constraint 层**
2. 关节 anchor / local frame 数据
3. joints 与 contacts 的统一迭代调度
4. 更合适的 limb shape（未来可以先 box 近似，之后再考虑 capsule）

### 对你“单文件 + JS 迁移”的意义

这版 v4 没有引入 island / CCD / full tangent warm start / 真正 contact manager。
因此复杂度仍然在单文件可控范围内。

换句话说：

- **这版 v4 依然适合继续朝单文件收敛**
- 它没有把项目带进“只能在 C++ 里复杂工业实现”的方向
- 之后如果做 ragdoll，最合理的是在这个基础上再加一个**很薄的 joint 约束层**

## 我这边做的 headless 回归结论

### BoxStack
- v4：可稳定睡死，后段 0 漂移

### Deterministic irregular pile
- v4：30/30 睡死
- 后段平均漂移约 `0.0076`
- 最大漂移约 `0.1222`

这说明 v4 至少在“保持金字塔不回退”的前提下，
把 irregular pile 的长期磨蹭收得比 v3.1 更干净。

## 建议的下一步

先做你本地肉眼回归。
如果这版在你看到的：
- 金字塔
- 乱堆 box
- F 键大量抛撒小方块
- Scene 7 的 boxified limb pile

几个观察场景里都没有明显回退，
那么它就可以作为新的主线版本。

再往后我建议的顺序是：
1. 先决定 v4 是否转正
2. 若转正，再讨论“是否开始收紧为单文件后端”
3. 再下一步才是“ragdoll V1 需要的 joint 最小集合”

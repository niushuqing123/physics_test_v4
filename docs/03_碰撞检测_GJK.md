# Jolt 刚体物理核心算法 — GJK 碰撞检测

对应 Jolt 源码: `Geometry/GJKClosestPoint.h` 和 `Geometry/ClosestPoint.h`。
基于 Gino van den Bergen 的方法。

## 一、GJK 算法概述

GJK (Gilbert-Johnson-Keerthi) 求解**两个凸体之间的最短距离**。

核心思想：在 Minkowski 差 `A ⊖ B = {a-b | a∈A, b∈B}` 上找**离原点最近的点**。
- 如果最近点距离 > 0 → 不相交，距离就是两体间距
- 如果原点在 Minkowski 差内部 → 相交（交给 EPA 处理）

## 二、Support 函数

GJK 只需要每个形状提供一个 Support 函数：

```
Support_A(d) = argmax_{a∈A} (a · d)   // A中在d方向最远的点
Support_B(d) = argmax_{b∈B} (b · d)

// Minkowski差的Support:
Support_{A⊖B}(d) = Support_A(d) - Support_B(-d)
```

## 三、GJK 核心循环 (GetClosestPoints)

```
输入: 凸体A, 凸体B, tolerance, maxDistSq
输出: 最近点 pointA (在A上), pointB (在B上), 距离²

simplex Y = {}          // 单纯形点集 (最多4个点)
P[], Q[] = {}           // 对应A上/B上的支撑点
v = 初始分离轴           // 任意非零向量
v_len_sq = |v|²
prev_v_len_sq = FLT_MAX

loop:
    // 1. 取支撑点
    p = A.GetSupport(v)       // A在v方向最远
    q = B.GetSupport(-v)      // B在-v方向最远
    w = p - q                 // Minkowski差上的新点

    // 2. 分离检测: 如果w在v方向的投影为负，距离太远
    dot = v · w
    if dot < 0 且 dot² > v_len_sq * maxDistSq:
        return FLT_MAX  // 距离超过阈值，提前返回

    // 3. 加入单纯形
    Y.add(w); P.add(p); Q.add(q)

    // 4. 求单纯形上离原点最近的点
    // 调用 ClosestPoint 子系统（见下文）
    (v_new, set) = GetClosestPointOnSimplex(Y)

    // 5. 判断终止条件
    if set == 0xF:           // 4个点都保留 = 原点在四面体内部
        v = (0,0,0)          // 碰撞! 距离=0
        break

    if |v_new|² ≤ tolerance²:  // 足够近
        v = (0,0,0); break

    if |v_new|² ≤ FLT_EPSILON * max(|Y_i|²):  // 机器精度
        v = (0,0,0); break

    // 6. 收敛检测: 距离不再减小
    v = -v_new               // 下一个搜索方向 = 反向最近点
    if prev_v_len_sq - |v_new|² ≤ FLT_EPSILON * prev_v_len_sq:
        break                // 已收敛

    prev_v_len_sq = |v_new|²

    // 7. 精简单纯形: 只保留 set 中的点
    RemovePointsNotInSet(Y, P, Q, set)

// 8. 用重心坐标插值出A/B上的最近点
pointA, pointB = CalculateFromBarycentricCoords(Y, P, Q)
return |v|²
```

## 四、ClosestPoint 子系统

`ClosestPoint.h` 提供离原点最近点的计算，分4种情况:

### 4.1 线段上最近点 (2点)
```
GetClosestPointOnLine(A, B):
    AB = B - A
    denom = |AB|²
    if denom < ε²:  // 退化为点
        return 距原点更近的那个
    v = -A·AB / denom    // B的重心坐标
    u = 1 - v
    if v ≤ 0: return A (set=01)
    if u ≤ 0: return B (set=10)
    return u*A + v*B (set=11)
```

### 4.2 三角形上最近点 (3点)

使用 Christer Ericson 的 "Real-Time Collision Detection" 算法:

```
GetClosestPointOnTriangle(A, B, C):
    // 1. 选择最短边参与计算以提高精度
    // 2. 计算三角形法线 n = 最短边 × 另一边
    // 3. 测试原点在各边的哪一侧
    // 4. 分情况返回:
    //    - 原点投影在三角形内部 → 返回投影点
    //    - 原点最近某条边 → 退化到线段
    //    - 原点最近某个顶点 → 直接返回顶点
    
    // 重心坐标 (u,v,w) 满足 u+v+w=1
    // 使用行列式公式计算:
    d00 = v0·v0;  d01 = v0·v1;  d11 = v1·v1
    denom = d00*d11 - d01*d01  // ≥ 0 (柯西不等式)
    if denom < 1e-10: // 退化三角形，退到最长边
        ...
    返回 set 位掩码标识哪些顶点参与
```

### 4.3 四面体上最近点 (4点)

```
GetClosestPointOnTetrahedron(A, B, C, D):
    // 1. 检查原点是否在四面体内部
    //    通过测试原点在每个面的哪一侧
    // 2. 如果在内部: set = 0xF, 返回原点本身
    // 3. 如果在外部: 测试4个面，找最近的面
    //    对最近的面退化到三角形情况
```

### 4.4 GJK的模板优化

Jolt 有一个关键优化 `LastPointPartOfClosestFeature`:
- 最后加入的点必然在最近特征中（因为它是沿着最近方向取的支撑点）
- 这允许跳过很多不必要的子特征测试

## 五、Convex Radius (膨胀半径)

Jolt对所有凸体形状都有一个 `ConvexRadius` 概念:
- GJK 核心运算使用**不含radius的形状** (ExcludingConvexRadius)
- 求出距离后再加上两体的 radius 之和
- 如果 `distance ≤ radiusA + radiusB` → 碰撞

这样做的好处: 
- GJK 收敛更快 (不含radius的Minkowski差更"尖锐")
- 圆角效果自动获得
- EPA 则使用**含radius的形状** (IncludingConvexRadius)

```
// EPAPenetrationDepth.h: GetPenetrationDepthStepGJK
combined_radius = radiusA + radiusB
dist_sq = GJK.GetClosestPoints(A_excl, B_excl, ...)

if dist_sq > combined_radius²:
    return NotColliding

if dist_sq > 0:
    // 在convex radius内碰撞
    // 调整接触点: 沿分离方向各偏移自己的radius
    pointA += direction * (radiusA / dist)
    pointB -= direction * (radiusB / dist)
    return Colliding

// dist_sq == 0: 深度穿透，需要EPA
return Indeterminate
```

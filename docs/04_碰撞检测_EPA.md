# Jolt 刚体物理核心算法 — EPA 穿透深度计算

对应 Jolt 源码: `Geometry/EPAPenetrationDepth.h` 和 `Geometry/EPAConvexHullBuilder.h`。
基于 Gino van den Bergen 的 Expanding Polytope Algorithm。

## 一、EPA 的作用

当 GJK 判定两凸体**相交** (原点在 Minkowski 差内部) 后，
GJK 只能告诉你"相交了"，但无法给出：
- **穿透深度** (penetration depth)
- **穿透方向** (分离法线)
- **最近接触点对**

EPA 就是解决这个问题的。

## 二、EPA 算法概述

```
输入: GJK的最终单纯形 (包含原点的3~4个点), 两凸体的Support函数
输出: 穿透法线, 穿透深度, A/B上的接触点

思路:
  把GJK的单纯形扩展成一个凸包 (polytope)
  不断找凸包上离原点最近的面
  用该面的法线方向取新的支撑点来扩展凸包
  直到新点不再显著靠近原点 → 收敛
```

## 三、详细算法流程

### Step 1: 初始化凸包

```
// 从GJK得到的单纯形可能是1~4个点
// 需要保证至少4个点形成非退化四面体

if numPoints < 4:
    // 补充支撑点: 沿坐标轴方向取新支撑点
    // 直到有4个不共面的点
    补点方向 = ±x, ±y, ±z 依次尝试

// 用这4+个点构建初始凸包 (三角形面集合)
convexHull = EPAConvexHullBuilder(points)

// 验证: 原点必须在凸包内部
// 如果不在 → GJK判断有误或数值问题 → 返回false
```

### Step 2: 扩展循环

```
loop:
    // 1. 从所有三角形面中找离原点最近的面
    closestTriangle = 凸包面中 (面到原点距离²) 最小的
    closestDistSq = 该面到原点的距离²
    normal = 该面的外法线

    // 2. 沿法线方向取新支撑点
    w = Support_{A⊖B}(normal)   // Minkowski差的新支撑点
    // 同时记录 p=A.Support(normal), q=B.Support(-normal)

    // 3. 收敛检测
    newDistSq = (w · normal)² / |normal|²
    if newDistSq - closestDistSq < tolerance * closestDistSq:
        break  // 新点没有显著改善，收敛了

    // 4. 将新点加入凸包
    // - 删除所有"面向新点"的三角形 (新点在其正面)
    // - 用新点和暴露的边缘构建新三角形
    // 这就是凸包的增量构建
    convexHull.AddPoint(w, p, q)

// 5. 从最近面的重心坐标插值出A/B上的接触点
// 最近面有3个顶点Y[i], 对应P[i]和Q[i]
// 用重心坐标 (u,v,w) 插值:
pointA = u*P[0] + v*P[1] + w*P[2]
pointB = u*Q[0] + v*Q[1] + w*Q[2]

// 穿透法线 = normalize(pointA - pointB) 或 closestTriangle法线方向
// 穿透深度 = |pointA - pointB|
```

## 四、EPAConvexHullBuilder

Jolt 不使用简单的"三角形分裂"方法，而是真正的增量凸包构建:

```
数据结构:
    Triangle {
        int indices[3];       // 顶点索引
        Vec3 normal;          // 外法线
        float closestDistSq;  // 到原点距离²
        Triangle* adjacent[3]; // 邻接三角形
    }
    优先队列 (按closestDistSq排序)

AddPoint(newPoint):
    1. 标记所有面向新点的三角形 (newPoint在其法线正侧)
    2. 收集这些三角形的外边界 (horizon edges)
       — 即: 一侧三角形被删除、另一侧保留的边
    3. 删除所有被标记的三角形
    4. 用新点 + 每条horizon edge 生成新三角形
    5. 设置邻接关系
    6. 加入优先队列
```

**这种方法的优势**: 避免产生极细长的三角形 (文章中提到的问题)，
保持凸包质量良好。

## 五、两步法: GJK + EPA 的完整流程

Jolt 在 `EPAPenetrationDepth.h` 中将两者串联:

```
// Step 1: GJK
status = GetPenetrationDepthStepGJK(
    A_excludingRadius, radiusA,
    B_excludingRadius, radiusB,
    tolerance, ioV, outPointA, outPointB)

switch status:
    case NotColliding:
        return false  // 无碰撞

    case Colliding:
        // GJK已找到接触点 (在convex radius范围内)
        // outPointA/B 已调整了 radius
        return true

    case Indeterminate:
        // 深度穿透，需要EPA
        // Step 2: EPA
        return GetPenetrationDepthStepEPA(
            A_includingRadius,  // 注意这里用含radius版本
            B_includingRadius,
            tolerance, outV, outPointA, outPointB)
```

## 六、简化版仿写要点

如果只处理简单形状 (球、盒)，可以大幅简化:

| 形状对 | GJK需要？ | EPA需要？ | 替代方案 |
|--------|----------|----------|---------|
| 球 vs 球 | 不需要 | 不需要 | 直接解析: dist = |c1-c2| - r1 - r2 |
| 球 vs 盒 | 不需要 | 不需要 | 最近点投影公式 |
| 盒 vs 盒 | 可以SAT替代 | 可以SAT替代 | SAT (15轴) 更直觉 |
| 凸体 vs 凸体 | **需要** | **需要** | 无替代 |

对于完整凸体，GJK+EPA 是唯一实用的通用算法。

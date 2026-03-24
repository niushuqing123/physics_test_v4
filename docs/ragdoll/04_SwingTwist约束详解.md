# Jolt 布娃娃系统调查报告 — Part 4: SwingTwist 约束详解

## 一、为什么是 SwingTwistConstraint

布娃娃关节本质上是**球窝关节**（ball-and-socket joint），需要限制：
1. **摆动 (Swing)**: 骨骼偏离其自然轴线的旋转（如手臂侧举、前抬）
2. **扭转 (Twist)**: 骨骼沿自身轴线的旋转（如前臂旋转手腕）

`SwingTwistConstraint` 正是为此设计的专用约束，是 Jolt 布娃娃的**首选约束类型**。

**源文件**: 
- `Jolt/Physics/Constraints/SwingTwistConstraint.h/.cpp`
- `Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h`

---

## 二、约束参数

### 2.1 SwingTwistConstraintSettings

```cpp
struct SwingTwistConstraintSettings {
    // === 参考帧（关节位置和轴向）===
    EConstraintSpace  mSpace;      // WorldSpace 或 LocalSpace
    RVec3  mPosition1, mPosition2; // body1/body2 上的约束锚点
    Vec3   mTwistAxis1, mTwistAxis2; // 扭转轴（沿骨骼延伸方向）
    Vec3   mPlaneAxis1, mPlaneAxis2; // 平面轴（与twist轴垂直的参考方向）
    
    // === 摆动限制类型 ===
    ESwingType  mSwingType;  // Cone (锥形) 或 Pyramid (金字塔形)
    
    // === 摆动角度限制 ===
    float  mNormalHalfConeAngle;  // 法线方向半锥角 (rad)
    float  mPlaneHalfConeAngle;   // 平面方向半锥角 (rad)
    
    // === 扭转角度限制 ===
    float  mTwistMinAngle;        // 最小扭转角 (rad), ∈ [-π, π]
    float  mTwistMaxAngle;        // 最大扭转角 (rad), ∈ [-π, π]
    
    // === 摩擦 ===
    float  mMaxFrictionTorque;    // 无电机时的摩擦扭矩 (N·m)
    
    // === 电机设置 ===
    MotorSettings  mSwingMotorSettings;  // 摆动电机
    MotorSettings  mTwistMotorSettings;  // 扭转电机
};
```

### 2.2 三轴示意

```
         PlaneAxis (Z)
              |
              |  NormalHalfConeAngle
              |  ↕
    ----------+----------→ TwistAxis (X) ← 骨骼延伸方向
              |  ↕
              |  PlaneHalfConeAngle
              |
         NormalAxis (Y) = PlaneAxis × TwistAxis
```

- **TwistAxis**: 沿骨骼延伸的方向，扭转围绕此轴
- **PlaneAxis**: 约束参考平面的法线方向
- **NormalAxis**: 自动计算 = PlaneAxis × TwistAxis

---

## 三、内部约束结构

SwingTwistConstraint 内部由**三个约束部件**组成：

```cpp
class SwingTwistConstraint {
    // 1. 位置约束：锁定两个锚点位置
    PointConstraintPart      mPointConstraintPart;     // 3-DOF 位置
    
    // 2. 摆动+扭转限制
    SwingTwistConstraintPart mSwingTwistConstraintPart; // 角度限制
    
    // 3. 电机约束（3轴）
    AngleConstraintPart      mMotorConstraintPart[3];   // twist + swing_y + swing_z
};
```

### 3.1 约束空间内的旋转分解

关键数学：将 body2 相对于 body1 的旋转分解为 swing 和 twist：

```
q = (body1_rotation * constraintToBody1)^-1 * (body2_rotation * constraintToBody2)
```

然后将 `q` 分解为：
```
q = q_swing * q_twist
```
其中：
- `q_twist` 只有 x 分量 + w（绕 X 轴旋转）
- `q_swing` 只有 y, z 分量 + w（绕 Y, Z 轴旋转）

---

## 四、摆动限制的两种模式

### 4.1 Cone（锥形）— 默认

将 `(q_swing.y, q_swing.z)` 看作椭圆上的点：

$$\frac{q_{swing}.y^2}{\sin^2(\theta_{swingY}/2)} + \frac{q_{swing}.z^2}{\sin^2(\theta_{swingZ}/2)} \leq 1$$

如果在椭圆外，投影到椭圆边界上的最近点。

**优点**: 平滑的锥形限制空间
**缺点**: 大角度时锥形会变形

### 4.2 Pyramid（金字塔形）

将 swing 分解为绕 Y 和 Z 轴的独立旋转角度：

```
halfAngleY = atan2(q_swing.y, q_swing.w)
halfAngleZ = atan2(q_swing.z, q_swing.w)
```

各自独立夹紧到 `[minAngle, maxAngle]`。

**优点**: 支持非对称限制（minY ≠ maxY）
**缺点**: 角落处限制范围不够平滑

### 4.3 特殊情况优化

约束部件通过 flag 位快速判断各轴状态：

| Flag | 含义 | 条件 |
|------|------|------|
| `TwistXLocked` | 扭转锁死 | angle < 0.5° |
| `TwistXFree` | 扭转自由 | angle > 179.5° |
| `SwingYLocked` | Y摆动锁死 | 如肘部 |
| `SwingYFree` | Y摆动自由 | |
| `SwingZLocked` | Z摆动锁死 | |
| `SwingZFree` | Z摆动自由 | |

---

## 五、电机系统

### 5.1 MotorSettings

```cpp
struct MotorSettings {
    SpringSettings mSpringSettings;  // 弹簧参数
    // 默认: frequency=2.0Hz, damping=1.0

    float mMinForceLimit, mMaxForceLimit;   // 力限制 (线性电机用)
    float mMinTorqueLimit, mMaxTorqueLimit; // 扭矩限制 (角度电机用)
};
```

### 5.2 三种电机状态

```cpp
enum EMotorState {
    Off,       // 关闭
    Velocity,  // 速度模式：驱动到目标角速度
    Position   // 位置模式：驱动到目标朝向
};
```

### 5.3 位置模式的驱动原理

```
// 计算目标朝向与当前朝向的差异
diff = q_current^-1 * q_target

// 角度误差（小角度近似）
rotation_error = -2.0 * diff.xyz

// 用弹簧约束驱动误差收敛到零
// 调用 AngleConstraintPart::CalculateConstraintPropertiesWithSettings()
// 使用 SpringSettings 中的频率和阻尼
```

**布娃娃中使用电机（`DriveToPoseUsingMotors`）的流程**：
```cpp
for each joint with constraint:
    SwingTwistConstraint* st = ...;
    st->SetSwingMotorState(EMotorState::Position);
    st->SetTwistMotorState(EMotorState::Position);
    st->SetTargetOrientationBS(jointState.mRotation);
```

---

## 六、约束求解流程

每个物理步的求解过程：

```
1. SetupVelocityConstraint(dt)
   ├─ 计算 PointConstraintPart 属性（位置锁定）
   ├─ 计算当前 swing/twist 旋转
   ├─ 调用 SwingTwistConstraintPart::CalculateConstraintProperties()
   │   ├─ 分解 q → q_swing, q_twist
   │   ├─ 夹紧到限制范围
   │   └─ 设置约束 Jacobian + bias
   └─ 设置电机/摩擦约束部件

2. WarmStartVelocityConstraint(ratio)
   ├─ 对所有约束部件应用上一帧的累积冲量 × 比率

3. SolveVelocityConstraint(dt)  [在PGS迭代中多次调用]
   ├─ 求解 PointConstraintPart（位置）
   ├─ 求解 SwingTwistConstraintPart（角度限制）
   ├─ 求解 MotorConstraintPart[0..2]（电机/摩擦）
   │   └─ 电机扭矩夹紧到 MotorSettings 的限制内

4. SolvePositionConstraint(dt, baumgarte)
   ├─ Baumgarte 位置修正
```

---

## 七、与其他约束类型的对比

| 约束类型 | 自由度 | 适用关节 | 布娃娃推荐度 |
|----------|--------|----------|-------------|
| **SwingTwistConstraint** | 位置锁定 + 摆动/扭转限制 + 电机 | 肩、髋、头、脊柱 | ★★★★★ |
| HingeConstraint | 1轴旋转 | 肘、膝 | ★★★★ |
| ConeConstraint | 锥形限制(无扭转) | 简化关节 | ★★★ |
| PointConstraint | 仅位置锁定 | 调试用 | ★★ |
| FixedConstraint | 完全固定 | 调试用 | ★ |

在 Jolt 的示例中，所有关节统一使用 SwingTwistConstraint，但通过设置 `normalAngle=0°` 来模拟铰链效果（如膝盖只能弯曲不能侧向）。

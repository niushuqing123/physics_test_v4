// ============================================================================
// RigidBodyPhysics.java
// 单文件刚体物理引擎 — Java 版（移植自 physics_null_v4.cpp）
// ============================================================================
//
// 功能特性：
//   - Box / Sphere / Capsule 碰撞体（Sphere / Capsule 近似为 AABB-aligned box）
//   - Static / Kinematic / Dynamic 三种运动类型
//   - OBB-OBB SAT 窄相碰撞检测（15 轴：3+3+9）
//   - 面接触流形裁切（Sutherland-Hodgman，每对最多 4 个接触点）
//   - 顺序脉冲速度约束求解 + 库仑摩擦（两切线轴）
//   - 伪速度位置修正（Baumgarte）
//   - 法线向量 warm-start（帧间缓存）
//   - 基于速度阈值的休眠系统
//   - 质心偏移（COM offset）与碰撞体局部姿态支持
//
// 用法示例：
//   RigidBodyPhysics.Config cfg = new RigidBodyPhysics.Config();
//   RigidBodyPhysics sim = new RigidBodyPhysics(cfg);
//
//   RigidBodyPhysics.BodyDesc desc = new RigidBodyPhysics.BodyDesc();
//   desc.motionType = RigidBodyPhysics.MotionType.DYNAMIC;
//   desc.shapeType  = RigidBodyPhysics.ShapeType.BOX;
//   desc.boxHalfExtents = new RigidBodyPhysics.Vec3(0.5f, 0.5f, 0.5f);
//   desc.mass = 1.0f;
//   int handle = sim.addBody(desc);
//
//   sim.step(1.0f / 60.0f);
//
//   RigidBodyPhysics.BodyState state = sim.getBodyState(handle);
// ============================================================================

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RigidBodyPhysics {

    // ========================================================================
    // 公开 API 类型
    // ========================================================================

    /** 刚体的运动类型 */
    public enum MotionType {
        /** 静态体：不移动，质量无限大，用于地面/墙壁 */
        STATIC,
        /** 运动学体：由代码直接控制位置，不被物理力影响，但可以推动 Dynamic 体 */
        KINEMATIC,
        /** 动态体：受力和碰撞响应驱动 */
        DYNAMIC
    }

    /** 碰撞体形状类型 */
    public enum ShapeType {
        /** 盒子（OBB） */
        BOX,
        /** 球体（内部近似为盒子） */
        SPHERE,
        /** 胶囊体（内部近似为盒子） */
        CAPSULE
    }

    /** 三维向量 */
    public static final class Vec3 {
        public float x, y, z;
        public Vec3() {}
        public Vec3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public Vec3 copy() { return new Vec3(x, y, z); }
        @Override public String toString() { return String.format("Vec3(%.4f, %.4f, %.4f)", x, y, z); }
    }

    /** 单位四元数，存储顺序为 (x, y, z, w)，默认为单位旋转 */
    public static final class Quat {
        public float x, y, z, w;
        public Quat() { w = 1.0f; }
        public Quat(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
        public Quat copy() { return new Quat(x, y, z, w); }
        @Override public String toString() { return String.format("Quat(%.4f, %.4f, %.4f, %.4f)", x, y, z, w); }
    }

    /** 创建刚体时传入的描述/配置数据 */
    public static final class BodyDesc {
        /** 世界空间初始位置（Transform 原点，非质心） */
        public Vec3 position = new Vec3(0, 0, 0);
        /** 世界空间初始旋转（单位四元数） */
        public Quat rotation = new Quat(0, 0, 0, 1);
        /** 各轴缩放因子 */
        public Vec3 scale = new Vec3(1, 1, 1);
        /** 运动类型 */
        public MotionType motionType = MotionType.DYNAMIC;
        /** 碰撞体形状类型 */
        public ShapeType shapeType = ShapeType.BOX;

        // --- 形状参数 ---
        /** Box 碰撞体半尺寸 (hx, hy, hz) */
        public Vec3 boxHalfExtents = new Vec3(0.5f, 0.5f, 0.5f);
        /** Sphere 半径 */
        public float sphereRadius = 0.5f;
        /** Capsule 截面半径 */
        public float capsuleRadius = 0.5f;
        /** Capsule 半高（不含端球） */
        public float capsuleHalfHeight = 0.5f;

        // --- 物理属性 ---
        /** 质量 (kg)，仅 Dynamic 有效 */
        public float mass = 1.0f;
        /** 是否受重力影响 */
        public boolean useGravity = true;
        /** 摩擦系数（0=无摩擦，1=高摩擦） */
        public float friction = 0.6f;
        /** 线性速度阻尼 */
        public float linearDamping = 0.0f;
        /** 角速度阻尼 */
        public float angularDamping = 0.0f;

        // --- 质心与碰撞体局部姿态 ---
        /** 质心相对 Transform 原点的局部偏移（body-local） */
        public Vec3 centerOfMassOffset = new Vec3(0, 0, 0);
        /** 碰撞体相对 Transform 原点的局部位置偏移 */
        public Vec3 colliderLocalOffset = new Vec3(0, 0, 0);
        /** 碰撞体相对刚体的局部旋转 */
        public Quat colliderLocalRotation = new Quat(0, 0, 0, 1);

        // --- 初始速度 ---
        public Vec3 linearVelocity = new Vec3(0, 0, 0);
        public Vec3 angularVelocity = new Vec3(0, 0, 0);
        /** 初始是否处于睡眠状态 */
        public boolean sleeping = false;
    }

    /** 从物理引擎读回的刚体状态 */
    public static final class BodyState {
        /** 世界空间位置（Transform 原点，非质心） */
        public Vec3 position;
        /** 世界空间旋转 */
        public Quat rotation;
        /** 线速度 (m/s) */
        public Vec3 linearVelocity;
        /** 角速度 (rad/s) */
        public Vec3 angularVelocity;
        /** 是否处于休眠状态 */
        public boolean sleeping;
    }

    /** 接触点调试信息 */
    public static final class ContactInfo {
        public Vec3 point;
        public Vec3 normal;
        public float penetration;
    }

    /** 刚体调试信息（AABB、质心、碰撞体中心等） */
    public static final class BodyDebugInfo {
        /** 刚体句柄 */
        public int handle;
        /** 质心世界坐标 */
        public Vec3 comWorld;
        /** Transform 原点世界坐标 */
        public Vec3 transformOrigin;
        /** 碰撞体中心世界坐标 */
        public Vec3 colliderCenter;
        /** AABB 最小点 */
        public Vec3 aabbMin;
        /** AABB 最大点 */
        public Vec3 aabbMax;
    }

    /** 物理世界全局配置 */
    public static final class Config {
        /** 全局重力加速度 (m/s²)，默认 Y 轴向下 */
        public Vec3 gravity = new Vec3(0, -9.81f, 0);
    }

    // ========================================================================
    // 私有常量
    // ========================================================================

    private static final float EPS                         = 1e-6f;
    private static final float SLEEP_LIN_THRESHOLD_SQ      = 0.05f * 0.05f;
    private static final float SLEEP_ANG_THRESHOLD_SQ      = 0.05f * 0.05f;
    private static final int   SLEEP_TICKS_REQUIRED        = 30;
    private static final int   VELOCITY_ITERATIONS         = 10;
    private static final int   POSITION_ITERATIONS         = 4;
    private static final float VELOCITY_BAUMGARTE          = 0.08f;
    private static final float POSITION_BAUMGARTE          = 0.25f;
    private static final float PENETRATION_SLOP            = 0.01f;
    private static final float MAX_POSITION_CORRECTION     = 0.08f;
    private static final float MAX_ANGULAR_POSITION_STEP   = 0.35f;
    private static final float RESTITUTION_THRESHOLD       = 1.0f;
    private static final float DUPLICATE_POINT_EPS_SQ      = 1e-4f * 1e-4f;
    private static final float WARM_START_MATCH_DIST_SQ    = 0.08f * 0.08f;
    private static final float WARM_START_NORMAL_DOT_MIN   = 0.92f;
    private static final float RESTING_BIAS_VEL_THRESHOLD  = 0.25f;
    private static final float RESTING_BIAS_PENETRATION    = 0.02f;
    private static final float WAKE_NORMAL_SPEED_THRESHOLD = 0.35f;
    private static final float WAKE_TANGENT_SPEED_THRESHOLD= 0.25f;
    private static final float SAT_EDGE_AXIS_BIAS          = 0.05f;
    private static final float SAT_PREFERRED_NORMAL_DOT    = 0.96f;
    private static final float SAT_PREFERRED_NORMAL_BIAS   = 0.015f;
    private static final float WAKE_PENETRATION_THRESHOLD  = 0.03f;

    // ========================================================================
    // 内部数据类型
    // ========================================================================

    private static final class ContactData {
        int a, b;
        float[] normal      = new float[3];
        float[] point       = new float[3];
        float   penetration;
        float[] r1          = new float[3];
        float[] r2          = new float[3];
        float[] tangent1    = new float[3];
        float[] tangent2    = new float[3];
        float   combinedFriction;
        float   restitution;
        float   bias;
        float   normalEffMass;
        float   t1EffMass;
        float   t2EffMass;
        float   normalLambda;
        float   t1Lambda;
        float   t2Lambda;
    }

    private static final class ManifoldData {
        int     a, b;
        float[] normal      = new float[3];
        float[] repPoint    = new float[3];
        float   penetration;
        int     firstContact;
        int     contactCount;
    }

    private static final class CachedContactPoint {
        float[] point       = new float[3];
        float   normalLambda;
        float   t1Lambda;
        float   t2Lambda;
    }

    private static final class CachedManifold {
        float[]             normal  = new float[3];
        int                 count;
        CachedContactPoint[] points = new CachedContactPoint[4];
        CachedManifold() { for (int i = 0; i < 4; i++) points[i] = new CachedContactPoint(); }
    }

    private static final class SatResult {
        boolean separated   = true;
        float[] normal      = { 0, 1, 0 };
        float   penetration;
        int     axisIndex   = -1;
        int     edgeA       = -1;
        int     edgeB       = -1;
        float[] point       = new float[3];
    }

    private static final class ContactCandidate {
        float[] point       = new float[3];
        float   penetration;
    }

    // ========================================================================
    // 成员变量（SoA 模式，以句柄 handle 为索引）
    // ========================================================================

    private final Config config;

    // 存活标志与基本属性
    private int              bodyCount = 0;
    private final List<Boolean>     alive           = new ArrayList<>();
    private final List<MotionType>  motionType      = new ArrayList<>();
    private final List<boolean[]>   solverEnabled   = new ArrayList<>();  // [0]

    // 运动状态（position = COM 世界坐标）
    private final List<float[]>     position        = new ArrayList<>();  // xyz
    private final List<float[]>     rotation        = new ArrayList<>();  // quat xyzw
    private final List<float[]>     scale           = new ArrayList<>();  // xyz
    private final List<float[]>     prevPosition    = new ArrayList<>();  // xyz

    // 速度
    private final List<float[]>     linearVelocity  = new ArrayList<>();  // xyz
    private final List<float[]>     angularVelocity = new ArrayList<>();  // xyz

    // 质量属性
    private final List<float[]>     mass            = new ArrayList<>();  // [0]
    private final List<float[]>     invMass         = new ArrayList<>();  // [0]
    private final List<float[]>     invInertiaLocal = new ArrayList<>();  // xyz 对角线

    // 材质
    private final List<float[]>     friction        = new ArrayList<>();  // [0]
    private final List<float[]>     linearDamping   = new ArrayList<>();  // [0]
    private final List<float[]>     angularDamping  = new ArrayList<>();  // [0]
    private final List<boolean[]>   useGravity      = new ArrayList<>();  // [0]

    // 休眠
    private final List<boolean[]>   sleeping        = new ArrayList<>();  // [0]
    private final List<int[]>       sleepCounter    = new ArrayList<>();  // [0]

    // 几何（碰撞体）
    private final List<float[]>     comOffsetLocal      = new ArrayList<>(); // xyz
    private final List<float[]>     colliderLocalOffset = new ArrayList<>(); // xyz
    private final List<float[]>     colliderLocalRot    = new ArrayList<>(); // quat xyzw
    private final List<float[]>     boxHalfExtLocal     = new ArrayList<>(); // xyz
    private final List<float[]>     boxHalfExtWorld     = new ArrayList<>(); // xyz
    private final List<float[]>     colliderCenterWorld = new ArrayList<>(); // xyz
    private final List<float[]>     aabbMin             = new ArrayList<>(); // xyz
    private final List<float[]>     aabbMax             = new ArrayList<>(); // xyz
    // OBB proxy: center(0-2) + axis0(3-5) + axis1(6-8) + axis2(9-11) + halfExtents(12-14)
    private final List<float[]>     obbProxy            = new ArrayList<>(); // float[15]

    // 力累加器
    private final List<float[]>     forceAcc        = new ArrayList<>();  // xyz
    private final List<float[]>     torqueAcc       = new ArrayList<>();  // xyz

    // 活跃句柄列表
    private final List<Integer>     activeHandles   = new ArrayList<>();

    // 碰撞检测输出
    private final List<int[]>             broadphasePairs = new ArrayList<>();
    private final List<ContactData>       contacts        = new ArrayList<>();
    private final List<ManifoldData>      manifolds       = new ArrayList<>();

    // Warm-start 缓存（pair key → CachedManifold）
    private Map<Long, CachedManifold> prevCache = new HashMap<>();
    private Map<Long, CachedManifold> nextCache = new HashMap<>();
    private float prevStepDt = 0.0f;

    // ========================================================================
    // 构造函数
    // ========================================================================

    public RigidBodyPhysics(Config cfg) {
        this.config = (cfg != null) ? cfg : new Config();
    }

    // ========================================================================
    // 公开 API
    // ========================================================================

    /**
     * 向物理世界添加一个刚体。
     * @param desc 刚体描述/配置
     * @return 刚体句柄（非负整数）
     */
    public int addBody(BodyDesc desc) {
        int handle = alive.size();

        alive.add(true);
        motionType.add(MotionType.STATIC);   // 由 initBodyFromDesc 更新
        solverEnabled.add(new boolean[]{false});

        position.add(new float[3]);
        rotation.add(new float[]{ 0, 0, 0, 1 });
        scale.add(new float[]{ 1, 1, 1 });
        prevPosition.add(new float[3]);

        linearVelocity.add(new float[3]);
        angularVelocity.add(new float[3]);

        mass.add(new float[1]);
        invMass.add(new float[1]);
        invInertiaLocal.add(new float[3]);

        friction.add(new float[1]);
        linearDamping.add(new float[1]);
        angularDamping.add(new float[1]);
        useGravity.add(new boolean[1]);

        sleeping.add(new boolean[1]);
        sleepCounter.add(new int[1]);

        comOffsetLocal.add(new float[3]);
        colliderLocalOffset.add(new float[3]);
        colliderLocalRot.add(new float[]{ 0, 0, 0, 1 });
        boxHalfExtLocal.add(new float[]{ 0.5f, 0.5f, 0.5f });
        boxHalfExtWorld.add(new float[]{ 0.5f, 0.5f, 0.5f });
        colliderCenterWorld.add(new float[3]);
        aabbMin.add(new float[3]);
        aabbMax.add(new float[3]);
        obbProxy.add(new float[15]);

        forceAcc.add(new float[3]);
        torqueAcc.add(new float[3]);

        activeHandles.add(handle);
        bodyCount++;

        initBodyFromDesc(handle, desc);
        updateDerivedBodyState(handle);
        return handle;
    }

    /**
     * 从物理世界移除一个刚体。
     * @param handle addBody 返回的句柄
     */
    public void removeBody(int handle) {
        if (!isValidHandle(handle)) return;
        alive.set(handle, false);
        activeHandles.remove(Integer.valueOf(handle));
        bodyCount--;
    }

    /**
     * 推进物理仿真一步。
     * @param dt 时间步长（秒）；通常为固定步长，如 1/60
     */
    public void step(float dt) {
        if (dt <= 0.0f) return;

        broadphasePairs.clear();
        contacts.clear();
        manifolds.clear();
        nextCache.clear();

        // 注意：外部调用者应在每次 step() 前通过 applyForce / applyCentralForce / applyTorque
        // 累积本帧想要施加的力；step() 结束时 clearAccumulators() 会将其清零。
        integrateForces(dt);
        integrateVelocities(dt);
        updateAllDerivedBodyState();
        buildBroadphasePairs();
        generateContacts(dt);
        warmStartContacts();

        for (int i = 0; i < VELOCITY_ITERATIONS; i++) solveVelocityConstraints();
        for (int i = 0; i < POSITION_ITERATIONS;  i++) solvePositionConstraints();

        updateAllDerivedBodyState();
        updateContactCache();
        updateSleepState();
        clearAccumulators();
        prevStepDt = dt;
    }

    /**
     * 读取刚体当前状态。
     * @param handle 刚体句柄
     * @return 当前状态；若句柄无效则返回 null
     */
    public BodyState getBodyState(int handle) {
        if (!isValidHandle(handle)) return null;
        BodyState s = new BodyState();

        float[] pos = position.get(handle);
        float[] rot = rotation.get(handle);
        float[] com = comOffsetLocal.get(handle);
        float[] tPos = bodyPositionToWorldTransform(pos, rot, com);
        s.position          = new Vec3(tPos[0], tPos[1], tPos[2]);
        s.rotation          = new Quat(rot[0], rot[1], rot[2], rot[3]);

        float[] lv = linearVelocity.get(handle);
        float[] av = angularVelocity.get(handle);
        s.linearVelocity    = new Vec3(lv[0], lv[1], lv[2]);
        s.angularVelocity   = new Vec3(av[0], av[1], av[2]);
        s.sleeping          = sleeping.get(handle)[0];
        return s;
    }

    /**
     * 传送一个刚体到指定姿态（position 为 Transform 原点，非质心）。
     */
    public void setBodyTransform(int handle, Vec3 worldPosition, Quat worldRotation) {
        if (!isValidHandle(handle)) return;
        float[] rot = new float[]{ worldRotation.x, worldRotation.y, worldRotation.z, worldRotation.w };
        normalizeQuatInPlace(rot);
        float[] com = comOffsetLocal.get(handle);
        float[] comWorld = worldTransformToBodyPosition(
                new float[]{ worldPosition.x, worldPosition.y, worldPosition.z }, rot, com);

        float[] pos = position.get(handle);
        pos[0] = comWorld[0]; pos[1] = comWorld[1]; pos[2] = comWorld[2];
        float[] r = rotation.get(handle);
        r[0] = rot[0]; r[1] = rot[1]; r[2] = rot[2]; r[3] = rot[3];
        updateDerivedBodyState(handle);
    }

    /**
     * 直接设置刚体速度。
     */
    public void setBodyVelocity(int handle, Vec3 linearVel, Vec3 angularVel) {
        if (!isValidHandle(handle)) return;
        float[] lv = linearVelocity.get(handle);
        lv[0] = linearVel.x; lv[1] = linearVel.y; lv[2] = linearVel.z;
        float[] av = angularVelocity.get(handle);
        av[0] = angularVel.x; av[1] = angularVel.y; av[2] = angularVel.z;
    }

    /**
     * 在世界空间某点对刚体施加一个力（在下一个 step() 中生效）。
     * @param force      世界空间力向量 (N)
     * @param worldPoint 施力点世界坐标
     */
    public void applyForce(int handle, Vec3 force, Vec3 worldPoint) {
        if (!isValidHandle(handle)) return;
        float[] fa = forceAcc.get(handle);
        fa[0] += force.x; fa[1] += force.y; fa[2] += force.z;
        float[] pos = position.get(handle);
        float rx = worldPoint.x - pos[0];
        float ry = worldPoint.y - pos[1];
        float rz = worldPoint.z - pos[2];
        float[] ta = torqueAcc.get(handle);
        ta[0] += ry * force.z - rz * force.y;
        ta[1] += rz * force.x - rx * force.z;
        ta[2] += rx * force.y - ry * force.x;
    }

    /**
     * 对刚体施加一个中心力（通过质心，不产生力矩）。
     */
    public void applyCentralForce(int handle, Vec3 force) {
        if (!isValidHandle(handle)) return;
        float[] fa = forceAcc.get(handle);
        fa[0] += force.x; fa[1] += force.y; fa[2] += force.z;
    }

    /**
     * 施加一个力矩（torque）。
     */
    public void applyTorque(int handle, Vec3 torque) {
        if (!isValidHandle(handle)) return;
        float[] ta = torqueAcc.get(handle);
        ta[0] += torque.x; ta[1] += torque.y; ta[2] += torque.z;
    }

    /**
     * 唤醒一个休眠的 Dynamic 刚体。
     */
    public void wakeBody(int handle) {
        if (!isValidHandle(handle)) return;
        if (motionType.get(handle) == MotionType.DYNAMIC) {
            sleeping.get(handle)[0] = false;
            sleepCounter.get(handle)[0] = 0;
        }
    }

    /** 获取活跃刚体总数 */
    public int getBodyCount() { return bodyCount; }

    /** 获取休眠刚体数量 */
    public int getSleepingCount() {
        int cnt = 0;
        for (int h : activeHandles) if (sleeping.get(h)[0]) cnt++;
        return cnt;
    }

    /** 获取上一步生成的接触点数量 */
    public int getContactCount() { return contacts.size(); }

    /** 获取上一步的宽相碰撞对数量 */
    public int getBroadphasePairCount() { return broadphasePairs.size(); }

    /** 获取上一步所有接触点的调试信息 */
    public List<ContactInfo> getContacts() {
        List<ContactInfo> result = new ArrayList<>(contacts.size());
        for (ContactData c : contacts) {
            ContactInfo ci = new ContactInfo();
            ci.point       = new Vec3(c.point[0],  c.point[1],  c.point[2]);
            ci.normal      = new Vec3(c.normal[0], c.normal[1], c.normal[2]);
            ci.penetration = c.penetration;
            result.add(ci);
        }
        return result;
    }

    /** 获取所有活跃刚体的调试几何信息（AABB、质心等） */
    public List<BodyDebugInfo> getBodyDebugInfos() {
        List<BodyDebugInfo> result = new ArrayList<>(activeHandles.size());
        for (int h : activeHandles) {
            if (!solverEnabled.get(h)[0]) continue;
            BodyDebugInfo info = new BodyDebugInfo();
            info.handle = h;
            float[] pos = position.get(h);
            float[] rot = rotation.get(h);
            float[] com = comOffsetLocal.get(h);
            float[] tPos = bodyPositionToWorldTransform(pos, rot, com);
            info.comWorld        = new Vec3(pos[0], pos[1], pos[2]);
            info.transformOrigin = new Vec3(tPos[0], tPos[1], tPos[2]);
            float[] cc = colliderCenterWorld.get(h);
            info.colliderCenter  = new Vec3(cc[0], cc[1], cc[2]);
            float[] amn = aabbMin.get(h);
            float[] amx = aabbMax.get(h);
            info.aabbMin = new Vec3(amn[0], amn[1], amn[2]);
            info.aabbMax = new Vec3(amx[0], amx[1], amx[2]);
            result.add(info);
        }
        return result;
    }

    // ========================================================================
    // 私有：刚体初始化
    // ========================================================================

    private void initBodyFromDesc(int h, BodyDesc d) {
        motionType.set(h, d.motionType);

        float[] sc = scale.get(h);
        sc[0] = d.scale.x; sc[1] = d.scale.y; sc[2] = d.scale.z;

        float[] com = comOffsetLocal.get(h);
        com[0] = d.centerOfMassOffset.x; com[1] = d.centerOfMassOffset.y; com[2] = d.centerOfMassOffset.z;

        float[] clo = colliderLocalOffset.get(h);
        clo[0] = d.colliderLocalOffset.x; clo[1] = d.colliderLocalOffset.y; clo[2] = d.colliderLocalOffset.z;

        float[] clr = colliderLocalRot.get(h);
        clr[0] = d.colliderLocalRotation.x; clr[1] = d.colliderLocalRotation.y;
        clr[2] = d.colliderLocalRotation.z; clr[3] = d.colliderLocalRotation.w;

        // Box half extents（Sphere/Capsule 近似为 box）
        float[] bhe = boxHalfExtLocal.get(h);
        boolean enabled = true;
        switch (d.shapeType) {
            case BOX:
                bhe[0] = d.boxHalfExtents.x; bhe[1] = d.boxHalfExtents.y; bhe[2] = d.boxHalfExtents.z;
                break;
            case SPHERE:
                bhe[0] = d.sphereRadius; bhe[1] = d.sphereRadius; bhe[2] = d.sphereRadius;
                break;
            case CAPSULE:
                bhe[0] = d.capsuleRadius; bhe[1] = d.capsuleRadius + d.capsuleHalfHeight; bhe[2] = d.capsuleRadius;
                break;
            default:
                enabled = false;
                break;
        }
        solverEnabled.get(h)[0] = enabled;

        // 材质
        friction.get(h)[0]       = d.friction;
        linearDamping.get(h)[0]  = d.linearDamping;
        angularDamping.get(h)[0] = d.angularDamping;
        useGravity.get(h)[0]     = d.useGravity;

        // 质量（仅 Dynamic 有效）
        float effectiveMass = (d.motionType == MotionType.DYNAMIC) ? d.mass : 0.0f;
        if (effectiveMass <= EPS) effectiveMass = 0.0f;
        mass.get(h)[0]    = effectiveMass;
        invMass.get(h)[0] = (effectiveMass > EPS) ? 1.0f / effectiveMass : 0.0f;

        // 世界空间 half extents（应用缩放）
        float[] whe = boxHalfExtWorld.get(h);
        whe[0] = Math.abs(bhe[0] * sc[0]);
        whe[1] = Math.abs(bhe[1] * sc[1]);
        whe[2] = Math.abs(bhe[2] * sc[2]);

        // 惯性张量
        computeBoxInvInertiaDiag(effectiveMass, whe, invInertiaLocal.get(h));

        // 初始变换（将 Transform 原点转为 COM 世界坐标）
        float[] rot = rotation.get(h);
        rot[0] = d.rotation.x; rot[1] = d.rotation.y; rot[2] = d.rotation.z; rot[3] = d.rotation.w;
        normalizeQuatInPlace(rot);

        float[] descPos = { d.position.x, d.position.y, d.position.z };
        float[] comWorld = worldTransformToBodyPosition(descPos, rot, com);
        float[] pos = position.get(h);
        pos[0] = comWorld[0]; pos[1] = comWorld[1]; pos[2] = comWorld[2];
        System.arraycopy(pos, 0, prevPosition.get(h), 0, 3);

        // 初始速度
        float[] lv = linearVelocity.get(h);
        lv[0] = d.linearVelocity.x; lv[1] = d.linearVelocity.y; lv[2] = d.linearVelocity.z;
        float[] av = angularVelocity.get(h);
        av[0] = d.angularVelocity.x; av[1] = d.angularVelocity.y; av[2] = d.angularVelocity.z;

        // 休眠
        sleeping.get(h)[0]     = d.sleeping;
        sleepCounter.get(h)[0] = 0;
    }

    private boolean isValidHandle(int h) {
        return h >= 0 && h < alive.size() && alive.get(h);
    }

    // ========================================================================
    // 数学工具函数（静态）
    // ========================================================================

    private static float dot(float[] a, float[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static float[] cross(float[] a, float[] b) {
        return new float[]{
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    private static float lengthSq(float[] v) { return dot(v, v); }
    private static float length(float[] v)   { return (float) Math.sqrt(lengthSq(v)); }

    private static float[] normalizeVec3(float[] v) {
        float len = length(v);
        if (len <= EPS) return new float[]{ 0, 0, 0 };
        return new float[]{ v[0] / len, v[1] / len, v[2] / len };
    }

    private static void normalizeQuatInPlace(float[] q) {
        float len = (float) Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (len <= EPS) { q[0] = 0; q[1] = 0; q[2] = 0; q[3] = 1; return; }
        float inv = 1.0f / len;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    }

    /**
     * 用四元数旋转向量 v。
     * 使用高效公式：t = 2*(q_vec × v); v' = v + qw*t + q_vec × t
     */
    private static float[] rotateByQuat(float[] q, float[] v) {
        float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
        float vx = v[0], vy = v[1], vz = v[2];
        float tx = 2.0f * (qy * vz - qz * vy);
        float ty = 2.0f * (qz * vx - qx * vz);
        float tz = 2.0f * (qx * vy - qy * vx);
        return new float[]{
            vx + qw * tx + qy * tz - qz * ty,
            vy + qw * ty + qz * tx - qx * tz,
            vz + qw * tz + qx * ty - qy * tx
        };
    }

    /** 四元数乘法（Hamilton 积）：a * b */
    private static float[] quatMul(float[] a, float[] b) {
        return new float[]{
            a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
            a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
            a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
            a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2]
        };
    }

    private static float[] add3(float[] a, float[] b) {
        return new float[]{ a[0]+b[0], a[1]+b[1], a[2]+b[2] };
    }
    private static float[] sub3(float[] a, float[] b) {
        return new float[]{ a[0]-b[0], a[1]-b[1], a[2]-b[2] };
    }
    private static float[] scale3(float[] v, float s) {
        return new float[]{ v[0]*s, v[1]*s, v[2]*s };
    }
    private static void addInPlace(float[] dst, float[] src) {
        dst[0] += src[0]; dst[1] += src[1]; dst[2] += src[2];
    }
    private static void subInPlace(float[] dst, float[] src) {
        dst[0] -= src[0]; dst[1] -= src[1]; dst[2] -= src[2];
    }
    private static void scaleInPlace(float[] dst, float s) {
        dst[0] *= s; dst[1] *= s; dst[2] *= s;
    }

    private static float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(v, hi));
    }

    /** 一阶欧拉旋转积分：q_new = normalize(q + 0.5 * omega * q * dt) */
    private static float[] integrateRotationEuler(float[] q, float[] angVel, float dt) {
        // omega 作为纯四元数: {wx, wy, wz, 0}
        float[] omega = { angVel[0], angVel[1], angVel[2], 0 };
        float[] dq = quatMul(omega, q);
        float[] out = {
            q[0] + 0.5f * dq[0] * dt,
            q[1] + 0.5f * dq[1] * dt,
            q[2] + 0.5f * dq[2] * dt,
            q[3] + 0.5f * dq[3] * dt
        };
        normalizeQuatInPlace(out);
        return out;
    }

    /** 将增量角度应用到四元数（限幅防止过大跳变） */
    private static float[] applyRotationDelta(float[] q, float[] deltaAngle) {
        float lenSq = lengthSq(deltaAngle);
        if (lenSq <= EPS * EPS) return q.clone();
        float len = (float) Math.sqrt(lenSq);
        float[] da = deltaAngle.clone();
        if (len > MAX_ANGULAR_POSITION_STEP) {
            float s = MAX_ANGULAR_POSITION_STEP / len;
            da[0] *= s; da[1] *= s; da[2] *= s;
        }
        return integrateRotationEuler(q, da, 1.0f);
    }

    // ========================================================================
    // 物理辅助函数
    // ========================================================================

    /** 计算长方体逆惯性张量对角线（主轴坐标系） */
    private static void computeBoxInvInertiaDiag(float m, float[] he, float[] out) {
        if (m <= EPS) { out[0] = 0; out[1] = 0; out[2] = 0; return; }
        float sx = 2 * he[0], sy = 2 * he[1], sz = 2 * he[2];
        float ix = (m / 12.0f) * (sy*sy + sz*sz);
        float iy = (m / 12.0f) * (sx*sx + sz*sz);
        float iz = (m / 12.0f) * (sx*sx + sy*sy);
        out[0] = (ix > EPS) ? 1.0f / ix : 0.0f;
        out[1] = (iy > EPS) ? 1.0f / iy : 0.0f;
        out[2] = (iz > EPS) ? 1.0f / iz : 0.0f;
    }

    /** Transform 原点 → COM 世界坐标：p_com = p_transform + rotate(rot, com_offset) */
    private static float[] worldTransformToBodyPosition(float[] tPos, float[] rot, float[] comOffset) {
        return add3(tPos, rotateByQuat(rot, comOffset));
    }

    /** COM 世界坐标 → Transform 原点：p_transform = p_com - rotate(rot, com_offset) */
    private static float[] bodyPositionToWorldTransform(float[] bodyPos, float[] rot, float[] comOffset) {
        return sub3(bodyPos, rotateByQuat(rot, comOffset));
    }

    /** 将向量 value 乘以刚体 h 的世界空间逆惯性张量 */
    private float[] applyInvInertiaWorld(int h, float[] value) {
        float[] rot = rotation.get(h);
        float[] clr = colliderLocalRot.get(h);
        float[] inertiaRot = quatMul(rot, clr);
        float[] axX = rotateByQuat(inertiaRot, new float[]{ 1, 0, 0 });
        float[] axY = rotateByQuat(inertiaRot, new float[]{ 0, 1, 0 });
        float[] axZ = rotateByQuat(inertiaRot, new float[]{ 0, 0, 1 });
        float[] invI = invInertiaLocal.get(h);
        float dx = invI[0] * dot(value, axX);
        float dy = invI[1] * dot(value, axY);
        float dz = invI[2] * dot(value, axZ);
        return new float[]{
            axX[0]*dx + axY[0]*dy + axZ[0]*dz,
            axX[1]*dx + axY[1]*dy + axZ[1]*dz,
            axX[2]*dx + axY[2]*dy + axZ[2]*dz
        };
    }

    /** 计算刚体 h 在世界空间点 r（相对于 COM）处的点速度 */
    private float[] computePointVelocity(int h, float[] r) {
        return add3(linearVelocity.get(h), cross(angularVelocity.get(h), r));
    }

    /**
     * 计算某约束轴的等效质量（1 自由度）。
     * 同一公式可复用于法线约束、摩擦约束，未来还可复用于关节约束。
     */
    private float computeConstraintEffMass(int a, int b, float[] r1, float[] r2, float[] axis) {
        float k = invMass.get(a)[0] + invMass.get(b)[0];
        if (invMass.get(a)[0] > 0) {
            float[] tmp = applyInvInertiaWorld(a, cross(r1, axis));
            k += dot(cross(tmp, r1), axis);
        }
        if (invMass.get(b)[0] > 0) {
            float[] tmp = applyInvInertiaWorld(b, cross(r2, axis));
            k += dot(cross(tmp, r2), axis);
        }
        return (k > EPS) ? 1.0f / k : 0.0f;
    }

    /** 对 a、b 两体施加速度冲量对 */
    private void applyImpulsePair(int a, int b, float[] impulse, float[] r1, float[] r2) {
        float im_a = invMass.get(a)[0], im_b = invMass.get(b)[0];
        if (im_a > 0) {
            subInPlace(linearVelocity.get(a),  scale3(impulse, im_a));
            subInPlace(angularVelocity.get(a), applyInvInertiaWorld(a, cross(r1, impulse)));
        }
        if (im_b > 0) {
            addInPlace(linearVelocity.get(b),  scale3(impulse, im_b));
            addInPlace(angularVelocity.get(b), applyInvInertiaWorld(b, cross(r2, impulse)));
        }
    }

    /** 对 a、b 两体施加位置修正冲量对（影响 position/rotation，不影响速度） */
    private void applyPositionImpulsePair(int a, int b, float[] impulse, float[] r1, float[] r2) {
        float im_a = invMass.get(a)[0], im_b = invMass.get(b)[0];
        if (im_a > 0) {
            subInPlace(position.get(a), scale3(impulse, im_a));
            float[] ang = scale3(applyInvInertiaWorld(a, cross(r1, impulse)), -1.0f);
            float[] newRot = applyRotationDelta(rotation.get(a), ang);
            System.arraycopy(newRot, 0, rotation.get(a), 0, 4);
        }
        if (im_b > 0) {
            addInPlace(position.get(b), scale3(impulse, im_b));
            float[] ang = applyInvInertiaWorld(b, cross(r2, impulse));
            float[] newRot = applyRotationDelta(rotation.get(b), ang);
            System.arraycopy(newRot, 0, rotation.get(b), 0, 4);
        }
    }

    // ========================================================================
    // OBB Proxy 与 AABB 维护
    // ========================================================================

    // OBB proxy float[15] 布局: center(0-2), axis0(3-5), axis1(6-8), axis2(9-11), halfExtents(12-14)
    private static float[] obbCenter(float[] obb) { return new float[]{ obb[0], obb[1], obb[2] }; }
    private static float[] obbAxis(float[] obb, int i) {
        return new float[]{ obb[3+i*3], obb[3+i*3+1], obb[3+i*3+2] };
    }
    private static float obbHeAt(float[] obb, int i) { return obb[12 + i]; }

    private static void buildObbProxy(float[] obb, float[] center, float[] q, float[] he) {
        obb[0] = center[0]; obb[1] = center[1]; obb[2] = center[2];
        float[] ax0 = normalizeVec3(rotateByQuat(q, new float[]{ 1, 0, 0 }));
        float[] ax1 = normalizeVec3(rotateByQuat(q, new float[]{ 0, 1, 0 }));
        float[] ax2 = normalizeVec3(rotateByQuat(q, new float[]{ 0, 0, 1 }));
        obb[3]  = ax0[0]; obb[4]  = ax0[1]; obb[5]  = ax0[2];
        obb[6]  = ax1[0]; obb[7]  = ax1[1]; obb[8]  = ax1[2];
        obb[9]  = ax2[0]; obb[10] = ax2[1]; obb[11] = ax2[2];
        obb[12] = he[0];  obb[13] = he[1];  obb[14] = he[2];
    }

    private void updateDerivedBodyState(int h) {
        float[] pos = position.get(h);
        float[] rot = rotation.get(h);
        float[] com = comOffsetLocal.get(h);
        float[] sc  = scale.get(h);
        float[] clo = colliderLocalOffset.get(h);
        float[] clr = colliderLocalRot.get(h);
        float[] bhe = boxHalfExtLocal.get(h);

        float[] tOrigin = bodyPositionToWorldTransform(pos, rot, com);
        float[] wlo = rotateByQuat(rot, clo);
        float[] cc = colliderCenterWorld.get(h);
        cc[0] = tOrigin[0] + wlo[0]; cc[1] = tOrigin[1] + wlo[1]; cc[2] = tOrigin[2] + wlo[2];

        float[] whe = boxHalfExtWorld.get(h);
        whe[0] = Math.abs(bhe[0] * sc[0]);
        whe[1] = Math.abs(bhe[1] * sc[1]);
        whe[2] = Math.abs(bhe[2] * sc[2]);

        float[] collWorldRot = quatMul(rot, clr);
        buildObbProxy(obbProxy.get(h), cc, collWorldRot, whe);

        // AABB from OBB
        float[] obb = obbProxy.get(h);
        float hx = 0, hy = 0, hz = 0;
        for (int i = 0; i < 3; i++) {
            float[] ax = obbAxis(obb, i);
            float   he = obbHeAt(obb, i);
            hx += Math.abs(ax[0]) * he;
            hy += Math.abs(ax[1]) * he;
            hz += Math.abs(ax[2]) * he;
        }
        float[] amn = aabbMin.get(h); float[] amx = aabbMax.get(h);
        amn[0] = cc[0]-hx; amn[1] = cc[1]-hy; amn[2] = cc[2]-hz;
        amx[0] = cc[0]+hx; amx[1] = cc[1]+hy; amx[2] = cc[2]+hz;
    }

    private void updateAllDerivedBodyState() {
        for (int h : activeHandles) updateDerivedBodyState(h);
    }

    // ========================================================================
    // 物理步骤
    // ========================================================================

    private boolean isTrulyDynamic(int h) {
        return motionType.get(h) == MotionType.DYNAMIC && invMass.get(h)[0] > 0;
    }

    private void integrateForces(float dt) {
        for (int h : activeHandles) {
            if (!isTrulyDynamic(h)) continue;
            if (sleeping.get(h)[0]) continue;

            float[] g = useGravity.get(h)[0]
                ? new float[]{ config.gravity.x, config.gravity.y, config.gravity.z }
                : new float[3];
            float im = invMass.get(h)[0];
            float[] fa = forceAcc.get(h);
            float[] lv = linearVelocity.get(h);
            lv[0] += (g[0] + fa[0] * im) * dt;
            lv[1] += (g[1] + fa[1] * im) * dt;
            lv[2] += (g[2] + fa[2] * im) * dt;

            float[] angAcc = applyInvInertiaWorld(h, torqueAcc.get(h));
            float[] av = angularVelocity.get(h);
            av[0] += angAcc[0] * dt; av[1] += angAcc[1] * dt; av[2] += angAcc[2] * dt;

            float ld = clamp(1.0f - linearDamping.get(h)[0]  * dt, 0, 1);
            float ad = clamp(1.0f - angularDamping.get(h)[0] * dt, 0, 1);
            scaleInPlace(lv, ld);
            scaleInPlace(av, ad);
        }
    }

    private void integrateVelocities(float dt) {
        for (int h : activeHandles) {
            System.arraycopy(position.get(h), 0, prevPosition.get(h), 0, 3);
            if (!isTrulyDynamic(h)) continue;
            if (sleeping.get(h)[0]) continue;

            float[] pos = position.get(h);
            float[] lv  = linearVelocity.get(h);
            pos[0] += lv[0] * dt; pos[1] += lv[1] * dt; pos[2] += lv[2] * dt;

            float[] av = angularVelocity.get(h);
            if (lengthSq(av) > EPS * EPS) {
                float[] newRot = integrateRotationEuler(rotation.get(h), av, dt);
                System.arraycopy(newRot, 0, rotation.get(h), 0, 4);
            }
        }
    }

    private boolean aabbOverlap(int a, int b) {
        float[] aminA = aabbMin.get(a), amaxA = aabbMax.get(a);
        float[] aminB = aabbMin.get(b), amaxB = aabbMax.get(b);
        return !(amaxA[0] < aminB[0] || aminA[0] > amaxB[0]
              || amaxA[1] < aminB[1] || aminA[1] > amaxB[1]
              || amaxA[2] < aminB[2] || aminA[2] > amaxB[2]);
    }

    private void buildBroadphasePairs() {
        int n = activeHandles.size();
        for (int i = 0; i < n; i++) {
            int a = activeHandles.get(i);
            if (!solverEnabled.get(a)[0]) continue;
            for (int j = i + 1; j < n; j++) {
                int b = activeHandles.get(j);
                if (!solverEnabled.get(b)[0]) continue;
                boolean aDyn = isTrulyDynamic(a);
                boolean bDyn = isTrulyDynamic(b);
                if (!aDyn && !bDyn) continue;
                if (aDyn && sleeping.get(a)[0] && bDyn && sleeping.get(b)[0]) continue;
                if (!aabbOverlap(a, b)) continue;
                broadphasePairs.add(new int[]{ a, b });
            }
        }
    }

    // ========================================================================
    // SAT 碰撞检测（OBB vs OBB）
    // ========================================================================

    /**
     * OBB-OBB SAT 检测（15 轴：3 个 A 面法线 + 3 个 B 面法线 + 9 个边叉积轴）。
     * preferredNormal 为上一帧缓存的法线，用于轻量级 SAT 迟滞抑制轴抖动。
     */
    private SatResult boxBoxSat(float[] obbA, float[] obbB, float[] preferredNormal) {
        SatResult result = new SatResult();
        result.separated  = false;
        result.penetration = Float.MAX_VALUE;
        result.normal      = new float[]{ 0, 1, 0 };
        result.axisIndex   = -1;
        result.edgeA = result.edgeB = -1;
        float cx = (obbA[0] + obbB[0]) * 0.5f;
        float cy = (obbA[1] + obbB[1]) * 0.5f;
        float cz = (obbA[2] + obbB[2]) * 0.5f;
        result.point = new float[]{ cx, cy, cz };

        float[] d = sub3(obbCenter(obbB), obbCenter(obbA));
        float bestMetric = Float.MAX_VALUE;

        for (int pass = 0; pass < 15; pass++) {
            float[] axis;
            int axisIndex, edgeA = -1, edgeB = -1;

            if (pass < 3) {
                axis = obbAxis(obbA, pass);      axisIndex = pass;
            } else if (pass < 6) {
                axis = obbAxis(obbB, pass - 3);  axisIndex = pass;
            } else {
                int i = (pass - 6) / 3;
                int j = (pass - 6) % 3;
                axis = cross(obbAxis(obbA, i), obbAxis(obbB, j));
                axisIndex = pass; edgeA = i; edgeB = j;
            }

            float axLenSq = lengthSq(axis);
            if (axLenSq < EPS * EPS) continue;
            axis = normalizeVec3(axis);
            if (dot(axis, d) < 0) { axis[0] = -axis[0]; axis[1] = -axis[1]; axis[2] = -axis[2]; }

            float ra = 0, rb = 0;
            for (int k = 0; k < 3; k++) {
                ra += Math.abs(dot(obbAxis(obbA, k), axis)) * obbHeAt(obbA, k);
                rb += Math.abs(dot(obbAxis(obbB, k), axis)) * obbHeAt(obbB, k);
            }
            float overlap = ra + rb - Math.abs(dot(d, axis));
            if (overlap < 0) { result.separated = true; return result; }

            float selMetric = overlap;
            if (axisIndex >= 6) selMetric += SAT_EDGE_AXIS_BIAS;
            if (preferredNormal != null) {
                float align = dot(axis, preferredNormal);
                if (align > SAT_PREFERRED_NORMAL_DOT) {
                    float t = clamp((align - SAT_PREFERRED_NORMAL_DOT) / (1.0f - SAT_PREFERRED_NORMAL_DOT), 0, 1);
                    selMetric -= SAT_PREFERRED_NORMAL_BIAS * t;
                }
            }

            if (selMetric < bestMetric) {
                bestMetric        = selMetric;
                result.penetration = overlap;
                result.normal      = axis.clone();
                result.axisIndex   = axisIndex;
                result.edgeA       = edgeA;
                result.edgeB       = edgeB;
            }
        }
        return result;
    }

    /** 重新测量给定法线方向上的穿透深度（用于位置修正时重新取样） */
    private static float remeasurePenetration(float[] obbA, float[] obbB, float[] normal) {
        float ra = 0, rb = 0;
        for (int i = 0; i < 3; i++) {
            ra += Math.abs(dot(obbAxis(obbA, i), normal)) * obbHeAt(obbA, i);
            rb += Math.abs(dot(obbAxis(obbB, i), normal)) * obbHeAt(obbB, i);
        }
        float dist = Math.abs(dot(sub3(obbCenter(obbB), obbCenter(obbA)), normal));
        return ra + rb - dist;
    }

    /** 在给定方向上的 OBB 支撑点 */
    private static float[] obbSupport(float[] obb, float[] dir) {
        float[] res = obbCenter(obb).clone();
        for (int i = 0; i < 3; i++) {
            float[] ax = obbAxis(obb, i);
            float sign = (dot(dir, ax) >= 0) ? 1.0f : -1.0f;
            float he = obbHeAt(obb, i);
            res[0] += ax[0] * sign * he;
            res[1] += ax[1] * sign * he;
            res[2] += ax[2] * sign * he;
        }
        return res;
    }

    // ========================================================================
    // 面接触流形裁切
    // ========================================================================

    /**
     * 构造 OBB 某面的四个顶点（CCW 方向，从外侧看）。
     * outCN[0] = faceCenter, outCN[1] = faceNormal
     * outVertices[0..3] = 四顶点
     */
    private static void buildFaceVertices(float[] obb, int faceAxis, float faceSign,
                                           float[][] outCN, float[][] outVerts) {
        float[] axFace = obbAxis(obb, faceAxis);
        float[] normal = { axFace[0]*faceSign, axFace[1]*faceSign, axFace[2]*faceSign };
        float heFace = obbHeAt(obb, faceAxis);
        float[] center = { obb[0]+normal[0]*heFace, obb[1]+normal[1]*heFace, obb[2]+normal[2]*heFace };

        int uIdx = (faceAxis + 1) % 3, vIdx = (faceAxis + 2) % 3;
        float[] uAx = obbAxis(obb, uIdx), vAx = obbAxis(obb, vIdx);
        float uHe = obbHeAt(obb, uIdx),   vHe = obbHeAt(obb, vIdx);
        float[] u = scale3(uAx, uHe), v = scale3(vAx, vHe);

        outVerts[0] = add3(add3(center, u), v);
        outVerts[1] = add3(sub3(center, u), v);
        outVerts[2] = sub3(sub3(center, u), v);
        outVerts[3] = add3(add3(center, u), scale3(v, -1));

        // 确保 CCW 绕序与法线一致
        float[] winding = cross(sub3(outVerts[1], outVerts[0]), sub3(outVerts[2], outVerts[1]));
        if (dot(winding, normal) < 0) {
            float[] tmp = outVerts[1]; outVerts[1] = outVerts[3]; outVerts[3] = tmp;
        }
        outCN[0] = center;
        outCN[1] = normal;
    }

    /** 从 OBB 中选出与 direction 最对齐的面 */
    private static void buildMostAlignedFace(float[] obb, float[] direction,
                                              float[][] outCN, float[][] outVerts) {
        int bestAxis = 0;
        float bestDot = dot(obbAxis(obb, 0), direction);
        float bestAbsDot = Math.abs(bestDot);
        for (int i = 1; i < 3; i++) {
            float d = dot(obbAxis(obb, i), direction);
            float ad = Math.abs(d);
            if (ad > bestAbsDot) { bestAbsDot = ad; bestDot = d; bestAxis = i; }
        }
        buildFaceVertices(obb, bestAxis, (bestDot >= 0) ? 1.0f : -1.0f, outCN, outVerts);
    }

    /** Sutherland-Hodgman 多边形裁切：保留在平面正侧的顶点 */
    private static List<float[]> clipPolygonAgainstPlane(List<float[]> input,
                                                          float[] planePoint,
                                                          float[] planeNormal) {
        List<float[]> output = new ArrayList<>();
        if (input.isEmpty()) return output;

        float[] a = input.get(input.size() - 1);
        float da = dot(sub3(a, planePoint), planeNormal);
        boolean insideA = (da >= -EPS);

        for (float[] b : input) {
            float db = dot(sub3(b, planePoint), planeNormal);
            boolean insideB = (db >= -EPS);
            if (insideA && insideB) {
                output.add(b);
            } else if (insideA) {
                float t = da / (da - db);
                output.add(add3(a, scale3(sub3(b, a), t)));
            } else if (insideB) {
                float t = da / (da - db);
                output.add(add3(a, scale3(sub3(b, a), t)));
                output.add(b);
            }
            a = b; da = db; insideA = insideB;
        }
        return output;
    }

    private static void deduplicateCandidates(List<ContactCandidate> cands) {
        List<ContactCandidate> unique = new ArrayList<>();
        for (ContactCandidate c : cands) {
            boolean dup = false;
            for (ContactCandidate u : unique) {
                if (lengthSq(sub3(c.point, u.point)) <= DUPLICATE_POINT_EPS_SQ) { dup = true; break; }
            }
            if (!dup) unique.add(c);
        }
        cands.clear();
        cands.addAll(unique);
    }

    /**
     * 将接触候选点剪枝到最多 4 个，优先保留覆盖面积最大的四边形。
     * 算法：贪心选取最远点组合（深度加权）。
     */
    private static List<ContactCandidate> pruneCandidates(List<ContactCandidate> input, float[] normal) {
        int sz = input.size();
        if (sz <= 4) return new ArrayList<>(input);

        float[][] projected = new float[sz][3];
        float[] centroid = new float[3];
        for (int i = 0; i < sz; i++) {
            float along = dot(input.get(i).point, normal);
            float[] p = input.get(i).point;
            projected[i] = new float[]{ p[0]-normal[0]*along, p[1]-normal[1]*along, p[2]-normal[2]*along };
            centroid[0] += projected[i][0]; centroid[1] += projected[i][1]; centroid[2] += projected[i][2];
        }
        centroid[0] /= sz; centroid[1] /= sz; centroid[2] /= sz;

        // 1. 距质心最远
        int first = 0; float firstScore = -1;
        for (int i = 0; i < sz; i++) {
            float s = lengthSq(sub3(projected[i], centroid))
                    * Math.max(input.get(i).penetration * input.get(i).penetration, 1.0f);
            if (s > firstScore) { firstScore = s; first = i; }
        }
        List<Integer> chosen = new ArrayList<>();
        chosen.add(first);

        // 2. 距第一点最远
        int second = first; float secondScore = -1;
        for (int i = 0; i < sz; i++) {
            float s = lengthSq(sub3(projected[i], projected[first]))
                    * Math.max(input.get(i).penetration * input.get(i).penetration, 1.0f);
            if (s > secondScore) { secondScore = s; second = i; }
        }
        if (second != first) chosen.add(second);

        float[] edgeDir = sub3(projected[second], projected[first]);
        if (lengthSq(edgeDir) <= EPS * EPS) edgeDir = new float[]{ 1, 0, 0 };
        else edgeDir = normalizeVec3(edgeDir);
        float[] perp = normalizeVec3(cross(normal, edgeDir));

        // 3. 垂直方向两侧最远点
        int posIdx = -1, negIdx = -1;
        float posD = -1, negD = -1;
        for (int i = 0; i < sz; i++) {
            if (i == first || i == second) continue;
            float sd = dot(sub3(projected[i], projected[first]), perp);
            float w  = Math.abs(sd) * Math.max(input.get(i).penetration, 1.0f);
            if (sd >= 0 && w > posD) { posD = w; posIdx = i; }
            if (sd <= 0 && w > negD) { negD = w; negIdx = i; }
        }
        if (posIdx >= 0) chosen.add(posIdx);
        if (negIdx >= 0 && negIdx != posIdx) chosen.add(negIdx);

        List<ContactCandidate> output = new ArrayList<>();
        for (int idx : chosen) {
            ContactCandidate c = input.get(idx);
            boolean dup = false;
            for (ContactCandidate e : output) {
                if (lengthSq(sub3(c.point, e.point)) <= DUPLICATE_POINT_EPS_SQ) { dup = true; break; }
            }
            if (!dup) output.add(c);
        }
        // 补满 4 个
        outer:
        while (output.size() < 4) {
            for (ContactCandidate c : input) {
                boolean dup = false;
                for (ContactCandidate e : output) {
                    if (lengthSq(sub3(c.point, e.point)) <= DUPLICATE_POINT_EPS_SQ) { dup = true; break; }
                }
                if (!dup) {
                    output.add(c);
                    if (output.size() >= 4) break outer;
                }
            }
            break;
        }
        return output;
    }

    /** 面-面接触：用 Sutherland-Hodgman 裁切 incident face，投影到 reference plane */
    private static List<ContactCandidate> buildFaceManifoldCandidates(float[] obbA, float[] obbB,
                                                                        SatResult sat) {
        List<ContactCandidate> contacts = new ArrayList<>();
        boolean refIsA = (sat.axisIndex >= 0 && sat.axisIndex < 3);
        float[] refObb  = refIsA ? obbA : obbB;
        float[] incObb  = refIsA ? obbB : obbA;
        int refAxis     = refIsA ? sat.axisIndex : (sat.axisIndex - 3);
        float[] refPN   = refIsA ? sat.normal : scale3(sat.normal, -1);

        float refSign = (dot(refPN, obbAxis(refObb, refAxis)) >= 0) ? 1.0f : -1.0f;

        float[][] refCN   = new float[2][];
        float[][] refVerts = new float[4][];
        buildFaceVertices(refObb, refAxis, refSign, refCN, refVerts);

        float[][] incCN   = new float[2][];
        float[][] incVerts = new float[4][];
        buildMostAlignedFace(incObb, scale3(refPN, -1), incCN, incVerts);

        List<float[]> polygon = new ArrayList<>(Arrays.asList(
                incVerts[0].clone(), incVerts[1].clone(), incVerts[2].clone(), incVerts[3].clone()));

        for (int i = 0; i < 4 && !polygon.isEmpty(); i++) {
            float[] edgeStart = refVerts[i];
            float[] edgeEnd   = refVerts[(i + 1) % 4];
            float[] sidePN    = normalizeVec3(cross(refCN[1], sub3(edgeEnd, edgeStart)));
            polygon = clipPolygonAgainstPlane(polygon, edgeStart, sidePN);
        }

        float[] refCenter = refCN[0], refNormal = refCN[1];
        for (float[] pInc : polygon) {
            float pen = dot(sub3(refCenter, pInc), refNormal);
            if (pen < -PENETRATION_SLOP) continue;
            float[] pRef = add3(pInc, scale3(refNormal, pen));
            ContactCandidate cc = new ContactCandidate();
            cc.point       = scale3(add3(pInc, pRef), 0.5f);
            cc.penetration = Math.max(0, pen);
            contacts.add(cc);
        }

        deduplicateCandidates(contacts);
        if (contacts.size() > 4) contacts = pruneCandidates(contacts, sat.normal);

        if (contacts.isEmpty()) {
            float[] pInc = obbSupport(incObb, scale3(refNormal, -1));
            float pen = dot(sub3(refCenter, pInc), refNormal);
            float[] pRef = add3(pInc, scale3(refNormal, pen));
            ContactCandidate cc = new ContactCandidate();
            cc.point       = scale3(add3(pInc, pRef), 0.5f);
            cc.penetration = Math.max(0, pen);
            contacts.add(cc);
        }
        return contacts;
    }

    /** 边-边接触：计算两段边的最近点 */
    private static ContactCandidate buildEdgeEdgeCandidate(float[] obbA, float[] obbB, SatResult sat) {
        float[] d = sub3(obbCenter(obbB), obbCenter(obbA));
        int ea = sat.edgeA, eb = sat.edgeB;

        float[] pA = obbCenter(obbA).clone();
        for (int k = 0; k < 3; k++) {
            if (k == ea) continue;
            float[] axk = obbAxis(obbA, k);
            float s = (dot(d, axk) > 0) ? obbHeAt(obbA, k) : -obbHeAt(obbA, k);
            pA[0] += axk[0]*s; pA[1] += axk[1]*s; pA[2] += axk[2]*s;
        }
        float[] pB = obbCenter(obbB).clone();
        float[] nd = scale3(d, -1);
        for (int k = 0; k < 3; k++) {
            if (k == eb) continue;
            float[] axk = obbAxis(obbB, k);
            float s = (dot(nd, axk) > 0) ? obbHeAt(obbB, k) : -obbHeAt(obbB, k);
            pB[0] += axk[0]*s; pB[1] += axk[1]*s; pB[2] += axk[2]*s;
        }

        float[] dirA = obbAxis(obbA, ea), dirB = obbAxis(obbB, eb);
        float[] w = sub3(pA, pB);
        float dotAA = dot(dirA, dirA), dotAB = dot(dirA, dirB), dotBB = dot(dirB, dirB);
        float dotWA = dot(w, dirA),    dotWB = dot(w, dirB);
        float denom = dotAA * dotBB - dotAB * dotAB;
        float t = 0, u = 0;
        if (Math.abs(denom) > EPS) {
            t = (dotAB * dotWB - dotBB * dotWA) / denom;
            u = (dotAA * dotWB - dotAB * dotWA) / denom;
        }
        t = clamp(t, -obbHeAt(obbA, ea), obbHeAt(obbA, ea));
        u = clamp(u, -obbHeAt(obbB, eb), obbHeAt(obbB, eb));

        float[] cA = add3(pA, scale3(dirA, t));
        float[] cB = add3(pB, scale3(dirB, u));
        ContactCandidate cc = new ContactCandidate();
        cc.point       = scale3(add3(cA, cB), 0.5f);
        cc.penetration = sat.penetration;
        return cc;
    }

    // ========================================================================
    // 接触点初始化与 Warm Start
    // ========================================================================

    private void initializeContact(ContactData c, float dt) {
        c.r1 = sub3(c.point, position.get(c.a));
        c.r2 = sub3(c.point, position.get(c.b));

        // 切线基
        float[] hint = (Math.abs(c.normal[0]) < 0.57f) ? new float[]{ 1, 0, 0 } : new float[]{ 0, 1, 0 };
        c.tangent1 = normalizeVec3(cross(c.normal, hint));
        if (lengthSq(c.tangent1) <= EPS * EPS)
            c.tangent1 = normalizeVec3(cross(c.normal, new float[]{ 0, 0, 1 }));
        c.tangent2 = normalizeVec3(cross(c.normal, c.tangent1));

        c.combinedFriction = (float) Math.sqrt(
                Math.max(0, friction.get(c.a)[0]) * Math.max(0, friction.get(c.b)[0]));
        c.restitution = 0.0f;

        float[] va = computePointVelocity(c.a, c.r1);
        float[] vb = computePointVelocity(c.b, c.r2);
        float normalSpeed = dot(sub3(vb, va), c.normal);
        boolean nearlyResting = Math.abs(normalSpeed) < RESTING_BIAS_VEL_THRESHOLD
                             && c.penetration < RESTING_BIAS_PENETRATION;
        c.bias = (dt > 0 && !nearlyResting)
               ? (VELOCITY_BAUMGARTE / dt) * Math.max(c.penetration - PENETRATION_SLOP, 0) : 0;

        c.normalEffMass = computeConstraintEffMass(c.a, c.b, c.r1, c.r2, c.normal);
        c.t1EffMass     = computeConstraintEffMass(c.a, c.b, c.r1, c.r2, c.tangent1);
        c.t2EffMass     = computeConstraintEffMass(c.a, c.b, c.r1, c.r2, c.tangent2);
        c.normalLambda  = c.t1Lambda = c.t2Lambda = 0;
    }

    private float computeWarmStartRatio(float dt) {
        if (prevStepDt <= EPS || dt <= EPS) return 1.0f;
        return clamp(dt / prevStepDt, 0.5f, 2.0f);
    }

    private static long makePairKey(int a, int b) {
        long lo = Math.min(a, b), hi = Math.max(a, b);
        return (hi << 32L) | (lo & 0xFFFFFFFFL);
    }

    /** 尝试从上一帧缓存中为接触点 c 注入 warm-start 冲量 */
    private boolean seedWarmStart(long pairKey, float ratio, boolean[] usedMask, ContactData c) {
        CachedManifold cached = prevCache.get(pairKey);
        if (cached == null || cached.count == 0) return false;
        if (dot(cached.normal, c.normal) < WARM_START_NORMAL_DOT_MIN) return false;

        int best = -1; float bestDist = WARM_START_MATCH_DIST_SQ;
        for (int i = 0; i < cached.count; i++) {
            if (usedMask[i]) continue;
            float dsq = lengthSq(sub3(c.point, cached.points[i].point));
            if (dsq <= bestDist) { bestDist = dsq; best = i; }
        }
        if (best < 0) return false;

        usedMask[best]  = true;
        c.normalLambda  = Math.max(0, cached.points[best].normalLambda * ratio);
        // 仅热启动法线冲量，切线方向归零。
        // 与 C++ v4 设计一致：法线热启动修复了金字塔坍塌问题，
        // 同时避免帧间切线方向抖动导致的不稳定。
        c.t1Lambda      = 0;
        c.t2Lambda      = 0;

        float maxFriction = c.combinedFriction * c.normalLambda;
        float tLenSq = c.t1Lambda * c.t1Lambda + c.t2Lambda * c.t2Lambda;
        if (tLenSq > maxFriction * maxFriction && tLenSq > EPS * EPS) {
            float s = maxFriction / (float) Math.sqrt(tLenSq);
            c.t1Lambda *= s; c.t2Lambda *= s;
        }
        return true;
    }

    private void warmStartContacts() {
        for (ContactData c : contacts) {
            float[] impulse = add3(add3(scale3(c.normal, c.normalLambda),
                                        scale3(c.tangent1, c.t1Lambda)),
                                        scale3(c.tangent2, c.t2Lambda));
            if (lengthSq(impulse) <= 0) continue;
            applyImpulsePair(c.a, c.b, impulse, c.r1, c.r2);
        }
    }

    private boolean shouldWakePair(int a, int b, float[] point, float[] normal,
                                   float penetration, boolean hadCache) {
        boolean aSleepDyn = isTrulyDynamic(a) && sleeping.get(a)[0];
        boolean bSleepDyn = isTrulyDynamic(b) && sleeping.get(b)[0];
        if (!aSleepDyn && !bSleepDyn) return false;

        float[] r1 = sub3(point, position.get(a));
        float[] r2 = sub3(point, position.get(b));
        float[] rv = sub3(computePointVelocity(b, r2), computePointVelocity(a, r1));
        float normalSpeed  = Math.abs(dot(rv, normal));
        float[] tv         = sub3(rv, scale3(normal, dot(rv, normal)));
        float tangentSpeed = (float) Math.sqrt(Math.max(0, lengthSq(tv)));

        if (!hadCache) {
            return penetration > WAKE_PENETRATION_THRESHOLD
                || normalSpeed  > WAKE_NORMAL_SPEED_THRESHOLD
                || tangentSpeed > WAKE_TANGENT_SPEED_THRESHOLD;
        }
        return normalSpeed  > 2 * WAKE_NORMAL_SPEED_THRESHOLD
            || tangentSpeed > 2 * WAKE_TANGENT_SPEED_THRESHOLD;
    }

    private void generateContacts(float dt) {
        float warmRatio = computeWarmStartRatio(dt);
        for (int[] pair : broadphasePairs) {
            int a = pair[0], b = pair[1];
            float[] obbA = obbProxy.get(a), obbB = obbProxy.get(b);

            long    pairKey    = makePairKey(a, b);
            float[] prefNormal = null;
            CachedManifold prev = prevCache.get(pairKey);
            if (prev != null && lengthSq(prev.normal) > EPS * EPS) prefNormal = prev.normal;

            SatResult sat = boxBoxSat(obbA, obbB, prefNormal);
            if (sat.separated) continue;

            List<ContactCandidate> candidates;
            if (sat.axisIndex >= 0 && sat.axisIndex < 6) {
                candidates = buildFaceManifoldCandidates(obbA, obbB, sat);
            } else {
                candidates = new ArrayList<>();
                candidates.add(buildEdgeEdgeCandidate(obbA, obbB, sat));
            }
            if (candidates.isEmpty()) continue;

            ManifoldData manifold = new ManifoldData();
            manifold.a = a; manifold.b = b;
            manifold.normal      = sat.normal;
            manifold.penetration = sat.penetration;
            manifold.firstContact = contacts.size();
            manifold.contactCount = 0;
            float[] repSum = new float[3];
            boolean hadCache    = false;
            boolean[] usedMask  = new boolean[4];

            for (ContactCandidate cand : candidates) {
                ContactData c = new ContactData();
                c.a          = a; c.b = b;
                c.normal     = sat.normal.clone();
                c.point      = cand.point.clone();
                c.penetration = Math.max(0, cand.penetration);
                initializeContact(c, dt);
                hadCache |= seedWarmStart(pairKey, warmRatio, usedMask, c);
                addInPlace(repSum, c.point);
                manifold.penetration = Math.max(manifold.penetration, c.penetration);
                manifold.contactCount++;
                contacts.add(c);
            }

            if (manifold.contactCount > 0) {
                float inv = 1.0f / manifold.contactCount;
                manifold.repPoint = new float[]{ repSum[0]*inv, repSum[1]*inv, repSum[2]*inv };
                manifolds.add(manifold);
                if (shouldWakePair(a, b, manifold.repPoint, manifold.normal,
                                   manifold.penetration, hadCache)) {
                    if (isTrulyDynamic(a)) { sleeping.get(a)[0] = false; sleepCounter.get(a)[0] = 0; }
                    if (isTrulyDynamic(b)) { sleeping.get(b)[0] = false; sleepCounter.get(b)[0] = 0; }
                }
            }
        }
    }

    // ========================================================================
    // 约束求解
    // ========================================================================

    /**
     * 顺序脉冲速度约束求解（摩擦优先，法线靠后）。
     * 每次迭代依次处理两个切线轴（Coulomb 圆锥裁切），再处理法线轴。
     */
    private void solveVelocityConstraints() {
        for (ContactData c : contacts) {
            float[] va = computePointVelocity(c.a, c.r1);
            float[] vb = computePointVelocity(c.b, c.r2);
            float[] rv = sub3(vb, va);

            // --- 摩擦 ---
            float jt1 = -dot(rv, c.tangent1) * c.t1EffMass;
            float jt2 = -dot(rv, c.tangent2) * c.t2EffMass;
            float oldT1 = c.t1Lambda, oldT2 = c.t2Lambda;
            float newT1 = oldT1 + jt1, newT2 = oldT2 + jt2;

            float maxFriction = c.combinedFriction * c.normalLambda;
            float tLenSq = newT1 * newT1 + newT2 * newT2;
            if (tLenSq > maxFriction * maxFriction && tLenSq > EPS * EPS) {
                float s = maxFriction / (float) Math.sqrt(tLenSq);
                newT1 *= s; newT2 *= s;
            }
            float apT1 = newT1 - oldT1, apT2 = newT2 - oldT2;
            c.t1Lambda = newT1; c.t2Lambda = newT2;

            float[] tImpulse = add3(scale3(c.tangent1, apT1), scale3(c.tangent2, apT2));
            if (lengthSq(tImpulse) > 0) applyImpulsePair(c.a, c.b, tImpulse, c.r1, c.r2);

            // --- 法线 ---
            float[] vaAfter   = computePointVelocity(c.a, c.r1);
            float[] vbAfter   = computePointVelocity(c.b, c.r2);
            float normalSpeed = dot(sub3(vbAfter, vaAfter), c.normal);
            float restBias    = (normalSpeed < -RESTITUTION_THRESHOLD) ? -c.restitution * normalSpeed : 0;
            float delta       = c.normalEffMass * (c.bias + restBias - normalSpeed);
            float oldN        = c.normalLambda;
            c.normalLambda    = Math.max(0, oldN + delta);
            float apN         = c.normalLambda - oldN;
            if (apN != 0) applyImpulsePair(c.a, c.b, scale3(c.normal, apN), c.r1, c.r2);
        }
    }

    /**
     * 伪速度位置修正（Baumgarte，每次迭代重新测量穿透深度）。
     */
    private void solvePositionConstraints() {
        for (ManifoldData manifold : manifolds) {
            int a = manifold.a, b = manifold.b;
            if (!solverEnabled.get(a)[0] || !solverEnabled.get(b)[0]) continue;

            float freshPen = remeasurePenetration(obbProxy.get(a), obbProxy.get(b), manifold.normal);
            float depth    = Math.max(freshPen - PENETRATION_SLOP, 0);
            if (depth <= 0) continue;

            float[] r1 = sub3(manifold.repPoint, position.get(a));
            float[] r2 = sub3(manifold.repPoint, position.get(b));
            float effMass = computeConstraintEffMass(a, b, r1, r2, manifold.normal);
            if (effMass <= 0) continue;

            float corrDist   = Math.min(POSITION_BAUMGARTE * depth, MAX_POSITION_CORRECTION);
            float pseudoLambda = corrDist * effMass;
            applyPositionImpulsePair(a, b, scale3(manifold.normal, pseudoLambda), r1, r2);
            updateDerivedBodyState(a);
            updateDerivedBodyState(b);
        }
    }

    private void updateContactCache() {
        nextCache.clear();
        for (ManifoldData manifold : manifolds) {
            CachedManifold cached = new CachedManifold();
            cached.normal = manifold.normal.clone();
            cached.count  = Math.min(manifold.contactCount, 4);
            for (int i = 0; i < cached.count; i++) {
                ContactData c = contacts.get(manifold.firstContact + i);
                cached.points[i].point       = c.point.clone();
                cached.points[i].normalLambda = c.normalLambda;
                cached.points[i].t1Lambda     = c.t1Lambda;
                cached.points[i].t2Lambda     = c.t2Lambda;
            }
            nextCache.put(makePairKey(manifold.a, manifold.b), cached);
        }
        Map<Long, CachedManifold> tmp = prevCache;
        prevCache = nextCache;
        nextCache = tmp;
    }

    private void updateSleepState() {
        for (int h : activeHandles) {
            if (!isTrulyDynamic(h)) {
                sleeping.get(h)[0]     = (motionType.get(h) == MotionType.STATIC);
                sleepCounter.get(h)[0] = 0;
                continue;
            }
            float lvSq = lengthSq(linearVelocity.get(h));
            float avSq = lengthSq(angularVelocity.get(h));
            boolean canSleep = (lvSq <= SLEEP_LIN_THRESHOLD_SQ && avSq <= SLEEP_ANG_THRESHOLD_SQ);
            if (!canSleep) { sleeping.get(h)[0] = false; sleepCounter.get(h)[0] = 0; continue; }

            int cnt = sleepCounter.get(h)[0];
            if (cnt < SLEEP_TICKS_REQUIRED) sleepCounter.get(h)[0] = cnt + 1;
            if (sleepCounter.get(h)[0] >= SLEEP_TICKS_REQUIRED) {
                sleeping.get(h)[0] = true;
                float[] lv = linearVelocity.get(h);  lv[0] = 0; lv[1] = 0; lv[2] = 0;
                float[] av = angularVelocity.get(h);  av[0] = 0; av[1] = 0; av[2] = 0;
            }
        }
    }

    private void clearAccumulators() {
        for (int h : activeHandles) {
            float[] fa = forceAcc.get(h);  fa[0] = 0; fa[1] = 0; fa[2] = 0;
            float[] ta = torqueAcc.get(h); ta[0] = 0; ta[1] = 0; ta[2] = 0;
        }
    }

    // ========================================================================
    // 简单的自测入口（可直接 java RigidBodyPhysics 运行）
    // ========================================================================

    public static void main(String[] args) {
        System.out.println("=== RigidBodyPhysics self-test ===");

        Config cfg = new Config();
        RigidBodyPhysics sim = new RigidBodyPhysics(cfg);

        // 地面（Static Box）
        BodyDesc ground = new BodyDesc();
        ground.motionType   = MotionType.STATIC;
        ground.shapeType    = ShapeType.BOX;
        ground.position     = new Vec3(0, -0.25f, 0);
        ground.boxHalfExtents = new Vec3(10, 0.25f, 10);
        ground.mass         = 0;
        ground.friction     = 0.5f;
        sim.addBody(ground);

        // 动态盒子（从高处落下）
        BodyDesc box = new BodyDesc();
        box.motionType    = MotionType.DYNAMIC;
        box.shapeType     = ShapeType.BOX;
        box.position      = new Vec3(0, 5, 0);
        box.boxHalfExtents = new Vec3(0.5f, 0.5f, 0.5f);
        box.mass          = 1.0f;
        box.friction      = 0.5f;
        int boxHandle = sim.addBody(box);

        System.out.printf("Initial box Y = %.4f%n", sim.getBodyState(boxHandle).position.y);

        // 模拟 3 秒 @ 60 Hz
        float dt = 1.0f / 60.0f;
        for (int tick = 0; tick < 180; tick++) {
            sim.step(dt);
        }

        BodyState state = sim.getBodyState(boxHandle);
        System.out.printf("After 3s: pos=(%.4f, %.4f, %.4f) sleeping=%b contacts=%d%n",
                state.position.x, state.position.y, state.position.z,
                state.sleeping, sim.getContactCount());
        System.out.printf("Bodies=%d  Sleeping=%d%n", sim.getBodyCount(), sim.getSleepingCount());
        System.out.println("=== Test passed ===");
    }
}

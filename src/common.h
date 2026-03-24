// ============================================================================
// common.h — 项目的基础类型与数学辅助函数
// ============================================================================
// 本文件定义了整个项目共用的基础数学类型（Vec3、Quat、Transform）以及
// 常用的数学运算函数。所有模块（World、Physics、Renderer）都依赖此头文件。
//
// 设计原则：
//   - 不依赖任何外部库（raylib / Jolt），保持纯 C++ 实现
//   - 使用 inline 函数避免多重定义问题
//   - 所有类型都有合理默认值（零向量、单位四元数等）
// ============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace lab {

// --- 核心类型别名 ---

// ObjectId：场景中每个对象的唯一标识符，作为 SoA 数组的索引使用
using ObjectId = std::uint32_t;
// 无效对象 ID 的哨兵值，用于表示"未选中"或"不存在"
constexpr ObjectId kInvalidObject = std::numeric_limits<ObjectId>::max();

// PhysicsBodyHandle：物理后端内部用来标识刚体的句柄
// 每个物理后端可以自由定义其含义（例如 Jolt 用 BodyID 的组合索引）
using PhysicsBodyHandle = std::uint32_t;
// 无效物理句柄的哨兵值
constexpr PhysicsBodyHandle kInvalidBodyHandle = std::numeric_limits<PhysicsBodyHandle>::max();

// --- 基础数学结构体 ---

// 三维向量，默认值为零向量 (0, 0, 0)
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// 四元数，用于表示 3D 旋转
// 默认值为单位四元数 (0, 0, 0, 1)，代表"无旋转"
// 存储顺序为 (x, y, z, w)，其中 w 是实部
struct Quat {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;
};

// 颜色结构体，RGBA 各 8 位，默认为不透明白色
struct ColorRGBA8 {
    std::uint8_t r = 255;
    std::uint8_t g = 255;
    std::uint8_t b = 255;
    std::uint8_t a = 255;
};

// 仿射变换：位置 + 旋转 + 缩放
// 这是场景中每个对象最基本的空间描述
struct Transform {
    Vec3 position{0.0f, 0.0f, 0.0f};   // 世界空间位置
    Quat rotation{0.0f, 0.0f, 0.0f, 1.0f}; // 世界空间旋转（单位四元数）
    Vec3 scale{1.0f, 1.0f, 1.0f};       // 各轴缩放因子
};

// --- 工厂 / 便利构造函数 ---

// 构造一个 Vec3
inline Vec3 MakeVec3(float x, float y, float z) {
    return Vec3{x, y, z};
}

// 构造一个 Quat
inline Quat MakeQuat(float x, float y, float z, float w) {
    return Quat{x, y, z, w};
}

// 返回单位四元数（无旋转）
inline Quat IdentityQuat() {
    return Quat{0.0f, 0.0f, 0.0f, 1.0f};
}

// --- 标量工具函数 ---

// 将 value 钳制到 [min_value, max_value] 范围内
inline float Clamp(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(value, max_value));
}

// --- Vec3 算术运算符 ---

// 向量加法
inline Vec3 operator+(Vec3 a, Vec3 b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

// 向量减法
inline Vec3 operator-(Vec3 a, Vec3 b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

// 向量取反
inline Vec3 operator-(Vec3 v) {
    return Vec3{-v.x, -v.y, -v.z};
}

// 向量乘标量（右乘）
inline Vec3 operator*(Vec3 v, float s) {
    return Vec3{v.x * s, v.y * s, v.z * s};
}

// 向量乘标量（左乘）
inline Vec3 operator*(float s, Vec3 v) {
    return v * s;
}

// 向量除以标量
inline Vec3 operator/(Vec3 v, float s) {
    return Vec3{v.x / s, v.y / s, v.z / s};
}

// 复合赋值：向量加法
inline Vec3& operator+=(Vec3& a, Vec3 b) {
    a = a + b;
    return a;
}

// 复合赋值：向量减法
inline Vec3& operator-=(Vec3& a, Vec3 b) {
    a = a - b;
    return a;
}

// 复合赋值：向量乘标量
inline Vec3& operator*=(Vec3& a, float s) {
    a = a * s;
    return a;
}

// --- Vec3 数学函数 ---

// 分量逐元素乘法（Hadamard 积），常用于将缩放应用到半尺寸上
inline Vec3 MultiplyComponents(Vec3 a, Vec3 b) {
    return Vec3{a.x * b.x, a.y * b.y, a.z * b.z};
}

// 向量点积（内积）
inline float Dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 向量叉积（外积），结果垂直于 a 和 b 所在平面
inline Vec3 Cross(Vec3 a, Vec3 b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

// 向量长度的平方（避免开方，用于比较距离时更高效）
inline float LengthSq(Vec3 v) {
    return Dot(v, v);
}

// 向量长度（欧几里得范数）
inline float Length(Vec3 v) {
    return std::sqrt(LengthSq(v));
}

// 归一化向量为单位长度；若输入接近零向量则返回零向量
inline Vec3 Normalize(Vec3 v) {
    const float len = Length(v);
    if (len <= 1e-6f) {
        return Vec3{0.0f, 0.0f, 0.0f};
    }
    return v / len;
}

// 向量线性插值：alpha=0 返回 a，alpha=1 返回 b
inline Vec3 Lerp(Vec3 a, Vec3 b, float alpha) {
    return a * (1.0f - alpha) + b * alpha;
}

// --- 四元数运算 ---

// 归一化四元数为单位长度；若接近零则返回单位四元数
inline Quat Normalize(Quat q) {
    const float len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (len <= 1e-6f) {
        return IdentityQuat();
    }
    const float inv = 1.0f / len;
    return Quat{q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

// 四元数共轭：将虚部取反，等价于逆旋转（前提是单位四元数）
inline Quat Conjugate(Quat q) {
    return Quat{-q.x, -q.y, -q.z, q.w};
}

// 四元数乘法（Hamilton 积），用于组合两个旋转
// 结果表示先应用 b 旋转，再应用 a 旋转
inline Quat operator*(Quat a, Quat b) {
    return Quat{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

// 用四元数旋转一个向量：v' = q * v * q⁻¹
// 这是将向量 v 按旋转 q 变换到新方向的标准方法
inline Vec3 Rotate(Quat q, Vec3 v) {
    const Quat unit = Normalize(q);
    const Quat p{v.x, v.y, v.z, 0.0f};           // 将向量升维为纯四元数
    const Quat rotated = unit * p * Conjugate(unit); // 三明治公式
    return Vec3{rotated.x, rotated.y, rotated.z};    // 提取旋转后的向量部分
}

// 从轴-角表示构造四元数
// axis：旋转轴（会被内部归一化），radians：旋转角度（弧度）
// 公式：q = (sin(θ/2) * axis, cos(θ/2))
inline Quat FromAxisAngle(Vec3 axis, float radians) {
    const Vec3 n = Normalize(axis);
    const float half = radians * 0.5f;
    const float s = std::sin(half);
    return Normalize(Quat{n.x * s, n.y * s, n.z * s, std::cos(half)});
}

// 归一化线性插值（Nlerp）：在两个四元数之间做插值
// 比 Slerp 便宜，恒定速度特性稍差但足够用于渲染插值
// dot < 0 时翻转 b，确保走最短路径
inline Quat Nlerp(Quat a, Quat b, float alpha) {
    float dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (dot < 0.0f) {
        // 四元数的 q 和 -q 代表相同旋转，选 dot > 0 的那条路径（最短弧）
        b.x = -b.x;
        b.y = -b.y;
        b.z = -b.z;
        b.w = -b.w;
    }
    Quat mixed{
        a.x * (1.0f - alpha) + b.x * alpha,
        a.y * (1.0f - alpha) + b.y * alpha,
        a.z * (1.0f - alpha) + b.z * alpha,
        a.w * (1.0f - alpha) + b.w * alpha,
    };
    return Normalize(mixed);
}

// --- Transform 插值 ---

// 在两帧的 Transform 之间做插值，用于渲染时的平滑显示
// 位置和缩放用 Lerp，旋转用 Nlerp
inline Transform Interpolate(Transform previous, Transform current, float alpha) {
    return Transform{
        Lerp(previous.position, current.position, alpha),
        Nlerp(previous.rotation, current.rotation, alpha),
        Lerp(previous.scale, current.scale, alpha),
    };
}

// 将四元数分解回轴-角表示
// out_axis：输出旋转轴，out_angle_radians：输出旋转角度
// 当角度接近 0 时退化为任意轴（默认 Y 轴）
inline void QuatToAxisAngle(Quat input, Vec3& out_axis, float& out_angle_radians) {
    const Quat q = Normalize(input);
    // w = cos(θ/2)，所以 θ = 2 * acos(w)
    out_angle_radians = 2.0f * std::acos(Clamp(q.w, -1.0f, 1.0f));
    // 虚部 = sin(θ/2) * axis
    const float s = std::sqrt(std::max(0.0f, 1.0f - q.w * q.w));
    if (s < 1e-4f) {
        // 角度接近 0，轴不确定，给一个默认值
        out_axis = Vec3{0.0f, 1.0f, 0.0f};
        out_angle_radians = 0.0f;
        return;
    }
    out_axis = Vec3{q.x / s, q.y / s, q.z / s};
}

}  // namespace lab

#include "physics_api.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lab {
namespace {

// ============================================================================
// NullPhysicsBackend v3
// ----------------------------------------------------------------------------
// Goals of this version:
//   1. Keep the existing World / main fixed-step lifecycle unchanged.
//   2. Read collider data only from World.colliders.
//   3. Keep box-only as the main path; Sphere/Capsule are still temporary boxified.
//   4. Upgrade the solver from translation-only contacts to real point contacts:
//      - rotation-aware OBB/SAT narrowphase
//      - face manifold clipping (up to 4 points)
//      - angular effective mass in sequential impulse
//      - Coulomb friction with two tangent axes
//   5. Still stay intentionally simple:
//      - no island builder
//      - no persistent manifold cache / warm start across frames
//      - no CCD
//      - no joints
//
// This is still intentionally small, but now it also adds simple temporal coherence
// (warm starting + lightweight manifold cache + calmer sleep/wake behavior) to make
// resting edge/line contacts converge more cleanly.
// ============================================================================

constexpr float kEpsilon = 1e-6f;
constexpr float kSleepLinearThresholdSq = 0.05f * 0.05f;
constexpr float kSleepAngularThresholdSq = 0.05f * 0.05f;
constexpr std::uint32_t kSleepTicksRequired = 30;
constexpr int kVelocityIterations = 10;
constexpr int kPositionIterations = 4;
constexpr float kVelocityBaumgarte = 0.08f;
constexpr float kPositionBaumgarte = 0.25f;
constexpr float kPenetrationSlop = 0.01f;
constexpr float kMaxPositionCorrection = 0.08f;
constexpr float kMaxAngularPositionStep = 0.35f;
constexpr float kRestitutionThreshold = 1.0f;
constexpr float kDuplicatePointEpsilonSq = 1e-4f * 1e-4f;
constexpr float kWarmStartPointMatchDistanceSq = 0.08f * 0.08f;
constexpr float kWarmStartNormalDotMin = 0.92f;
constexpr float kRestingBiasVelocityThreshold = 0.25f;
constexpr float kRestingBiasPenetration = 0.02f;
constexpr float kWakeNormalSpeedThreshold = 0.35f;
constexpr float kWakeTangentSpeedThreshold = 0.25f;
constexpr float kWakePenetrationThreshold = 0.03f;

inline Vec3 AbsVec3(Vec3 v) {
    return Vec3{std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)};
}

inline Vec3 SafeDivide(Vec3 v, float s) {
    return (std::fabs(s) <= kEpsilon) ? Vec3{0.0f, 0.0f, 0.0f} : (v / s);
}

inline float LengthSqNoNaN(Vec3 v) {
    return Dot(v, v);
}

inline Vec3 HadamardAbsScale(Vec3 value, Vec3 scale) {
    return MultiplyComponents(value, AbsVec3(scale));
}

inline float Vec3At(Vec3 v, int i) {
    return (i == 0) ? v.x : (i == 1) ? v.y : v.z;
}

inline Quat IntegrateRotationEuler(Quat q, Vec3 angular_velocity, float dt) {
    const Quat omega{angular_velocity.x, angular_velocity.y, angular_velocity.z, 0.0f};
    const Quat dq = omega * q;
    Quat out{
        q.x + 0.5f * dq.x * dt,
        q.y + 0.5f * dq.y * dt,
        q.z + 0.5f * dq.z * dt,
        q.w + 0.5f * dq.w * dt,
    };
    return Normalize(out);
}

inline Quat ApplyRotationDelta(Quat q, Vec3 delta_angle) {
    const float len_sq = LengthSqNoNaN(delta_angle);
    if (len_sq <= kEpsilon * kEpsilon) {
        return q;
    }

    const float len = std::sqrt(len_sq);
    if (len > kMaxAngularPositionStep) {
        delta_angle *= (kMaxAngularPositionStep / len);
    }

    return IntegrateRotationEuler(q, delta_angle, 1.0f);
}

enum class SolverShapeKind : std::uint8_t {
    None,
    Box,
};

struct ColliderProxy {
    SolverShapeKind solver_shape = SolverShapeKind::None;
    ShapeKind source_shape = ShapeKind::Box;
    Vec3 box_half_extents_local{0.5f, 0.5f, 0.5f};
    Vec3 local_offset{0.0f, 0.0f, 0.0f};
    Quat local_rotation{0.0f, 0.0f, 0.0f, 1.0f};
};

struct Aabb {
    Vec3 min{0.0f, 0.0f, 0.0f};
    Vec3 max{0.0f, 0.0f, 0.0f};
};

struct BroadphasePair {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
};

struct BoxWorldProxy {
    Vec3 center{0.0f, 0.0f, 0.0f};
    Vec3 axes[3]{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    Vec3 half_extents{0.5f, 0.5f, 0.5f};
};

struct BoxBoxSatResult {
    bool separated = true;
    Vec3 normal{0.0f, 1.0f, 0.0f};
    float penetration = 0.0f;
    int axis_index = -1; // 0-2 A face, 3-5 B face, 6-14 edge-edge
    int edge_a = -1;
    int edge_b = -1;
    Vec3 point{0.0f, 0.0f, 0.0f};
};

struct ContactPointCandidate {
    Vec3 point{0.0f, 0.0f, 0.0f};
    float penetration = 0.0f;
};

struct ContactTangentBasis {
    Vec3 tangent1{1.0f, 0.0f, 0.0f};
    Vec3 tangent2{0.0f, 0.0f, 1.0f};
};

struct Contact {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 normal{0.0f, 1.0f, 0.0f}; // from a to b
    Vec3 point{0.0f, 0.0f, 0.0f};
    float penetration = 0.0f;

    Vec3 r1{0.0f, 0.0f, 0.0f};
    Vec3 r2{0.0f, 0.0f, 0.0f};
    Vec3 tangent1{1.0f, 0.0f, 0.0f};
    Vec3 tangent2{0.0f, 0.0f, 1.0f};

    float combined_friction = 0.6f;
    float restitution = 0.0f;
    float bias = 0.0f;

    float normal_effective_mass = 0.0f;
    float tangent1_effective_mass = 0.0f;
    float tangent2_effective_mass = 0.0f;

    float normal_lambda = 0.0f;
    float tangent1_lambda = 0.0f;
    float tangent2_lambda = 0.0f;
};

struct ContactManifold {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    Vec3 normal{0.0f, 1.0f, 0.0f};
    Vec3 representative_point{0.0f, 0.0f, 0.0f};
    float penetration = 0.0f;
    std::uint32_t first_contact = 0;
    std::uint32_t contact_count = 0;
};

struct CachedContactPoint {
    Vec3 point{0.0f, 0.0f, 0.0f};
    float normal_lambda = 0.0f;
    float tangent1_lambda = 0.0f;
    float tangent2_lambda = 0.0f;
};

struct CachedManifold {
    Vec3 normal{0.0f, 1.0f, 0.0f};
    std::uint32_t contact_count = 0;
    CachedContactPoint points[4]{};
};

inline std::uint64_t MakePairKey(std::uint32_t a, std::uint32_t b) {
    const std::uint64_t lo = std::min(a, b);
    const std::uint64_t hi = std::max(a, b);
    return (hi << 32u) | lo;
}

ColliderProxy BuildColliderProxyFromWorld(const World& world, ObjectId id) {
    ColliderProxy proxy{};

    if (!world.HasCollider(id) || world.colliders.enabled[id] == 0) {
        proxy.solver_shape = SolverShapeKind::None;
        return proxy;
    }

    proxy.source_shape = world.colliders.shape[id];
    proxy.local_offset = world.colliders.local_offset[id];
    proxy.local_rotation = world.colliders.local_rotation[id];

    switch (proxy.source_shape) {
        case ShapeKind::Box:
            proxy.solver_shape = SolverShapeKind::Box;
            proxy.box_half_extents_local = world.colliders.box_half_extents[id];
            break;

        case ShapeKind::Sphere: {
            const float r = world.colliders.sphere_radius[id];
            proxy.solver_shape = SolverShapeKind::Box;
            proxy.box_half_extents_local = Vec3{r, r, r};
            break;
        }

        case ShapeKind::Capsule: {
            const float r = world.colliders.capsule_radius[id];
            const float hh = world.colliders.capsule_half_height[id];
            proxy.solver_shape = SolverShapeKind::Box;
            proxy.box_half_extents_local = Vec3{r, r + hh, r};
            break;
        }

        default:
            proxy.solver_shape = SolverShapeKind::None;
            break;
    }

    return proxy;
}

bool TestAabbOverlap(const Aabb& a, const Aabb& b) {
    return !(a.max.x < b.min.x || a.min.x > b.max.x ||
             a.max.y < b.min.y || a.min.y > b.max.y ||
             a.max.z < b.min.z || a.min.z > b.max.z);
}

Vec3 ComputeBoxInverseInertiaDiagonal(float mass, Vec3 half_extents) {
    if (mass <= kEpsilon) {
        return Vec3{0.0f, 0.0f, 0.0f};
    }

    const float sx = 2.0f * half_extents.x;
    const float sy = 2.0f * half_extents.y;
    const float sz = 2.0f * half_extents.z;

    const float ix = (mass / 12.0f) * (sy * sy + sz * sz);
    const float iy = (mass / 12.0f) * (sx * sx + sz * sz);
    const float iz = (mass / 12.0f) * (sx * sx + sy * sy);

    return Vec3{
        (ix > kEpsilon) ? (1.0f / ix) : 0.0f,
        (iy > kEpsilon) ? (1.0f / iy) : 0.0f,
        (iz > kEpsilon) ? (1.0f / iz) : 0.0f,
    };
}

BoxWorldProxy MakeBoxWorldProxy(Vec3 center, Quat world_rotation, Vec3 half_extents) {
    BoxWorldProxy proxy;
    proxy.center = center;
    proxy.axes[0] = Normalize(Rotate(world_rotation, Vec3{1.0f, 0.0f, 0.0f}));
    proxy.axes[1] = Normalize(Rotate(world_rotation, Vec3{0.0f, 1.0f, 0.0f}));
    proxy.axes[2] = Normalize(Rotate(world_rotation, Vec3{0.0f, 0.0f, 1.0f}));
    proxy.half_extents = half_extents;
    return proxy;
}

Aabb AabbFromObb(const BoxWorldProxy& obb) {
    Vec3 h;
    h.x = std::fabs(obb.axes[0].x) * obb.half_extents.x
        + std::fabs(obb.axes[1].x) * obb.half_extents.y
        + std::fabs(obb.axes[2].x) * obb.half_extents.z;
    h.y = std::fabs(obb.axes[0].y) * obb.half_extents.x
        + std::fabs(obb.axes[1].y) * obb.half_extents.y
        + std::fabs(obb.axes[2].y) * obb.half_extents.z;
    h.z = std::fabs(obb.axes[0].z) * obb.half_extents.x
        + std::fabs(obb.axes[1].z) * obb.half_extents.y
        + std::fabs(obb.axes[2].z) * obb.half_extents.z;
    return Aabb{obb.center - h, obb.center + h};
}

ContactTangentBasis MakeTangentBasis(Vec3 normal) {
    ContactTangentBasis basis;
    const Vec3 hint = (std::fabs(normal.x) < 0.57f) ? Vec3{1.0f, 0.0f, 0.0f}
                                                    : Vec3{0.0f, 1.0f, 0.0f};
    basis.tangent1 = Normalize(Cross(normal, hint));
    if (LengthSqNoNaN(basis.tangent1) <= kEpsilon * kEpsilon) {
        basis.tangent1 = Normalize(Cross(normal, Vec3{0.0f, 0.0f, 1.0f}));
    }
    basis.tangent2 = Normalize(Cross(normal, basis.tangent1));
    return basis;
}

Vec3 ObbSupport(const BoxWorldProxy& obb, Vec3 dir) {
    Vec3 result = obb.center;
    for (int i = 0; i < 3; ++i) {
        const float sign = (Dot(dir, obb.axes[i]) >= 0.0f) ? 1.0f : -1.0f;
        result += obb.axes[i] * (sign * Vec3At(obb.half_extents, i));
    }
    return result;
}

float RemeasurePenetration(const BoxWorldProxy& a, const BoxWorldProxy& b, Vec3 normal) {
    float ra = 0.0f;
    float rb = 0.0f;
    for (int i = 0; i < 3; ++i) {
        ra += std::fabs(Dot(a.axes[i], normal)) * Vec3At(a.half_extents, i);
        rb += std::fabs(Dot(b.axes[i], normal)) * Vec3At(b.half_extents, i);
    }
    const float dist = std::fabs(Dot(b.center - a.center, normal));
    return ra + rb - dist;
}

BoxBoxSatResult BoxBoxSat(const BoxWorldProxy& a, const BoxWorldProxy& b) {
    BoxBoxSatResult result;
    result.separated = false;
    result.penetration = std::numeric_limits<float>::max();
    result.normal = Vec3{0.0f, 1.0f, 0.0f};
    result.axis_index = -1;
    result.edge_a = -1;
    result.edge_b = -1;
    result.point = (a.center + b.center) * 0.5f;

    const Vec3 d = b.center - a.center;

    auto test_axis = [&](Vec3 axis, int axis_index, int edge_a, int edge_b) -> bool {
        const float axis_len_sq = LengthSqNoNaN(axis);
        if (axis_len_sq < kEpsilon * kEpsilon) {
            return true;
        }
        axis = SafeDivide(axis, std::sqrt(axis_len_sq));
        if (Dot(axis, d) < 0.0f) {
            axis = -axis;
        }

        float ra = 0.0f;
        float rb = 0.0f;
        for (int k = 0; k < 3; ++k) {
            ra += std::fabs(Dot(a.axes[k], axis)) * Vec3At(a.half_extents, k);
            rb += std::fabs(Dot(b.axes[k], axis)) * Vec3At(b.half_extents, k);
        }

        const float overlap = ra + rb - std::fabs(Dot(d, axis));
        if (overlap < 0.0f) {
            result.separated = true;
            return false;
        }

        const float biased_overlap = (axis_index >= 6) ? (overlap + 1e-4f) : overlap;
        if (biased_overlap < result.penetration) {
            result.penetration = overlap;
            result.normal = axis;
            result.axis_index = axis_index;
            result.edge_a = edge_a;
            result.edge_b = edge_b;
        }
        return true;
    };

    if (!test_axis(a.axes[0], 0, -1, -1)) return result;
    if (!test_axis(a.axes[1], 1, -1, -1)) return result;
    if (!test_axis(a.axes[2], 2, -1, -1)) return result;

    if (!test_axis(b.axes[0], 3, -1, -1)) return result;
    if (!test_axis(b.axes[1], 4, -1, -1)) return result;
    if (!test_axis(b.axes[2], 5, -1, -1)) return result;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!test_axis(Cross(a.axes[i], b.axes[j]), 6 + i * 3 + j, i, j)) {
                return result;
            }
        }
    }

    return result;
}

void BuildFaceVertices(const BoxWorldProxy& box,
                       int face_axis,
                       float face_sign,
                       Vec3& out_center,
                       Vec3& out_normal,
                       Vec3 out_vertices[4]) {
    out_normal = box.axes[face_axis] * face_sign;
    out_center = box.center + out_normal * Vec3At(box.half_extents, face_axis);

    const int u_idx = (face_axis + 1) % 3;
    const int v_idx = (face_axis + 2) % 3;
    const Vec3 u = box.axes[u_idx] * Vec3At(box.half_extents, u_idx);
    const Vec3 v = box.axes[v_idx] * Vec3At(box.half_extents, v_idx);

    out_vertices[0] = out_center + u + v;
    out_vertices[1] = out_center - u + v;
    out_vertices[2] = out_center - u - v;
    out_vertices[3] = out_center + u - v;

    const Vec3 winding = Cross(out_vertices[1] - out_vertices[0], out_vertices[2] - out_vertices[1]);
    if (Dot(winding, out_normal) < 0.0f) {
        std::swap(out_vertices[1], out_vertices[3]);
    }
}

void BuildMostAlignedFace(const BoxWorldProxy& box,
                          Vec3 direction,
                          Vec3& out_center,
                          Vec3& out_normal,
                          Vec3 out_vertices[4]) {
    int best_axis = 0;
    float best_dot = Dot(box.axes[0], direction);
    float best_abs_dot = std::fabs(best_dot);

    for (int i = 1; i < 3; ++i) {
        const float d = Dot(box.axes[i], direction);
        const float ad = std::fabs(d);
        if (ad > best_abs_dot) {
            best_abs_dot = ad;
            best_dot = d;
            best_axis = i;
        }
    }

    const float sign = (best_dot >= 0.0f) ? 1.0f : -1.0f;
    BuildFaceVertices(box, best_axis, sign, out_center, out_normal, out_vertices);
}

std::vector<Vec3> ClipPolygonAgainstPlane(const std::vector<Vec3>& input,
                                          Vec3 plane_point,
                                          Vec3 plane_normal) {
    std::vector<Vec3> output;
    if (input.empty()) {
        return output;
    }

    Vec3 a = input.back();
    float da = Dot(a - plane_point, plane_normal);
    bool inside_a = (da >= -kEpsilon);

    for (const Vec3& b : input) {
        const float db = Dot(b - plane_point, plane_normal);
        const bool inside_b = (db >= -kEpsilon);

        if (inside_a && inside_b) {
            output.push_back(b);
        } else if (inside_a && !inside_b) {
            const float t = da / (da - db);
            output.push_back(a + (b - a) * t);
        } else if (!inside_a && inside_b) {
            const float t = da / (da - db);
            output.push_back(a + (b - a) * t);
            output.push_back(b);
        }

        a = b;
        da = db;
        inside_a = inside_b;
    }

    return output;
}

void DeduplicateContactCandidates(std::vector<ContactPointCandidate>& candidates) {
    std::vector<ContactPointCandidate> unique;
    unique.reserve(candidates.size());

    for (const ContactPointCandidate& c : candidates) {
        bool duplicate = false;
        for (const ContactPointCandidate& u : unique) {
            if (LengthSqNoNaN(c.point - u.point) <= kDuplicatePointEpsilonSq) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            unique.push_back(c);
        }
    }

    candidates.swap(unique);
}

std::vector<ContactPointCandidate> PruneContactCandidates(const std::vector<ContactPointCandidate>& input,
                                                          Vec3 normal) {
    if (input.size() <= 4) {
        return input;
    }

    std::vector<Vec3> projected(input.size());
    Vec3 centroid{0.0f, 0.0f, 0.0f};
    for (std::size_t i = 0; i < input.size(); ++i) {
        const float along_n = Dot(input[i].point, normal);
        projected[i] = input[i].point - normal * along_n;
        centroid += projected[i];
    }
    centroid = centroid / static_cast<float>(input.size());

    auto weighted_distance_sq = [&](std::size_t i, Vec3 ref) {
        const float dist_sq = LengthSqNoNaN(projected[i] - ref);
        const float depth_sq = input[i].penetration * input[i].penetration;
        return dist_sq * std::max(depth_sq, 1.0f);
    };

    std::vector<std::size_t> chosen;
    chosen.reserve(4);

    std::size_t first = 0;
    float first_score = -1.0f;
    for (std::size_t i = 0; i < input.size(); ++i) {
        const float score = weighted_distance_sq(i, centroid);
        if (score > first_score) {
            first_score = score;
            first = i;
        }
    }
    chosen.push_back(first);

    std::size_t second = first;
    float second_score = -1.0f;
    for (std::size_t i = 0; i < input.size(); ++i) {
        const float score = weighted_distance_sq(i, projected[first]);
        if (score > second_score) {
            second_score = score;
            second = i;
        }
    }
    if (second != first) {
        chosen.push_back(second);
    }

    Vec3 edge_dir = projected[second] - projected[first];
    if (LengthSqNoNaN(edge_dir) <= kEpsilon * kEpsilon) {
        edge_dir = Vec3{1.0f, 0.0f, 0.0f};
    } else {
        edge_dir = Normalize(edge_dir);
    }
    const Vec3 perp = Normalize(Cross(normal, edge_dir));

    std::size_t positive = input.size();
    std::size_t negative = input.size();
    float pos_dist = -1.0f;
    float neg_dist = -1.0f;
    for (std::size_t i = 0; i < input.size(); ++i) {
        if (i == first || i == second) {
            continue;
        }
        const float signed_dist = Dot(projected[i] - projected[first], perp);
        const float weighted = std::fabs(signed_dist) * std::max(input[i].penetration, 1.0f);
        if (signed_dist >= 0.0f && weighted > pos_dist) {
            pos_dist = weighted;
            positive = i;
        }
        if (signed_dist <= 0.0f && weighted > neg_dist) {
            neg_dist = weighted;
            negative = i;
        }
    }

    if (positive < input.size()) {
        chosen.push_back(positive);
    }
    if (negative < input.size() && negative != positive) {
        chosen.push_back(negative);
    }

    std::vector<ContactPointCandidate> output;
    output.reserve(4);
    for (std::size_t idx : chosen) {
        bool duplicate = false;
        for (const ContactPointCandidate& existing : output) {
            if (LengthSqNoNaN(existing.point - input[idx].point) <= kDuplicatePointEpsilonSq) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            output.push_back(input[idx]);
        }
    }

    while (output.size() < 4) {
        bool added = false;
        for (const ContactPointCandidate& c : input) {
            bool duplicate = false;
            for (const ContactPointCandidate& existing : output) {
                if (LengthSqNoNaN(existing.point - c.point) <= kDuplicatePointEpsilonSq) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                output.push_back(c);
                added = true;
                if (output.size() >= 4) {
                    break;
                }
            }
        }
        if (!added) {
            break;
        }
    }

    return output;
}

std::vector<ContactPointCandidate> BuildFaceManifoldCandidates(const BoxWorldProxy& a,
                                                               const BoxWorldProxy& b,
                                                               const BoxBoxSatResult& sat) {
    std::vector<ContactPointCandidate> contacts;

    const bool reference_is_a = (sat.axis_index >= 0 && sat.axis_index < 3);
    const BoxWorldProxy& reference_box = reference_is_a ? a : b;
    const BoxWorldProxy& incident_box = reference_is_a ? b : a;
    const int reference_axis = reference_is_a ? sat.axis_index : (sat.axis_index - 3);
    const Vec3 reference_plane_normal = reference_is_a ? sat.normal : -sat.normal;

    const float reference_sign = (Dot(reference_plane_normal, reference_box.axes[reference_axis]) >= 0.0f) ? 1.0f : -1.0f;

    Vec3 reference_center;
    Vec3 reference_normal;
    Vec3 reference_face[4];
    BuildFaceVertices(reference_box, reference_axis, reference_sign, reference_center, reference_normal, reference_face);

    Vec3 incident_center;
    Vec3 incident_normal;
    Vec3 incident_face_arr[4];
    BuildMostAlignedFace(incident_box, -reference_plane_normal, incident_center, incident_normal, incident_face_arr);

    std::vector<Vec3> polygon{
        incident_face_arr[0],
        incident_face_arr[1],
        incident_face_arr[2],
        incident_face_arr[3],
    };

    for (int i = 0; i < 4 && !polygon.empty(); ++i) {
        const Vec3 edge_start = reference_face[i];
        const Vec3 edge_end = reference_face[(i + 1) % 4];
        const Vec3 edge = edge_end - edge_start;
        const Vec3 side_plane_normal = Normalize(Cross(reference_normal, edge));
        polygon = ClipPolygonAgainstPlane(polygon, edge_start, side_plane_normal);
    }

    for (const Vec3& p_incident : polygon) {
        const float penetration = Dot(reference_center - p_incident, reference_plane_normal);
        if (penetration < -kPenetrationSlop) {
            continue;
        }
        const Vec3 p_reference = p_incident + reference_plane_normal * penetration;
        const Vec3 midpoint = (p_incident + p_reference) * 0.5f;
        contacts.push_back(ContactPointCandidate{midpoint, std::max(0.0f, penetration)});
    }

    DeduplicateContactCandidates(contacts);
    if (contacts.size() > 4) {
        contacts = PruneContactCandidates(contacts, sat.normal);
    }

    if (contacts.empty()) {
        const Vec3 p_incident = ObbSupport(incident_box, -reference_plane_normal);
        const float penetration = Dot(reference_center - p_incident, reference_plane_normal);
        const Vec3 p_reference = p_incident + reference_plane_normal * penetration;
        contacts.push_back(ContactPointCandidate{(p_incident + p_reference) * 0.5f,
                                                 std::max(0.0f, penetration)});
    }

    return contacts;
}

ContactPointCandidate BuildEdgeEdgeCandidate(const BoxWorldProxy& a,
                                             const BoxWorldProxy& b,
                                             const BoxBoxSatResult& sat) {
    const Vec3 d = b.center - a.center;
    const int edge_a = sat.edge_a;
    const int edge_b = sat.edge_b;

    Vec3 pA = a.center;
    for (int k = 0; k < 3; ++k) {
        if (k == edge_a) continue;
        const float s = (Dot(d, a.axes[k]) > 0.0f) ? Vec3At(a.half_extents, k)
                                                   : -Vec3At(a.half_extents, k);
        pA += a.axes[k] * s;
    }

    Vec3 pB = b.center;
    for (int k = 0; k < 3; ++k) {
        if (k == edge_b) continue;
        const float s = (Dot(-d, b.axes[k]) > 0.0f) ? Vec3At(b.half_extents, k)
                                                    : -Vec3At(b.half_extents, k);
        pB += b.axes[k] * s;
    }

    const Vec3 dirA = a.axes[edge_a];
    const Vec3 dirB = b.axes[edge_b];
    const Vec3 w = pA - pB;
    const float dotAA = Dot(dirA, dirA);
    const float dotAB = Dot(dirA, dirB);
    const float dotBB = Dot(dirB, dirB);
    const float dotWA = Dot(w, dirA);
    const float dotWB = Dot(w, dirB);
    const float denom = dotAA * dotBB - dotAB * dotAB;

    float t = 0.0f;
    float u = 0.0f;
    if (std::fabs(denom) > kEpsilon) {
        t = (dotAB * dotWB - dotBB * dotWA) / denom;
        u = (dotAA * dotWB - dotAB * dotWA) / denom;
    }

    t = Clamp(t, -Vec3At(a.half_extents, edge_a), Vec3At(a.half_extents, edge_a));
    u = Clamp(u, -Vec3At(b.half_extents, edge_b), Vec3At(b.half_extents, edge_b));

    const Vec3 cA = pA + dirA * t;
    const Vec3 cB = pB + dirB * u;
    return ContactPointCandidate{(cA + cB) * 0.5f, sat.penetration};
}

class NullPhysicsBackend final : public IPhysicsBackend {
public:
    const char* Name() const override {
        return "NullPhysicsBackendV3_1";
    }

    void Initialize(const World& world, const PhysicsConfig& config) override {
        config_ = config;
        simulation_time_ = 0.0;
        RebuildFromWorld(world);
    }

    void Shutdown() override {
        object_ids_.clear();
        object_to_index_.clear();

        motion_type_.clear();
        source_shape_.clear();
        solver_shape_.clear();
        solver_enabled_.clear();

        position_.clear();
        rotation_.clear();
        scale_.clear();
        linear_velocity_.clear();
        angular_velocity_.clear();
        previous_position_.clear();

        mass_.clear();
        inverse_mass_.clear();
        inverse_inertia_local_.clear();

        use_gravity_.clear();
        friction_.clear();
        linear_damping_.clear();
        angular_damping_.clear();
        sleeping_.clear();
        sleep_counter_.clear();

        center_of_mass_offset_local_.clear();
        collider_local_offset_.clear();
        collider_local_rotation_.clear();
        box_half_extents_local_.clear();
        box_half_extents_world_.clear();
        collider_center_world_.clear();
        aabb_.clear();
        obb_proxy_.clear();

        force_accumulator_.clear();
        torque_accumulator_.clear();

        broadphase_pairs_.clear();
        contacts_.clear();
        manifolds_.clear();
        previous_contact_cache_.clear();
        next_contact_cache_.clear();
        previous_step_dt_ = 0.0f;
    }

    void RebuildFromWorld(const World& world) override {
        Shutdown();

        const std::size_t count = world.ObjectCount();
        Reserve(count);

        for (ObjectId id = 0; id < static_cast<ObjectId>(count); ++id) {
            if (!world.IsValid(id) || !world.HasRigidBody(id)) {
                continue;
            }
            AddSingleBody(world, id);
        }
    }

    void SyncAuthoringToRuntime(const World& world) override {
        bool requires_rebuild = false;
        for (ObjectId id : object_ids_) {
            if (!world.IsValid(id) || !world.HasRigidBody(id)) {
                requires_rebuild = true;
                break;
            }
        }
        if (requires_rebuild) {
            RebuildFromWorld(world);
        }

        for (ObjectId id = 0; id < static_cast<ObjectId>(world.ObjectCount()); ++id) {
            if (!world.IsValid(id) || !world.HasRigidBody(id)) continue;
            if (object_to_index_.find(id) != object_to_index_.end()) continue;
            AddSingleBody(world, id);
        }

        for (std::size_t i = 0; i < object_ids_.size(); ++i) {
            const ObjectId id = object_ids_[i];
            RefreshAuthoringForBody(world, static_cast<std::uint32_t>(i), id);

            const MotionType mt = motion_type_[i];
            if (mt == MotionType::Static || mt == MotionType::Kinematic) {
                position_[i] = WorldTransformToBodyPosition(world, id);
                rotation_[i] = world.transforms.rotation[id];
                if (mt == MotionType::Static) {
                    linear_velocity_[i] = Vec3{0.0f, 0.0f, 0.0f};
                    angular_velocity_[i] = Vec3{0.0f, 0.0f, 0.0f};
                    sleeping_[i] = 1;
                }
                if (mt == MotionType::Kinematic) {
                    sleeping_[i] = 0;
                }
            }

            UpdateDerivedBodyState(static_cast<std::uint32_t>(i));
        }
    }

    void Step(float dt) override {
        simulation_time_ += static_cast<double>(dt);
        if (dt <= 0.0f) return;

        broadphase_pairs_.clear();
        contacts_.clear();
        manifolds_.clear();
        next_contact_cache_.clear();

        GatherExternalForces();
        IntegrateForces(dt);
        IntegrateVelocities(dt);
        UpdateAllDerivedBodyState();
        BuildBroadphasePairs();
        GenerateContacts(dt);
        WarmStartContacts();

        for (int iter = 0; iter < kVelocityIterations; ++iter) {
            SolveVelocityConstraints();
        }

        for (int iter = 0; iter < kPositionIterations; ++iter) {
            SolvePositionConstraints();
        }

        UpdateAllDerivedBodyState();
        UpdateContactCache();
        UpdateSleepState();
        ClearAccumulators();
        previous_step_dt_ = dt;
    }

    void SyncRuntimeToWorld(World& world) override {
        for (std::size_t i = 0; i < object_ids_.size(); ++i) {
            const ObjectId id = object_ids_[i];
            world.body_state.handle[id] = static_cast<PhysicsBodyHandle>(i);
            world.body_state.linear_velocity[id] = linear_velocity_[i];
            world.body_state.angular_velocity[id] = angular_velocity_[i];
            world.body_state.sleeping[id] = sleeping_[i];

            const MotionType mt = motion_type_[i];
            if (mt == MotionType::Dynamic || mt == MotionType::Kinematic) {
                world.transforms.position[id] = BodyPositionToWorldTransform(position_[i], rotation_[i], center_of_mass_offset_local_[i]);
                world.transforms.rotation[id] = rotation_[i];
            }
        }
    }

    PhysicsDebugStats GetDebugStats() const override {
        PhysicsDebugStats stats;
        stats.num_bodies = static_cast<int>(object_ids_.size());
        for (std::uint8_t s : sleeping_) {
            stats.num_sleeping += static_cast<int>(s);
        }
        stats.num_broadphase_pairs = static_cast<int>(broadphase_pairs_.size());
        stats.num_contacts = static_cast<int>(contacts_.size());
        stats.solver_velocity_iterations = kVelocityIterations;
        stats.solver_position_iterations = kPositionIterations;
        return stats;
    }

    void CollectDebugDrawData(PhysicsDebugDrawData& out) const override {
        for (const Contact& c : contacts_) {
            out.contacts.push_back({c.point, c.normal, c.penetration});
        }

        for (std::size_t i = 0; i < object_ids_.size(); ++i) {
            if (!solver_enabled_[i]) continue;

            const Vec3 transform_origin = BodyPositionToWorldTransform(position_[i], rotation_[i], center_of_mass_offset_local_[i]);
            out.body_coms.push_back({transform_origin, position_[i]});
            out.collider_frames.push_back({collider_center_world_[i], rotation_[i] * collider_local_rotation_[i]});
            out.aabbs.push_back({aabb_[i].min, aabb_[i].max});
        }
    }

private:
    void Reserve(std::size_t count) {
        object_ids_.reserve(count);
        motion_type_.reserve(count);
        source_shape_.reserve(count);
        solver_shape_.reserve(count);
        solver_enabled_.reserve(count);

        position_.reserve(count);
        rotation_.reserve(count);
        scale_.reserve(count);
        linear_velocity_.reserve(count);
        angular_velocity_.reserve(count);
        previous_position_.reserve(count);

        mass_.reserve(count);
        inverse_mass_.reserve(count);
        inverse_inertia_local_.reserve(count);

        use_gravity_.reserve(count);
        friction_.reserve(count);
        linear_damping_.reserve(count);
        angular_damping_.reserve(count);
        sleeping_.reserve(count);
        sleep_counter_.reserve(count);

        center_of_mass_offset_local_.reserve(count);
        collider_local_offset_.reserve(count);
        collider_local_rotation_.reserve(count);
        box_half_extents_local_.reserve(count);
        box_half_extents_world_.reserve(count);
        collider_center_world_.reserve(count);
        aabb_.reserve(count);
        obb_proxy_.reserve(count);

        force_accumulator_.reserve(count);
        torque_accumulator_.reserve(count);

        manifolds_.reserve(count);
    }

    void AddSingleBody(const World& world, ObjectId id) {
        const std::uint32_t index = static_cast<std::uint32_t>(object_ids_.size());
        object_ids_.push_back(id);
        object_to_index_[id] = index;

        motion_type_.push_back(MotionType::Static);
        source_shape_.push_back(ShapeKind::Box);
        solver_shape_.push_back(SolverShapeKind::None);
        solver_enabled_.push_back(0);

        position_.push_back(Vec3{0.0f, 0.0f, 0.0f});
        rotation_.push_back(IdentityQuat());
        scale_.push_back(Vec3{1.0f, 1.0f, 1.0f});
        linear_velocity_.push_back(Vec3{0.0f, 0.0f, 0.0f});
        angular_velocity_.push_back(Vec3{0.0f, 0.0f, 0.0f});
        previous_position_.push_back(Vec3{0.0f, 0.0f, 0.0f});

        mass_.push_back(0.0f);
        inverse_mass_.push_back(0.0f);
        inverse_inertia_local_.push_back(Vec3{0.0f, 0.0f, 0.0f});

        use_gravity_.push_back(0);
        friction_.push_back(0.6f);
        linear_damping_.push_back(0.0f);
        angular_damping_.push_back(0.0f);
        sleeping_.push_back(0);
        sleep_counter_.push_back(0);

        center_of_mass_offset_local_.push_back(Vec3{0.0f, 0.0f, 0.0f});
        collider_local_offset_.push_back(Vec3{0.0f, 0.0f, 0.0f});
        collider_local_rotation_.push_back(IdentityQuat());
        box_half_extents_local_.push_back(Vec3{0.5f, 0.5f, 0.5f});
        box_half_extents_world_.push_back(Vec3{0.5f, 0.5f, 0.5f});
        collider_center_world_.push_back(Vec3{0.0f, 0.0f, 0.0f});
        aabb_.push_back(Aabb{});
        obb_proxy_.push_back(BoxWorldProxy{});

        force_accumulator_.push_back(Vec3{0.0f, 0.0f, 0.0f});
        torque_accumulator_.push_back(Vec3{0.0f, 0.0f, 0.0f});

        RefreshAuthoringForBody(world, index, id);

        position_[index] = WorldTransformToBodyPosition(world, id);
        previous_position_[index] = position_[index];
        rotation_[index] = world.transforms.rotation[id];
        linear_velocity_[index] = world.body_state.linear_velocity[id];
        angular_velocity_[index] = world.body_state.angular_velocity[id];
        sleeping_[index] = world.body_state.sleeping[id];
        UpdateDerivedBodyState(index);
    }

    void RefreshAuthoringForBody(const World& world, std::uint32_t i, ObjectId id) {
        motion_type_[i] = world.bodies.motion_type[id];
        scale_[i] = world.transforms.scale[id];
        center_of_mass_offset_local_[i] = world.bodies.center_of_mass_offset[id];
        friction_[i] = world.bodies.friction[id];
        linear_damping_[i] = world.bodies.linear_damping[id];
        angular_damping_[i] = world.bodies.angular_damping[id];
        use_gravity_[i] = world.bodies.use_gravity[id];

        const ColliderProxy proxy = BuildColliderProxyFromWorld(world, id);
        source_shape_[i] = proxy.source_shape;
        solver_shape_[i] = proxy.solver_shape;
        solver_enabled_[i] = (proxy.solver_shape != SolverShapeKind::None) ? 1 : 0;
        collider_local_offset_[i] = proxy.local_offset;
        collider_local_rotation_[i] = proxy.local_rotation;
        box_half_extents_local_[i] = proxy.box_half_extents_local;

        float effective_mass = world.bodies.mass[id];
        if (motion_type_[i] == MotionType::Static || motion_type_[i] == MotionType::Kinematic) {
            effective_mass = 0.0f;
        }
        if (effective_mass <= kEpsilon) {
            effective_mass = 0.0f;
        }

        mass_[i] = effective_mass;
        inverse_mass_[i] = (effective_mass > kEpsilon) ? (1.0f / effective_mass) : 0.0f;
        box_half_extents_world_[i] = HadamardAbsScale(box_half_extents_local_[i], scale_[i]);
        inverse_inertia_local_[i] = ComputeBoxInverseInertiaDiagonal(effective_mass, box_half_extents_world_[i]);
    }

    Vec3 WorldTransformToBodyPosition(const World& world, ObjectId id) const {
        const Vec3 transform_position = world.transforms.position[id];
        const Quat transform_rotation = world.transforms.rotation[id];
        const Vec3 com_offset = world.bodies.center_of_mass_offset[id];
        return transform_position + Rotate(transform_rotation, com_offset);
    }

    static Vec3 BodyPositionToWorldTransform(Vec3 body_position, Quat body_rotation, Vec3 com_offset) {
        return body_position - Rotate(body_rotation, com_offset);
    }

    Vec3 ApplyInverseInertiaWorld(std::uint32_t i, Vec3 value) const {
        const Quat inertia_world_rotation = rotation_[i] * collider_local_rotation_[i];
        const Vec3 axis_x = Rotate(inertia_world_rotation, Vec3{1.0f, 0.0f, 0.0f});
        const Vec3 axis_y = Rotate(inertia_world_rotation, Vec3{0.0f, 1.0f, 0.0f});
        const Vec3 axis_z = Rotate(inertia_world_rotation, Vec3{0.0f, 0.0f, 1.0f});
        return axis_x * (inverse_inertia_local_[i].x * Dot(value, axis_x))
             + axis_y * (inverse_inertia_local_[i].y * Dot(value, axis_y))
             + axis_z * (inverse_inertia_local_[i].z * Dot(value, axis_z));
    }

    Vec3 ComputePointVelocity(std::uint32_t i, Vec3 r) const {
        return linear_velocity_[i] + Cross(angular_velocity_[i], r);
    }

    float ComputeConstraintEffectiveMass(std::uint32_t a,
                                         std::uint32_t b,
                                         Vec3 r1,
                                         Vec3 r2,
                                         Vec3 axis) const {
        float k = inverse_mass_[a] + inverse_mass_[b];
        if (inverse_mass_[a] > 0.0f) {
            const Vec3 invI_raXn = ApplyInverseInertiaWorld(a, Cross(r1, axis));
            k += Dot(Cross(invI_raXn, r1), axis);
        }
        if (inverse_mass_[b] > 0.0f) {
            const Vec3 invI_rbXn = ApplyInverseInertiaWorld(b, Cross(r2, axis));
            k += Dot(Cross(invI_rbXn, r2), axis);
        }
        return (k > kEpsilon) ? (1.0f / k) : 0.0f;
    }

    void ApplyImpulsePair(std::uint32_t a, std::uint32_t b, Vec3 impulse, Vec3 r1, Vec3 r2) {
        if (inverse_mass_[a] > 0.0f) {
            linear_velocity_[a] -= impulse * inverse_mass_[a];
            angular_velocity_[a] -= ApplyInverseInertiaWorld(a, Cross(r1, impulse));
        }
        if (inverse_mass_[b] > 0.0f) {
            linear_velocity_[b] += impulse * inverse_mass_[b];
            angular_velocity_[b] += ApplyInverseInertiaWorld(b, Cross(r2, impulse));
        }
    }

    void ApplyPositionImpulsePair(std::uint32_t a, std::uint32_t b, Vec3 impulse, Vec3 r1, Vec3 r2) {
        if (inverse_mass_[a] > 0.0f) {
            position_[a] -= impulse * inverse_mass_[a];
            const Vec3 angular_step = -ApplyInverseInertiaWorld(a, Cross(r1, impulse));
            rotation_[a] = ApplyRotationDelta(rotation_[a], angular_step);
        }
        if (inverse_mass_[b] > 0.0f) {
            position_[b] += impulse * inverse_mass_[b];
            const Vec3 angular_step = ApplyInverseInertiaWorld(b, Cross(r2, impulse));
            rotation_[b] = ApplyRotationDelta(rotation_[b], angular_step);
        }
    }

    void UpdateDerivedBodyState(std::uint32_t i) {
        const Vec3 transform_origin = BodyPositionToWorldTransform(position_[i], rotation_[i], center_of_mass_offset_local_[i]);
        const Vec3 world_local_offset = Rotate(rotation_[i], collider_local_offset_[i]);
        collider_center_world_[i] = transform_origin + world_local_offset;
        box_half_extents_world_[i] = HadamardAbsScale(box_half_extents_local_[i], scale_[i]);

        const Quat collider_world_rotation = rotation_[i] * collider_local_rotation_[i];
        obb_proxy_[i] = MakeBoxWorldProxy(collider_center_world_[i], collider_world_rotation, box_half_extents_world_[i]);
        aabb_[i] = AabbFromObb(obb_proxy_[i]);
    }

    void UpdateAllDerivedBodyState() {
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(object_ids_.size()); ++i) {
            UpdateDerivedBodyState(i);
        }
    }

    void GatherExternalForces() {
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(object_ids_.size()); ++i) {
            force_accumulator_[i] = Vec3{0.0f, 0.0f, 0.0f};
            torque_accumulator_[i] = Vec3{0.0f, 0.0f, 0.0f};
        }
    }

    void IntegrateForces(float dt) {
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(object_ids_.size()); ++i) {
            if (!IsTrulyDynamic(i)) continue;
            if (sleeping_[i]) continue;

            const Vec3 gravity_acc = use_gravity_[i] ? config_.gravity : Vec3{0.0f, 0.0f, 0.0f};
            const Vec3 force_acc = force_accumulator_[i] * inverse_mass_[i];
            linear_velocity_[i] += (gravity_acc + force_acc) * dt;

            const Vec3 angular_acc = ApplyInverseInertiaWorld(i, torque_accumulator_[i]);
            angular_velocity_[i] += angular_acc * dt;

            const float ld = Clamp(1.0f - linear_damping_[i] * dt, 0.0f, 1.0f);
            const float ad = Clamp(1.0f - angular_damping_[i] * dt, 0.0f, 1.0f);
            linear_velocity_[i] *= ld;
            angular_velocity_[i] *= ad;
        }
    }

    void IntegrateVelocities(float dt) {
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(object_ids_.size()); ++i) {
            previous_position_[i] = position_[i];
            if (!IsTrulyDynamic(i)) continue;
            if (sleeping_[i]) continue;

            position_[i] += linear_velocity_[i] * dt;
            if (LengthSqNoNaN(angular_velocity_[i]) > kEpsilon * kEpsilon) {
                rotation_[i] = IntegrateRotationEuler(rotation_[i], angular_velocity_[i], dt);
            }
        }
    }

    void BuildBroadphasePairs() {
        const std::uint32_t count = static_cast<std::uint32_t>(object_ids_.size());
        for (std::uint32_t a = 0; a < count; ++a) {
            if (!solver_enabled_[a]) continue;
            for (std::uint32_t b = a + 1; b < count; ++b) {
                if (!solver_enabled_[b]) continue;

                const bool a_dynamic = IsTrulyDynamic(a);
                const bool b_dynamic = IsTrulyDynamic(b);
                if (!a_dynamic && !b_dynamic) continue;

                const bool a_active = a_dynamic && !sleeping_[a];
                const bool b_active = b_dynamic && !sleeping_[b];
                if (!a_active && !b_active) continue;

                if (!TestAabbOverlap(aabb_[a], aabb_[b])) continue;
                broadphase_pairs_.push_back(BroadphasePair{a, b});
            }
        }
    }

    void InitializeContact(Contact& c, float dt) {
        c.r1 = c.point - position_[c.a];
        c.r2 = c.point - position_[c.b];

        const ContactTangentBasis basis = MakeTangentBasis(c.normal);
        c.tangent1 = basis.tangent1;
        c.tangent2 = basis.tangent2;

        c.combined_friction = std::sqrt(std::max(0.0f, friction_[c.a]) * std::max(0.0f, friction_[c.b]));
        c.restitution = 0.0f;

        const Vec3 va = ComputePointVelocity(c.a, c.r1);
        const Vec3 vb = ComputePointVelocity(c.b, c.r2);
        const float normal_speed = Dot(vb - va, c.normal);
        const bool nearly_resting = (std::fabs(normal_speed) < kRestingBiasVelocityThreshold)
                                 && (c.penetration < kRestingBiasPenetration);
        c.bias = (dt > 0.0f && !nearly_resting)
               ? (kVelocityBaumgarte / dt) * std::max(c.penetration - kPenetrationSlop, 0.0f)
               : 0.0f;

        c.normal_effective_mass = ComputeConstraintEffectiveMass(c.a, c.b, c.r1, c.r2, c.normal);
        c.tangent1_effective_mass = ComputeConstraintEffectiveMass(c.a, c.b, c.r1, c.r2, c.tangent1);
        c.tangent2_effective_mass = ComputeConstraintEffectiveMass(c.a, c.b, c.r1, c.r2, c.tangent2);
        c.normal_lambda = 0.0f;
        c.tangent1_lambda = 0.0f;
        c.tangent2_lambda = 0.0f;
    }

    float ComputeWarmStartRatio(float dt) const {
        if (previous_step_dt_ <= kEpsilon || dt <= kEpsilon) {
            return 1.0f;
        }
        return Clamp(dt / previous_step_dt_, 0.5f, 2.0f);
    }

    bool SeedWarmStartFromCache(std::uint64_t pair_key,
                                float warm_start_ratio,
                                std::uint8_t used_mask[4],
                                Contact& c) const {
        const auto it = previous_contact_cache_.find(pair_key);
        if (it == previous_contact_cache_.end()) {
            return false;
        }

        const CachedManifold& cached = it->second;
        if (cached.contact_count == 0 || Dot(cached.normal, c.normal) < kWarmStartNormalDotMin) {
            return false;
        }

        int best_index = -1;
        float best_dist_sq = kWarmStartPointMatchDistanceSq;
        for (std::uint32_t i = 0; i < cached.contact_count; ++i) {
            if (used_mask[i]) {
                continue;
            }
            const float dist_sq = LengthSqNoNaN(c.point - cached.points[i].point);
            if (dist_sq <= best_dist_sq) {
                best_dist_sq = dist_sq;
                best_index = static_cast<int>(i);
            }
        }

        if (best_index < 0) {
            return false;
        }

        used_mask[best_index] = 1;
        c.normal_lambda = std::max(0.0f, cached.points[best_index].normal_lambda * warm_start_ratio);
        c.tangent1_lambda = 0.0f;
        c.tangent2_lambda = 0.0f;

        const float max_friction = c.combined_friction * c.normal_lambda;
        const float tangent_len_sq = c.tangent1_lambda * c.tangent1_lambda + c.tangent2_lambda * c.tangent2_lambda;
        if (tangent_len_sq > max_friction * max_friction && tangent_len_sq > kEpsilon * kEpsilon) {
            const float scale = max_friction / std::sqrt(tangent_len_sq);
            c.tangent1_lambda *= scale;
            c.tangent2_lambda *= scale;
        }
        return true;
    }

    void WarmStartContacts() {
        for (Contact& c : contacts_) {
            const Vec3 impulse = c.normal * c.normal_lambda
                               + c.tangent1 * c.tangent1_lambda
                               + c.tangent2 * c.tangent2_lambda;
            if (LengthSqNoNaN(impulse) <= 0.0f) {
                continue;
            }
            ApplyImpulsePair(c.a, c.b, impulse, c.r1, c.r2);
        }
    }

    bool ShouldWakePair(std::uint32_t a,
                        std::uint32_t b,
                        Vec3 point,
                        Vec3 normal,
                        float penetration,
                        bool had_cache_match) const {
        const bool a_sleeping_dynamic = IsTrulyDynamic(a) && sleeping_[a];
        const bool b_sleeping_dynamic = IsTrulyDynamic(b) && sleeping_[b];
        if (!a_sleeping_dynamic && !b_sleeping_dynamic) {
            return false;
        }

        const Vec3 r1 = point - position_[a];
        const Vec3 r2 = point - position_[b];
        const Vec3 rv = ComputePointVelocity(b, r2) - ComputePointVelocity(a, r1);
        const float normal_speed = std::fabs(Dot(rv, normal));
        const Vec3 tangent_velocity = rv - normal * Dot(rv, normal);
        const float tangent_speed = std::sqrt(std::max(0.0f, LengthSqNoNaN(tangent_velocity)));

        if (!had_cache_match) {
            return penetration > kWakePenetrationThreshold
                || normal_speed > kWakeNormalSpeedThreshold
                || tangent_speed > kWakeTangentSpeedThreshold;
        }

        return normal_speed > (2.0f * kWakeNormalSpeedThreshold)
            || tangent_speed > (2.0f * kWakeTangentSpeedThreshold);
    }

    void WakeBody(std::uint32_t i) {
        if (!IsTrulyDynamic(i)) {
            return;
        }
        sleeping_[i] = 0;
        sleep_counter_[i] = 0;
    }

    void UpdateContactCache() {
        next_contact_cache_.clear();
        for (const ContactManifold& manifold : manifolds_) {
            CachedManifold cached{};
            cached.normal = manifold.normal;
            cached.contact_count = std::min<std::uint32_t>(manifold.contact_count, 4u);
            for (std::uint32_t i = 0; i < cached.contact_count; ++i) {
                const Contact& c = contacts_[manifold.first_contact + i];
                cached.points[i].point = c.point;
                cached.points[i].normal_lambda = c.normal_lambda;
                cached.points[i].tangent1_lambda = c.tangent1_lambda;
                cached.points[i].tangent2_lambda = c.tangent2_lambda;
            }
            next_contact_cache_[MakePairKey(manifold.a, manifold.b)] = cached;
        }
        previous_contact_cache_.swap(next_contact_cache_);
    }

    void GenerateContacts(float dt) {
        contacts_.reserve(broadphase_pairs_.size() * 4u);
        manifolds_.reserve(broadphase_pairs_.size());

        const float warm_start_ratio = ComputeWarmStartRatio(dt);

        for (const BroadphasePair& pair : broadphase_pairs_) {
            const std::uint32_t a = pair.a;
            const std::uint32_t b = pair.b;
            if (solver_shape_[a] != SolverShapeKind::Box || solver_shape_[b] != SolverShapeKind::Box) {
                continue;
            }

            const BoxBoxSatResult sat = BoxBoxSat(obb_proxy_[a], obb_proxy_[b]);
            if (sat.separated) {
                continue;
            }

            std::vector<ContactPointCandidate> candidates;
            if (sat.axis_index >= 0 && sat.axis_index < 6) {
                candidates = BuildFaceManifoldCandidates(obb_proxy_[a], obb_proxy_[b], sat);
            } else {
                candidates.push_back(BuildEdgeEdgeCandidate(obb_proxy_[a], obb_proxy_[b], sat));
            }

            if (candidates.empty()) {
                continue;
            }

            ContactManifold manifold{};
            manifold.a = a;
            manifold.b = b;
            manifold.normal = sat.normal;
            manifold.penetration = sat.penetration;
            manifold.first_contact = static_cast<std::uint32_t>(contacts_.size());
            manifold.contact_count = 0;
            Vec3 representative_sum{0.0f, 0.0f, 0.0f};
            bool had_cache_match = false;
            std::uint8_t used_cache_points[4]{0, 0, 0, 0};
            const std::uint64_t pair_key = MakePairKey(a, b);

            for (const ContactPointCandidate& candidate : candidates) {
                Contact c{};
                c.a = a;
                c.b = b;
                c.normal = sat.normal;
                c.point = candidate.point;
                c.penetration = std::max(0.0f, candidate.penetration);
                InitializeContact(c, dt);
                had_cache_match |= SeedWarmStartFromCache(pair_key, warm_start_ratio, used_cache_points, c);
                representative_sum += c.point;
                manifold.penetration = std::max(manifold.penetration, c.penetration);
                ++manifold.contact_count;
                contacts_.push_back(c);
            }

            if (manifold.contact_count > 0) {
                manifold.representative_point = representative_sum / static_cast<float>(manifold.contact_count);
                manifolds_.push_back(manifold);
                if (ShouldWakePair(a, b, manifold.representative_point, manifold.normal, manifold.penetration, had_cache_match)) {
                    WakeBody(a);
                    WakeBody(b);
                }
            }
        }
    }

    void SolveVelocityConstraints() {
        for (Contact& c : contacts_) {
            const Vec3 va = ComputePointVelocity(c.a, c.r1);
            const Vec3 vb = ComputePointVelocity(c.b, c.r2);
            const Vec3 rv = vb - va;
            const float normal_speed = Dot(rv, c.normal);

            const float restitution_bias = (normal_speed < -kRestitutionThreshold) ? (-c.restitution * normal_speed) : 0.0f;
            const float delta_n = c.normal_effective_mass * (c.bias + restitution_bias - normal_speed);
            const float old_n = c.normal_lambda;
            c.normal_lambda = std::max(0.0f, old_n + delta_n);
            const float applied_n = c.normal_lambda - old_n;
            if (applied_n != 0.0f) {
                ApplyImpulsePair(c.a, c.b, c.normal * applied_n, c.r1, c.r2);
            }

            const Vec3 va_after_n = ComputePointVelocity(c.a, c.r1);
            const Vec3 vb_after_n = ComputePointVelocity(c.b, c.r2);
            const Vec3 rv_after_n = vb_after_n - va_after_n;
            const float jt1 = -Dot(rv_after_n, c.tangent1) * c.tangent1_effective_mass;
            const float jt2 = -Dot(rv_after_n, c.tangent2) * c.tangent2_effective_mass;

            const float old_t1 = c.tangent1_lambda;
            const float old_t2 = c.tangent2_lambda;
            float new_t1 = old_t1 + jt1;
            float new_t2 = old_t2 + jt2;

            const float max_friction = c.combined_friction * c.normal_lambda;
            const float tangent_len_sq = new_t1 * new_t1 + new_t2 * new_t2;
            if (tangent_len_sq > max_friction * max_friction && tangent_len_sq > kEpsilon * kEpsilon) {
                const float scale = max_friction / std::sqrt(tangent_len_sq);
                new_t1 *= scale;
                new_t2 *= scale;
            }

            const float applied_t1 = new_t1 - old_t1;
            const float applied_t2 = new_t2 - old_t2;
            c.tangent1_lambda = new_t1;
            c.tangent2_lambda = new_t2;

            const Vec3 tangent_impulse = c.tangent1 * applied_t1 + c.tangent2 * applied_t2;
            if (LengthSqNoNaN(tangent_impulse) > 0.0f) {
                ApplyImpulsePair(c.a, c.b, tangent_impulse, c.r1, c.r2);
            }
        }
    }

    void SolvePositionConstraints() {
        for (ContactManifold& manifold : manifolds_) {
            const std::uint32_t a = manifold.a;
            const std::uint32_t b = manifold.b;
            if (!solver_enabled_[a] || !solver_enabled_[b]) {
                continue;
            }

            const float fresh_pen = RemeasurePenetration(obb_proxy_[a], obb_proxy_[b], manifold.normal);
            const float depth = std::max(fresh_pen - kPenetrationSlop, 0.0f);
            if (depth <= 0.0f) {
                continue;
            }

            const Vec3 point = manifold.representative_point;
            const Vec3 r1 = point - position_[a];
            const Vec3 r2 = point - position_[b];
            const float effective_mass = ComputeConstraintEffectiveMass(a, b, r1, r2, manifold.normal);
            if (effective_mass <= 0.0f) {
                continue;
            }

            const float correction_distance = std::min(kPositionBaumgarte * depth, kMaxPositionCorrection);
            const float pseudo_lambda = correction_distance * effective_mass;
            ApplyPositionImpulsePair(a, b, manifold.normal * pseudo_lambda, r1, r2);

            UpdateDerivedBodyState(a);
            UpdateDerivedBodyState(b);
        }
    }

    void UpdateSleepState() {
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(object_ids_.size()); ++i) {
            if (!IsTrulyDynamic(i)) {
                sleeping_[i] = (motion_type_[i] == MotionType::Static) ? 1 : 0;
                sleep_counter_[i] = 0;
                continue;
            }

            const float lv_sq = LengthSqNoNaN(linear_velocity_[i]);
            const float av_sq = LengthSqNoNaN(angular_velocity_[i]);
            const bool can_sleep = (lv_sq <= kSleepLinearThresholdSq && av_sq <= kSleepAngularThresholdSq);

            if (!can_sleep) {
                sleeping_[i] = 0;
                sleep_counter_[i] = 0;
                continue;
            }

            if (sleep_counter_[i] < kSleepTicksRequired) {
                ++sleep_counter_[i];
            }
            if (sleep_counter_[i] >= kSleepTicksRequired) {
                sleeping_[i] = 1;
                linear_velocity_[i] = Vec3{0.0f, 0.0f, 0.0f};
                angular_velocity_[i] = Vec3{0.0f, 0.0f, 0.0f};
            }
        }
    }

    void ClearAccumulators() {
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(object_ids_.size()); ++i) {
            force_accumulator_[i] = Vec3{0.0f, 0.0f, 0.0f};
            torque_accumulator_[i] = Vec3{0.0f, 0.0f, 0.0f};
        }
    }

    bool IsTrulyDynamic(std::uint32_t i) const {
        return motion_type_[i] == MotionType::Dynamic && inverse_mass_[i] > 0.0f;
    }

private:
    PhysicsConfig config_{};
    double simulation_time_ = 0.0;

    std::vector<ObjectId> object_ids_;
    std::unordered_map<ObjectId, std::uint32_t> object_to_index_;

    std::vector<MotionType> motion_type_;
    std::vector<ShapeKind> source_shape_;
    std::vector<SolverShapeKind> solver_shape_;
    std::vector<std::uint8_t> solver_enabled_;

    // position_ is the COM world position.
    std::vector<Vec3> position_;
    std::vector<Quat> rotation_;
    std::vector<Vec3> scale_;
    std::vector<Vec3> linear_velocity_;
    std::vector<Vec3> angular_velocity_;
    std::vector<Vec3> previous_position_;

    std::vector<float> mass_;
    std::vector<float> inverse_mass_;
    std::vector<Vec3> inverse_inertia_local_;

    std::vector<std::uint8_t> use_gravity_;
    std::vector<float> friction_;
    std::vector<float> linear_damping_;
    std::vector<float> angular_damping_;
    std::vector<std::uint8_t> sleeping_;
    std::vector<std::uint32_t> sleep_counter_;

    std::vector<Vec3> center_of_mass_offset_local_;
    std::vector<Vec3> collider_local_offset_;
    std::vector<Quat> collider_local_rotation_;
    std::vector<Vec3> box_half_extents_local_;
    std::vector<Vec3> box_half_extents_world_;
    std::vector<Vec3> collider_center_world_;
    std::vector<Aabb> aabb_;
    std::vector<BoxWorldProxy> obb_proxy_;

    std::vector<Vec3> force_accumulator_;
    std::vector<Vec3> torque_accumulator_;

    std::vector<BroadphasePair> broadphase_pairs_;
    std::vector<ContactManifold> manifolds_;
    std::vector<Contact> contacts_;

    std::unordered_map<std::uint64_t, CachedManifold> previous_contact_cache_;
    std::unordered_map<std::uint64_t, CachedManifold> next_contact_cache_;
    float previous_step_dt_ = 0.0f;
};

} // namespace

std::unique_ptr<IPhysicsBackend> CreateNullPhysicsBackendV3_1() {
    return std::make_unique<NullPhysicsBackend>();
}

} // namespace lab

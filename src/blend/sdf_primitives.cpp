#include <exd/geometry/blend.hpp>

#include <algorithm>
#include <cmath>

namespace exd::geometry
{

// ============================================================================
// Internal SDF functions — operate in local space (after inverse transform)
// ============================================================================

/// Signed distance to a sphere centered at origin.
static float sdf_sphere(const math::Vec3f& p, float r)
{
    return p.length() - r;
}

/// Signed distance to an axis-aligned box centered at origin.
static float sdf_box(const math::Vec3f& p, const math::Vec3f& halfExt)
{
    math::Vec3f q = {
        std::abs(p.x) - halfExt.x,
        std::abs(p.y) - halfExt.y,
        std::abs(p.z) - halfExt.z
    };
    math::Vec3f clamped = {
        std::max(q.x, 0.0f),
        std::max(q.y, 0.0f),
        std::max(q.z, 0.0f)
    };
    return clamped.length() + std::min(std::max({q.x, q.y, q.z}), 0.0f);
}

/// Signed distance to a capsule along Y axis from -h/2 to +h/2.
static float sdf_capsule_local(const math::Vec3f& p, float r, float h)
{
    float halfH = h * 0.5f;
    math::Vec3f a = {0.0f, -halfH, 0.0f};
    math::Vec3f b = {0.0f,  halfH, 0.0f};

    math::Vec3f pa = p - a;
    math::Vec3f ba = b - a;
    float h_clamp = std::clamp(pa.dot(ba) / ba.dot(ba), 0.0f, 1.0f);
    return (pa - ba * h_clamp).length() - r;
}

/// Signed distance to a capped cylinder along Y axis from -h/2 to +h/2.
static float sdf_cylinder_local(const math::Vec3f& p, float r, float h)
{
    float halfH = h * 0.5f;
    math::Vec3f d = {
        std::sqrt(p.x * p.x + p.z * p.z) - r,
        std::abs(p.y) - halfH,
        0.0f
    };
    math::Vec3f clamped = {
        std::max(d.x, 0.0f),
        std::max(d.y, 0.0f),
        0.0f
    };
    return clamped.length() + std::min(std::max(d.x, d.y), 0.0f);
}

/// Signed distance to a cone along Y axis from -h/2 (base) to +h/2 (tip).
static float sdf_cone_local(const math::Vec3f& p, float r, float h)
{
    float halfH = h * 0.5f;

    // Cone: radius at height y is r * (halfH - y) / h
    // At y = -halfH: radius = r (base)
    // At y = +halfH: radius = 0 (tip)

    float qx = std::sqrt(p.x * p.x + p.z * p.z);
    float qy = p.y + halfH; // shift so base is at y=0, tip at y=h

    // Cone angle: tan(angle) = r / h
    // Cone SDF approximation for capped cone:
    float c = r / h;
    float coneDist = qx - r + c * qy;

    // Cap at base (y = 0 → original y = -halfH)
    float baseDist = -qy;

    // Cap at tip
    float tipDist = qy - h;

    // Clamp the cone distance by the caps
    return std::max({coneDist, baseDist, tipDist});
}

/// Signed distance to a torus in XZ plane, centered at origin.
static float sdf_torus_local(const math::Vec3f& p, float R, float r)
{
    float qx = std::sqrt(p.x * p.x + p.z * p.z) - R;
    return std::sqrt(qx * qx + p.y * p.y) - r;
}

// ============================================================================
// Transform helpers
// ============================================================================

/// Apply inverse rotation to a point (using quaternion conjugate).
static math::Vec3f inv_rotate(const math::Vec3f& p, const math::Quat& q)
{
    // Conjugate performs inverse rotation for unit quaternions
    math::Quat conj{q.w, -q.x, -q.y, -q.z};
    return conj * p;  // Quat * Vec3f operator rotates the vector
}

/// Apply inverse scale to a point.
static math::Vec3f inv_scale(const math::Vec3f& p, const math::Vec3f& s)
{
    return {p.x / s.x, p.y / s.y, p.z / s.z};
}

// ============================================================================
// Public SDF evaluation entry point
// ============================================================================

/// Evaluate the signed distance from a world-space point to a primitive.
/// Negative = inside the shape, positive = outside.
float evaluate_primitive_sdf(const math::Vec3f& worldPoint,
                             const BlendPrimitive& prim)
{
    // Transform world point into primitive local space
    math::Vec3f local = worldPoint - prim.position;
    local = inv_rotate(local, prim.rotation);
    local = inv_scale(local, prim.scale);

    // Evaluate SDF in local space
    switch (prim.kind)
    {
    case BlendPrimitiveKind::Sphere:
        return sdf_sphere(local, prim.radius);

    case BlendPrimitiveKind::Box:
        return sdf_box(local, {prim.halfExtent, prim.halfExtent, prim.halfExtent});

    case BlendPrimitiveKind::Capsule:
        return sdf_capsule_local(local, prim.radius, prim.height);

    case BlendPrimitiveKind::Cylinder:
        return sdf_cylinder_local(local, prim.radius, prim.height);

    case BlendPrimitiveKind::Cone:
        return sdf_cone_local(local, prim.radius, prim.height);

    case BlendPrimitiveKind::Torus:
        return sdf_torus_local(local, prim.radius2, prim.radius);
    }

    return 1e10f; // unreachable
}

} // namespace exd::geometry

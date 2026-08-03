#include <exd/geometry/deform.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <cmath>

namespace exd::geometry
{

// Simple hash-based pseudo-random (for noise seed)
static float hash_noise(float x, float y, float z, uint32_t seed)
{
    uint32_t h = seed;
    h = h * 16777619u ^ *reinterpret_cast<const uint32_t*>(&x);
    h = h * 16777619u ^ *reinterpret_cast<const uint32_t*>(&y);
    h = h * 16777619u ^ *reinterpret_cast<const uint32_t*>(&z);
    return static_cast<float>(h & 0x7FFFFFFF) / 2147483648.0f;
}

MeshData deform_mesh(const MeshData& mesh, const DeformDescriptor& desc)
{
    MeshData result;
    result.topology = mesh.topology;
    result.indices = mesh.indices;
    result.vertices.reserve(mesh.vertices.size());

    // Compute the center and extent for axis-normalization
    Bounds b = compute_bounds(mesh.vertices);
    math::Vec3f center = {(b.min.x + b.max.x) * 0.5f,
                           (b.min.y + b.max.y) * 0.5f,
                           (b.min.z + b.max.z) * 0.5f};
    math::Vec3f extent = {(b.max.x - b.min.x) * 0.5f,
                           (b.max.y - b.min.y) * 0.5f,
                           (b.max.z - b.min.z) * 0.5f};
    float maxExtent = std::max({extent.x, extent.y, extent.z, 0.001f});

    for (const auto& v : mesh.vertices) {
        Vertex dv = v;
        math::Vec3f p = v.position;

        // ── Bend ──
        if (desc.bend && std::abs(desc.bendAngle) > 1e-6f) {
            // Project onto bend axis to get the coordinate along the axis
            math::Vec3f ax = desc.bendAxis;
            float axLen = ax.length();
            if (axLen < 1e-8f) ax = {0, 1, 0};
            else ax = ax / axLen;

            float t = p.dot(ax) / maxExtent; // normalized [-1, 1]
            float radius = desc.bendRadius;
            float angle = t * desc.bendAngle;
            float ca = std::cos(angle), sa = std::sin(angle);

            // Radial distance from axis
            math::Vec3f radial = p - ax * p.dot(ax);
            float radDist = radial.length();

            // Bend: rotate radial vector around perpendicular axis
            math::Vec3f rotAxis = ax.cross(radial);
            if (rotAxis.length() < 1e-8f) rotAxis = {0, 0, 1};
            rotAxis = rotAxis / rotAxis.length();

            math::Vec3f bent = radial * ca + rotAxis.cross(radial) * sa;
            p = ax * (t * maxExtent) + bent * (radius / maxExtent);
        }

        // ── Twist ──
        if (desc.twist && std::abs(desc.twistAngle) > 1e-6f) {
            math::Vec3f ax = desc.twistAxis;
            float axLen = ax.length();
            if (axLen < 1e-8f) ax = {0, 1, 0};
            else ax = ax / axLen;

            float t = (p - center).dot(ax) / (maxExtent * 2.0f) + 0.5f; // [0, 1]
            float angle = t * desc.twistAngle;
            float ca = std::cos(angle), sa = std::sin(angle);

            // Rotate around twist axis
            math::Vec3f radial = p - ax * p.dot(ax);
            math::Vec3f rotated = radial * ca + ax.cross(radial) * sa;
            p = ax * p.dot(ax) + rotated;
        }

        // ── Taper ──
        if (desc.taper) {
            math::Vec3f ax = desc.taperAxis;
            float axLen = ax.length();
            if (axLen < 1e-8f) ax = {0, 1, 0};
            else ax = ax / axLen;

            float t = (p - center).dot(ax) / (maxExtent * 2.0f) + 0.5f; // [0, 1]
            float s = desc.taperStart + (desc.taperEnd - desc.taperStart) * t;
            math::Vec3f radial = p - ax * p.dot(ax);
            p = ax * p.dot(ax) + radial * s;
        }

        // ── Noise ──
        if (desc.noise && desc.noiseAmplitude > 0.0f) {
            float nx = p.x * desc.noiseFrequency;
            float ny = p.y * desc.noiseFrequency;
            float nz = p.z * desc.noiseFrequency;
            float n = hash_noise(nx, ny, nz, desc.noiseSeed);
            // Use gradient of noise to displace along normal
            math::Vec3f displacement = v.normal * n * desc.noiseAmplitude;
            p = p + displacement;
        }

        dv.position = p;
        result.vertices.push_back(dv);
    }

    result.bounds = compute_bounds(result.vertices);
    return result;
}

} // namespace exd::geometry

#include <exd/geometry/primitives3d.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace exd::geometry
{

MeshData generate_arrow3d_mesh(const Arrow3DGeometry& geometry)
{
    constexpr float pi = 3.14159265358979323846f;

    const math::Vec3f dir = geometry.end - geometry.start;
    const float totalLen  = dir.length();

    // Edge case: zero-length arrow
    if (totalLen < 1e-6f)
        return {};

    // Edge case: invalid radii
    if (geometry.shaftRadius <= 0.0f && geometry.headRadius <= 0.0f)
        return {};

    const math::Vec3f forward = dir / totalLen;
    const math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

    // Build local frame
    math::Vec3f ref = {0.0f, 1.0f, 0.0f};
    if (std::abs(forward.dot(ref)) > 0.999f)
        ref = {0.0f, 0.0f, 1.0f};

    const math::Vec3f right = ref.cross(forward).normalized();
    const math::Vec3f up    = forward.cross(right);

    // Clamp headLength to total length
    const float headLen  = std::min(geometry.headLength, totalLen);
    const float shaftLen = totalLen - headLen;

    // Key positions
    const math::Vec3f shaftEnd = geometry.start + forward * shaftLen;

    uint32_t slices       = geometry.slices < 3 ? 3 : geometry.slices;
    uint32_t vertsPerRing = slices + 1;

    // ── Build vertices and indices directly ──
    const bool hasShaft = (shaftLen > 1e-6f && geometry.shaftRadius > 0.0f);
    const bool hasHead  = (headLen > 1e-6f && geometry.headRadius > 0.0f);

    size_t shaftVerts = hasShaft ? 2 * vertsPerRing : 0;
    size_t headVerts  = hasHead  ? vertsPerRing + 1  : 0;
    size_t shaftIdxs  = hasShaft ? slices * 6 : 0;
    size_t headIdxs   = hasHead  ? slices * 3 : 0;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(shaftVerts + headVerts);
    indices.reserve(shaftIdxs + headIdxs);

    uint32_t nextVert = 0;

    // ── Shaft: cylinder from start to shaftEnd ──
    if (hasShaft)
    {
        const float shaftRadius = geometry.shaftRadius;

        // Bottom ring (at start)
        for (uint32_t i = 0; i <= slices; ++i)
        {
            float alpha     = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
            float cos_alpha = std::cos(alpha);
            float sin_alpha = std::sin(alpha);

            math::Vec3f radial = right * cos_alpha + up * sin_alpha;

            Vertex v;
            v.position = geometry.start + radial * shaftRadius;
            v.normal   = radial;
            v.uv       = {static_cast<float>(i) / static_cast<float>(slices), 0.0f, 0.0f};
            v.tangent  = tangent_id;

            vertices.push_back(v);
        }

        // Top ring (at shaftEnd)
        for (uint32_t i = 0; i <= slices; ++i)
        {
            float alpha     = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
            float cos_alpha = std::cos(alpha);
            float sin_alpha = std::sin(alpha);

            math::Vec3f radial = right * cos_alpha + up * sin_alpha;

            Vertex v;
            v.position = shaftEnd + radial * shaftRadius;
            v.normal   = radial;
            v.uv       = {static_cast<float>(i) / static_cast<float>(slices), shaftLen / totalLen, 0.0f};
            v.tangent  = tangent_id;

            vertices.push_back(v);
        }

        // Shaft indices: quads between bottom and top rings
        for (uint32_t i = 0; i < slices; ++i)
        {
            uint32_t b0 = i;
            uint32_t b1 = i + 1;
            uint32_t t0 = vertsPerRing + i;
            uint32_t t1 = t0 + 1;

            indices.push_back(b0);
            indices.push_back(t0);
            indices.push_back(b1);

            indices.push_back(b1);
            indices.push_back(t0);
            indices.push_back(t1);
        }

        nextVert = static_cast<uint32_t>(vertices.size());
    }

    // ── Head: cone from shaftEnd to tip ──
    if (hasHead)
    {
        const float headRadius = geometry.headRadius;
        const uint32_t headRingBase = nextVert;

        // Base ring at shaftEnd
        for (uint32_t i = 0; i <= slices; ++i)
        {
            float alpha     = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
            float cos_alpha = std::cos(alpha);
            float sin_alpha = std::sin(alpha);

            math::Vec3f radial = right * cos_alpha + up * sin_alpha;

            Vertex v;
            v.position = shaftEnd + radial * headRadius;
            // Cone normal: points outward from cone surface
            v.normal   = (radial * headLen + forward * headRadius).normalized();
            v.uv       = {static_cast<float>(i) / static_cast<float>(slices), shaftLen / totalLen, 0.0f};
            v.tangent  = tangent_id;

            vertices.push_back(v);
        }

        // Tip vertex
        uint32_t tipIdx = nextVert + vertsPerRing;
        {
            Vertex v;
            v.position = geometry.end;
            v.normal   = forward;
            v.uv       = {0.5f, 1.0f, 0.0f};
            v.tangent  = tangent_id;
            vertices.push_back(v);
        }

        // Head indices: triangles from base ring to tip
        for (uint32_t i = 0; i < slices; ++i)
        {
            uint32_t curr = headRingBase + i;
            uint32_t next = curr + 1;

            indices.push_back(curr);
            indices.push_back(next);
            indices.push_back(tipIdx);
        }
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = compute_bounds(mesh.vertices);

    return mesh;
}

} // namespace exd::geometry

#include <exd/geometry/primitives3d.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace exd::geometry
{

MeshData generate_axes_mesh(const AxesGeometry& geometry)
{
    constexpr float pi = 3.14159265358979323846f;

    if (geometry.length < 1e-6f)
        return {};

    const uint32_t slices       = 10;
    const uint32_t vertsPerRing = slices + 1;
    const math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

    const float actualHeadLen  = std::min(geometry.headLength, geometry.length);
    const float actualShaftLen = geometry.length - actualHeadLen;

    const bool hasShaft = (actualShaftLen > 1e-6f && geometry.shaftRadius > 0.0f);
    const bool hasHead  = (actualHeadLen > 0.0f && geometry.headRadius > 0.0f);

    if (!hasShaft && !hasHead)
        return {};

    // Per-axis vertex/index counts
    size_t axisShaftVerts = hasShaft ? 2 * vertsPerRing : 0;
    size_t axisHeadVerts  = hasHead  ? vertsPerRing + 1  : 0;
    size_t axisShaftIdxs  = hasShaft ? slices * 6 : 0;
    size_t axisHeadIdxs   = hasHead  ? slices * 3 : 0;
    size_t axisVerts      = axisShaftVerts + axisHeadVerts;
    size_t axisIdxs       = axisShaftIdxs + axisHeadIdxs;

    // Three axes
    size_t totalVerts = axisVerts * 3;
    size_t totalIdxs  = axisIdxs * 3;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(totalVerts);
    indices.reserve(totalIdxs);

    struct AxisDef
    {
        math::Vec3f direction;
        math::Quat  color;
    };

    const AxisDef axes[3] = {
        {{1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 1.0f}}, // X: red
        {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 1.0f}}, // Y: green
        {{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f}}, // Z: blue
    };

    // Build each axis
    uint32_t vertOffset = 0;

    for (const auto& axis : axes)
    {
        const math::Vec3f& dir      = axis.direction;
        const math::Quat&  col      = axis.color;
        const float        len      = geometry.length;
        const float        shaftR   = geometry.shaftRadius;
        const float        headR    = geometry.headRadius;
        const float        headLen  = actualHeadLen;
        const float        shaftLen = actualShaftLen;
        const math::Vec3f  shaftEnd = dir * shaftLen;

        // Build local frame
        math::Vec3f ref = {0.0f, 1.0f, 0.0f};
        if (std::abs(dir.dot(ref)) > 0.999f)
            ref = {0.0f, 0.0f, 1.0f};

        const math::Vec3f right = ref.cross(dir).normalized();
        const math::Vec3f up    = dir.cross(right);

        uint32_t ring0Base = vertOffset;

        // ── Shaft ──
        if (hasShaft)
        {
            // Bottom ring (at origin)
            for (uint32_t i = 0; i <= slices; ++i)
            {
                float alpha     = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
                float cos_alpha = std::cos(alpha);
                float sin_alpha = std::sin(alpha);

                math::Vec3f radial = right * cos_alpha + up * sin_alpha;

                Vertex v;
                v.position = radial * shaftR;
                v.normal   = radial;
                v.uv       = {static_cast<float>(i) / static_cast<float>(slices), 0.0f, 0.0f};
                v.tangent  = tangent_id;
                v.color    = col;

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
                v.position = shaftEnd + radial * shaftR;
                v.normal   = radial;
                v.uv       = {static_cast<float>(i) / static_cast<float>(slices), shaftLen / len, 0.0f};
                v.tangent  = tangent_id;
                v.color    = col;

                vertices.push_back(v);
            }

            // Shaft indices
            for (uint32_t i = 0; i < slices; ++i)
            {
                uint32_t b0 = ring0Base + i;
                uint32_t b1 = b0 + 1;
                uint32_t t0 = ring0Base + vertsPerRing + i;
                uint32_t t1 = t0 + 1;

                indices.push_back(b0);
                indices.push_back(t0);
                indices.push_back(b1);

                indices.push_back(b1);
                indices.push_back(t0);
                indices.push_back(t1);
            }

            vertOffset += static_cast<uint32_t>(2 * vertsPerRing);
        }

        // ── Head ──
        if (hasHead)
        {
            const uint32_t headRingBase = vertOffset;

            // Base ring at shaftEnd
            for (uint32_t i = 0; i <= slices; ++i)
            {
                float alpha     = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
                float cos_alpha = std::cos(alpha);
                float sin_alpha = std::sin(alpha);

                math::Vec3f radial = right * cos_alpha + up * sin_alpha;

                Vertex v;
                v.position = shaftEnd + radial * headR;
                v.normal   = (radial * headLen + dir * headR).normalized();
                v.uv       = {static_cast<float>(i) / static_cast<float>(slices), shaftLen / len, 0.0f};
                v.tangent  = tangent_id;
                v.color    = col;

                vertices.push_back(v);
            }

            // Tip vertex
            uint32_t tipIdx = vertOffset + vertsPerRing;
            {
                Vertex v;
                v.position = dir * len;
                v.normal   = dir;
                v.uv       = {0.5f, 1.0f, 0.0f};
                v.tangent  = tangent_id;
                v.color    = col;
                vertices.push_back(v);
            }

            // Head indices
            for (uint32_t i = 0; i < slices; ++i)
            {
                uint32_t curr = headRingBase + i;
                uint32_t next = curr + 1;

                indices.push_back(curr);
                indices.push_back(next);
                indices.push_back(tipIdx);
            }

            vertOffset += static_cast<uint32_t>(vertsPerRing + 1);
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

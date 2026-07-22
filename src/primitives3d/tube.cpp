#include <exd/geometry/primitives3d.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace exd::geometry
{

MeshData generate_tube_mesh(const TubeGeometry& geometry)
{
    constexpr float pi = 3.14159265358979323846f;

    // ── Edge cases ──
    if (geometry.path.size() < 2)
        return {};
    if (geometry.radius <= 0.0f)
        return {};

    uint32_t radialSegments = geometry.radialSegments < 3 ? 3 : geometry.radialSegments;
    const float radius = geometry.radius;

    // ── Build path frames (tangent, normal, binormal) ──
    const size_t n = geometry.path.size();
    struct Frame
    {
        math::Vec3f tangent;
        math::Vec3f normal;
        math::Vec3f binormal;
    };

    std::vector<Frame> frames(n);

    // First frame: use world up as reference
    {
        math::Vec3f dir = geometry.path[1] - geometry.path[0];
        float len = dir.length();
        if (len < 1e-8f)
        {
            // Degenerate first segment — skip ahead to find a valid direction
            bool found = false;
            for (size_t k = 2; k < n; ++k)
            {
                dir = geometry.path[k] - geometry.path[0];
                len = dir.length();
                if (len >= 1e-8f)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
                return {}; // all points are coincident
        }
        frames[0].tangent = dir / len;

        math::Vec3f ref = {0.0f, 1.0f, 0.0f};
        // If tangent is nearly parallel to world up, use alternative reference
        float dot = std::abs(frames[0].tangent.y);
        if (dot > 0.999f)
            ref = {0.0f, 0.0f, 1.0f};

        frames[0].normal   = ref.cross(frames[0].tangent).normalized();
        frames[0].binormal = frames[0].tangent.cross(frames[0].normal);
    }

    // Subsequent frames: propagate normal from previous frame
    for (size_t i = 1; i < n; ++i)
    {
        math::Vec3f dir = geometry.path[i] - geometry.path[i - 1];
        float len = dir.length();

        // Handle consecutive identical points
        if (len < 1e-8f)
        {
            frames[i] = frames[i - 1];
            continue;
        }

        frames[i].tangent = dir / len;

        // Use previous normal as reference for continuity
        frames[i].normal   = frames[i - 1].normal.cross(frames[i].tangent).normalized();
        frames[i].binormal = frames[i].tangent.cross(frames[i].normal);
    }

    // ── Build vertices ring by ring ──
    const uint32_t vertsPerRing = radialSegments + 1;
    const size_t totalVerts     = n * vertsPerRing;
    const size_t sideIdxCount   = (n - 1) * radialSegments * 6;
    const size_t capIdxCount    = geometry.capped ? 2 * radialSegments * 3 : 0;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(totalVerts);
    indices.reserve(sideIdxCount + capIdxCount);

    const math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

    for (size_t i = 0; i < n; ++i)
    {
        const auto& frame = frames[i];
        float v_uv        = static_cast<float>(i) / static_cast<float>(n - 1);

        for (uint32_t j = 0; j <= radialSegments; ++j)
        {
            float alpha     = static_cast<float>(j) * 2.0f * pi / static_cast<float>(radialSegments);
            float cos_alpha = std::cos(alpha);
            float sin_alpha = std::sin(alpha);

            math::Vec3f radial = frame.normal * cos_alpha + frame.binormal * sin_alpha;

            Vertex v;
            v.position = geometry.path[i] + radial * radius;
            v.normal   = radial; // already normalized
            v.uv       = {static_cast<float>(j) / static_cast<float>(radialSegments), v_uv, 0.0f};
            v.tangent  = tangent_id;

            vertices.push_back(v);
        }
    }

    // ── Side indices: connect consecutive rings ──
    for (size_t i = 0; i < n - 1; ++i)
    {
        uint32_t ringBase0 = static_cast<uint32_t>(i * vertsPerRing);
        uint32_t ringBase1 = ringBase0 + vertsPerRing;

        for (uint32_t j = 0; j < radialSegments; ++j)
        {
            uint32_t b0 = ringBase0 + j;
            uint32_t b1 = b0 + 1;
            uint32_t t0 = ringBase1 + j;
            uint32_t t1 = t0 + 1;

            // Triangle 1: b0, t0, b1
            indices.push_back(b0);
            indices.push_back(t0);
            indices.push_back(b1);

            // Triangle 2: b1, t0, t1
            indices.push_back(b1);
            indices.push_back(t0);
            indices.push_back(t1);
        }
    }

    // ── Caps ──
    if (geometry.capped)
    {
        // Start cap: center at path[0], fan to ring 0 vertices
        uint32_t startCapIdx = static_cast<uint32_t>(vertices.size());
        {
            Vertex capVert;
            capVert.position = geometry.path[0];
            capVert.normal   = -frames[0].tangent;
            capVert.uv       = {0.5f, 0.0f, 0.0f};
            capVert.tangent  = tangent_id;
            vertices.push_back(capVert);
        }

        for (uint32_t j = 0; j < radialSegments; ++j)
        {
            uint32_t center = startCapIdx;
            uint32_t curr   = j;
            uint32_t next   = j + 1;

            indices.push_back(center);
            indices.push_back(curr);
            indices.push_back(next);
        }

        // End cap: center at path[n-1], fan to last ring vertices
        uint32_t endCapIdx = static_cast<uint32_t>(vertices.size());
        {
            Vertex capVert;
            capVert.position = geometry.path[n - 1];
            capVert.normal   = frames[n - 1].tangent;
            capVert.uv       = {0.5f, 1.0f, 0.0f};
            capVert.tangent  = tangent_id;
            vertices.push_back(capVert);
        }

        uint32_t lastRingBase = static_cast<uint32_t>((n - 1) * vertsPerRing);
        for (uint32_t j = 0; j < radialSegments; ++j)
        {
            uint32_t center = endCapIdx;
            uint32_t curr   = lastRingBase + j;
            uint32_t next   = lastRingBase + j + 1;

            indices.push_back(center);
            indices.push_back(next);
            indices.push_back(curr);
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

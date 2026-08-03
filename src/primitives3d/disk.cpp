#include <exd/geometry/primitives3d.hpp>

#include <cmath>

namespace exd::geometry
{

MeshData generate_disk_mesh(const DiskGeometry& geometry)
{
    constexpr float pi = 3.14159265358979323846f;

    if (geometry.outerRadius <= 0.0f)
        return {};

    const uint32_t segs = geometry.segments < 3 ? 3 : geometry.segments;
    const float outerR = geometry.outerRadius;
    const float innerR = geometry.innerRadius;

    const bool isRing = (innerR > 0.0f && innerR < outerR);

    // Ring: two concentric rings of vertices, each with segs+1 points
    // Disk: one ring + center vertex
    const size_t vertCount = isRing
        ? static_cast<size_t>(2) * (segs + 1)
        : static_cast<size_t>(segs + 1) + 1;
    const size_t idxCount = isRing
        ? static_cast<size_t>(segs) * 6
        : static_cast<size_t>(segs) * 3;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(vertCount);
    indices.reserve(idxCount);

    // Disk/ring lies in the XZ plane (Y=0), normal facing +Y
    const math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

    if (isRing)
    {
        // ── Ring / annulus ──
        // Outer ring (j=0)
        for (uint32_t i = 0; i <= segs; ++i)
        {
            float angle = static_cast<float>(i) * 2.0f * pi / static_cast<float>(segs);
            float cos_a = std::cos(angle);
            float sin_a = std::sin(angle);

            Vertex v;
            v.position = {outerR * cos_a, 0.0f, outerR * sin_a};
            v.normal   = {0.0f, 1.0f, 0.0f};
            v.uv       = {
                (cos_a + 1.0f) * 0.5f,
                (sin_a + 1.0f) * 0.5f,
                0.0f
            };
            v.tangent  = tangent_id;
            v.color    = geometry.color;
            vertices.push_back(v);
        }

        // Inner ring (j=1)
        for (uint32_t i = 0; i <= segs; ++i)
        {
            float angle = static_cast<float>(i) * 2.0f * pi / static_cast<float>(segs);
            float cos_a = std::cos(angle);
            float sin_a = std::sin(angle);

            Vertex v;
            v.position = {innerR * cos_a, 0.0f, innerR * sin_a};
            v.normal   = {0.0f, 1.0f, 0.0f};
            v.uv       = {
                (cos_a * innerR / outerR + 1.0f) * 0.5f,
                (sin_a * innerR / outerR + 1.0f) * 0.5f,
                0.0f
            };
            v.tangent  = tangent_id;
            v.color    = geometry.color;
            vertices.push_back(v);
        }

        // Indices: each cell between rings is a quad
        for (uint32_t i = 0; i < segs; ++i)
        {
            uint32_t o0 = i;
            uint32_t o1 = i + 1;
            uint32_t i0 = static_cast<uint32_t>(segs + 1) + i;
            uint32_t i1 = i0 + 1;

            // Triangle 1: o0, i0, o1
            indices.push_back(o0);
            indices.push_back(i0);
            indices.push_back(o1);

            // Triangle 2: o1, i0, i1
            indices.push_back(o1);
            indices.push_back(i0);
            indices.push_back(i1);
        }
    }
    else
    {
        // ── Solid disk ──
        // Center vertex
        {
            Vertex v;
            v.position = {0.0f, 0.0f, 0.0f};
            v.normal   = {0.0f, 1.0f, 0.0f};
            v.uv       = {0.5f, 0.5f, 0.0f};
            v.tangent  = tangent_id;
            v.color    = geometry.color;
            vertices.push_back(v);
        }

        const uint32_t centerIdx = 0;

        // Perimeter ring
        for (uint32_t i = 0; i <= segs; ++i)
        {
            float angle = static_cast<float>(i) * 2.0f * pi / static_cast<float>(segs);
            float cos_a = std::cos(angle);
            float sin_a = std::sin(angle);

            Vertex v;
            v.position = {outerR * cos_a, 0.0f, outerR * sin_a};
            v.normal   = {0.0f, 1.0f, 0.0f};
            v.uv       = {
                (cos_a + 1.0f) * 0.5f,
                (sin_a + 1.0f) * 0.5f,
                0.0f
            };
            v.tangent  = tangent_id;
            v.color    = geometry.color;
            vertices.push_back(v);
        }

        // Triangle fan: center, curr, next
        for (uint32_t i = 0; i < segs; ++i)
        {
            uint32_t curr = 1 + i;
            uint32_t next = curr + 1;

            indices.push_back(centerIdx);
            indices.push_back(curr);
            indices.push_back(next);
        }
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {{-outerR, 0.0f, -outerR}, {outerR, 0.0f, outerR}};

    return mesh;
}

} // namespace exd::geometry

#include <exd/geometry/primitives3d.hpp>
#include <exd/geometry/part.hpp>

#include <cmath>

namespace exd::geometry
{

MeshData generate_cylinder_mesh(const CylinderGeometry& geometry)
{
    constexpr float pi = 3.14159265358979323846f;

    if (geometry.height <= 0.0f || geometry.radius <= 0.0f)
        return {};

    uint32_t slices = geometry.slices < 3 ? 3 : geometry.slices;
    float radius = geometry.radius;
    float halfH = geometry.height * 0.5f;

    size_t sideVertCount = static_cast<size_t>(2) * (slices + 1);
    size_t capVertCount  = geometry.capped ? 2 : 0;
    size_t sideIdxCount  = static_cast<size_t>(slices) * 6;
    size_t capIdxCount   = geometry.capped ? static_cast<size_t>(slices) * 3 * 2 : 0;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(sideVertCount + capVertCount);
    indices.reserve(sideIdxCount + capIdxCount);

    // Side vertices: bottom ring (ring 0) then top ring (ring 1)
    // Bottom ring: indices 0 .. slices
    // Top ring:    indices (slices+1) .. 2*(slices+1)-1
    for (uint32_t ring = 0; ring < 2; ++ring)
    {
        float y = (ring == 0) ? -halfH : halfH;
        float v = (ring == 0) ? 0.0f : 1.0f;

        for (uint32_t i = 0; i <= slices; ++i)
        {
            float phi = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
            float cos_phi = std::cos(phi);
            float sin_phi = std::sin(phi);

            Vertex vert;
            vert.position = {radius * cos_phi, y, radius * sin_phi};
            vert.normal   = {cos_phi, 0.0f, sin_phi};
            vert.uv       = {static_cast<float>(i) / static_cast<float>(slices), v, 0.0f};
            vert.color    = geometry.color;
            vertices.push_back(vert);
        }
    }

    // Side indices: quads between bottom and top rings
    for (uint32_t i = 0; i < slices; ++i)
    {
        uint32_t b0 = i;
        uint32_t b1 = i + 1;
        uint32_t t0 = static_cast<uint32_t>(slices + 1) + i;
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

    if (geometry.capped)
    {
        // Cap center vertices
        uint32_t bottomCapIdx = static_cast<uint32_t>(vertices.size());
        {
            Vertex vert;
            vert.position = {0.0f, -halfH, 0.0f};
            vert.normal   = {0.0f, -1.0f, 0.0f};
            vert.uv       = {0.5f, 0.5f, 0.0f};
            vert.color    = geometry.color;
            vertices.push_back(vert);
        }

        uint32_t topCapIdx = static_cast<uint32_t>(vertices.size());
        {
            Vertex vert;
            vert.position = {0.0f, halfH, 0.0f};
            vert.normal   = {0.0f, 1.0f, 0.0f};
            vert.uv       = {0.5f, 0.5f, 0.0f};
            vert.color    = geometry.color;
            vertices.push_back(vert);
        }

        // Bottom cap: triangle fan, normal -Y
        // Bottom ring vertices are at indices 0 .. slices
        for (uint32_t i = 0; i < slices; ++i)
        {
            uint32_t center = bottomCapIdx;
            uint32_t next   = i + 1;  // bottom ring, next
            uint32_t curr   = i;      // bottom ring, current

            indices.push_back(center);
            indices.push_back(next);
            indices.push_back(curr);
        }

        // Top cap: triangle fan, normal +Y
        // Top ring vertices are at indices (slices+1) .. 2*(slices+1)-1
        uint32_t topRingBase = slices + 1;
        for (uint32_t i = 0; i < slices; ++i)
        {
            uint32_t center = topCapIdx;
            uint32_t curr   = topRingBase + i;     // top ring, current
            uint32_t next   = topRingBase + i + 1; // top ring, next

            indices.push_back(center);
            indices.push_back(curr);
            indices.push_back(next);
        }
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {{-radius, -halfH, -radius}, {radius, halfH, radius}};

    return mesh;
}

Part generate_cylinder_part(const CylinderGeometry& geometry)
{
    Part part = as_part("cylinder", generate_cylinder_mesh(geometry));
    if (part.mesh.vertices.empty())
        return part;

    const uint32_t S = geometry.slices < 3 ? 3 : geometry.slices;
    part.patches.push_back(make_patch_range("wall", 0, 2 * S));
    if (geometry.capped)
    {
        part.patches.push_back(make_patch_range("cap_start", 2 * S, S));
        part.patches.push_back(make_patch_range("cap_end", 3 * S, S));
    }
    return part;
}

} // namespace exd::geometry

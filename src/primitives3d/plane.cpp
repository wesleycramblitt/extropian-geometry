#include <exd/geometry/primitives3d.hpp>

#include <cmath>

namespace exd::geometry
{

MeshData generate_plane_mesh(const PlaneGeometry& geometry)
{
    if (geometry.size.x <= 0.0f || geometry.size.z <= 0.0f)
        return {};

    uint32_t segW = geometry.segmentsW < 1 ? 1 : geometry.segmentsW;
    uint32_t segD = geometry.segmentsD < 1 ? 1 : geometry.segmentsD;

    float hw = geometry.size.x * 0.5f;
    float hd = geometry.size.z * 0.5f;

    size_t vertCount = static_cast<size_t>(segW + 1) * (segD + 1);
    size_t idxCount  = static_cast<size_t>(segW) * segD * 6;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(vertCount);
    indices.reserve(idxCount);

    // Grid of (segW+1) x (segD+1) vertices in the XZ plane (Y=0)
    for (uint32_t d = 0; d <= segD; ++d)
    {
        float z = -hd + static_cast<float>(d) * geometry.size.z / static_cast<float>(segD);
        float v = static_cast<float>(d) / static_cast<float>(segD);

        for (uint32_t w = 0; w <= segW; ++w)
        {
            float x = -hw + static_cast<float>(w) * geometry.size.x / static_cast<float>(segW);
            float u = static_cast<float>(w) / static_cast<float>(segW);

            Vertex vert;
            vert.position = {x, 0.0f, z};
            vert.normal   = {0.0f, 1.0f, 0.0f};
            vert.uv       = {u, v, 0.0f};
            vert.color    = geometry.color;
            vertices.push_back(vert);
        }
    }

    // Indices: each grid cell is a quad (2 triangles)
    for (uint32_t d = 0; d < segD; ++d)
    {
        for (uint32_t w = 0; w < segW; ++w)
        {
            uint32_t i0 = d * (segW + 1) + w;
            uint32_t i1 = i0 + 1;
            uint32_t i2 = i0 + static_cast<uint32_t>(segW + 1);
            uint32_t i3 = i2 + 1;

            // Triangle 1: i0, i2, i1
            indices.push_back(i0);
            indices.push_back(i2);
            indices.push_back(i1);

            // Triangle 2: i1, i2, i3
            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i3);
        }
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {{-hw, 0.0f, -hd}, {hw, 0.0f, hd}};

    return mesh;
}

} // namespace exd::geometry

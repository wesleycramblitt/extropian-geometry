#include <exd/geometry/primitives3d.hpp>
#include <exd/geometry/part.hpp>

#include <cmath>

namespace exd::geometry
{

MeshData generate_cone_mesh(const ConeGeometry& geometry)
{
    constexpr float pi = 3.14159265358979323846f;

    if (geometry.height <= 0.0f || geometry.radius <= 0.0f)
        return {};

    uint32_t slices = geometry.slices < 3 ? 3 : geometry.slices;
    float radius = geometry.radius;
    float halfH = geometry.height * 0.5f;

    // Side: (slices+1) base ring vertices + 1 tip vertex
    size_t sideVertCount = static_cast<size_t>(slices + 1) + 1;
    size_t capVertCount  = geometry.capped ? 1 : 0;
    size_t sideIdxCount  = static_cast<size_t>(slices) * 3;
    size_t capIdxCount   = geometry.capped ? static_cast<size_t>(slices) * 3 : 0;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(sideVertCount + capVertCount);
    indices.reserve(sideIdxCount + capIdxCount);

    math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

    // Base ring vertices at Y = -halfH
    for (uint32_t i = 0; i <= slices; ++i)
    {
        float phi = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
        float cos_phi = std::cos(phi);
        float sin_phi = std::sin(phi);

        Vertex vert;
        vert.position = {radius * cos_phi, -halfH, radius * sin_phi};
        // Side normal: perpendicular to edge from base to tip
        // Edge from base to tip: (0 - radius*cos_phi, halfH - (-halfH), 0 - radius*sin_phi)
        //                        = (-radius*cos_phi, height, -radius*sin_phi)
        // Tangent along ring: (-sin_phi, 0, cos_phi)
        // Normal = cross(tangent, edge) normalized
        float nx =  geometry.height * cos_phi;
        float ny =  radius;
        float nz =  geometry.height * sin_phi;
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 0.0f)
        {
            nx /= len;
            ny /= len;
            nz /= len;
        }
        vert.normal   = {nx, ny, nz};
        vert.uv       = {static_cast<float>(i) / static_cast<float>(slices), 0.0f, 0.0f};
        vert.tangent  = tangent_id;
        vert.color    = geometry.color;

        vertices.push_back(vert);
    }

    // Tip vertex at Y = +halfH
    uint32_t tipIdx = static_cast<uint32_t>(vertices.size());
    {
        Vertex vert;
        vert.position = {0.0f, halfH, 0.0f};
        vert.normal   = {0.0f, 1.0f, 0.0f};
        vert.uv       = {0.5f, 1.0f, 0.0f};
        vert.tangent  = tangent_id;
        vert.color    = geometry.color;
        vertices.push_back(vert);
    }

    // Side triangles: tip, base[i], base[i+1] for outward-facing normals
    for (uint32_t i = 0; i < slices; ++i)
    {
        indices.push_back(tipIdx);
        indices.push_back(i);
        indices.push_back(i + 1);
    }

    if (geometry.capped)
    {
        // Bottom cap center
        uint32_t capCenterIdx = static_cast<uint32_t>(vertices.size());
        {
            Vertex vert;
            vert.position = {0.0f, -halfH, 0.0f};
            vert.normal   = {0.0f, -1.0f, 0.0f};
            vert.uv       = {0.5f, 0.5f, 0.0f};
            vert.tangent  = tangent_id;
            vert.color    = geometry.color;
            vertices.push_back(vert);
        }

        // Bottom cap: triangle fan, winding faces downward (-Y)
        // Order: center, next, curr (reversed for -Y normal)
        for (uint32_t i = 0; i < slices; ++i)
        {
            indices.push_back(capCenterIdx);
            indices.push_back(i + 1);
            indices.push_back(i);
        }
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {{-radius, -halfH, -radius}, {radius, halfH, radius}};

    return mesh;
}

Part generate_cone_part(const ConeGeometry& geometry)
{
    Part part = as_part("cone", generate_cone_mesh(geometry));
    if (part.mesh.vertices.empty())
        return part;

    const uint32_t S = geometry.slices < 3 ? 3 : geometry.slices;
    part.patches.push_back(make_patch_range("wall", 0, S));
    if (geometry.capped)
        part.patches.push_back(make_patch_range("cap_start", S, S));
    return part;
}

} // namespace exd::geometry

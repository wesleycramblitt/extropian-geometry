#include <exd/geometry/heightmap.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>

namespace exd::geometry
{

MeshData generate_heightmap_mesh(const Heightmap& hm)
{
    if (hm.width < 2 || hm.height < 2 || hm.heightData.size() < hm.width * hm.height)
        return {};

    MeshBuilder builder;
    builder.reserve(hm.width * hm.height, (hm.width - 1) * (hm.height - 1) * 6);

    const float dx = hm.size.x / static_cast<float>(hm.width - 1);
    const float dz = hm.size.z / static_cast<float>(hm.height - 1);
    const float yScale = hm.size.y;

    // Generate vertices
    for (uint32_t z = 0; z < hm.height; ++z) {
        for (uint32_t x = 0; x < hm.width; ++x) {
            float h = hm.heightData[z * hm.width + x] * yScale;

            Vertex v;
            v.position = {static_cast<float>(x) * dx - hm.size.x * 0.5f,
                          h,
                          static_cast<float>(z) * dz - hm.size.z * 0.5f};
            v.normal   = {0, 1, 0};
            v.uv       = {static_cast<float>(x) / static_cast<float>(hm.width - 1),
                          static_cast<float>(z) / static_cast<float>(hm.height - 1),
                          0};
            v.color    = hm.color;
            builder.add_vertex(v);
        }
    }

    // Generate indices (two triangles per cell)
    for (uint32_t z = 0; z < hm.height - 1; ++z) {
        for (uint32_t x = 0; x < hm.width - 1; ++x) {
            uint32_t tl = z * hm.width + x;
            uint32_t tr = tl + 1;
            uint32_t bl = (z + 1) * hm.width + x;
            uint32_t br = bl + 1;

            builder.add_triangle(tl, bl, tr);
            builder.add_triangle(tr, bl, br);
        }
    }

    // Smooth per-vertex normals: accumulate unnormalized triangle normals
    // (winding is CCW from above, so crosses point up for flat terrain),
    // then normalize. Without this the surface would shade as if flat.
    auto verts = builder.build();
    std::vector<math::Vec3f> normals(verts.vertices.size(), math::Vec3f{0.0f, 0.0f, 0.0f});
    for (size_t i = 0; i + 2 < verts.indices.size(); i += 3) {
        const auto& a = verts.vertices[verts.indices[i]];
        const auto& b = verts.vertices[verts.indices[i + 1]];
        const auto& c = verts.vertices[verts.indices[i + 2]];
        const math::Vec3f n = (b.position - a.position).cross(c.position - a.position);
        normals[verts.indices[i]]     += n;
        normals[verts.indices[i + 1]] += n;
        normals[verts.indices[i + 2]] += n;
    }
    for (size_t i = 0; i < normals.size(); ++i) {
        const float len = normals[i].length();
        verts.vertices[i].normal = len > 1e-8f ? normals[i] / len : math::Vec3f{0.0f, 1.0f, 0.0f};
    }

    MeshData result = verts;
    result.bounds = compute_bounds(result.vertices);
    return result;
}

} // namespace exd::geometry

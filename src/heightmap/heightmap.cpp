#include <exd/geometry/heightmap.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>

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
            v.normal   = {0, 1, 0}; // will be computed by compute_bounds
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

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
}

} // namespace exd::geometry

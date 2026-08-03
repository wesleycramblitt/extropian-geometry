#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <cmath>
#include <numbers>

namespace exd::geometry
{

MeshData generate_star_mesh(const StarGeometry& geom)
{
    const uint32_t pts = std::max(3u, geom.points);
    const uint32_t totalVerts = pts * 2; // outer + inner vertices
    const float step = std::numbers::pi_v<float> / static_cast<float>(pts);

    MeshBuilder builder;
    builder.reserve(totalVerts + 1, pts * 2 * 3); // +1 for center, 2 triangles per point

    // Center vertex
    Vertex cv;
    cv.position = {0, 0, 0};
    cv.normal   = {0, 0, 1};
    cv.uv       = {0.5f, 0.5f, 0};
    cv.color    = geom.color;
    const uint32_t centerIdx = builder.add_vertex(cv);

    // Generate outer and inner vertices alternating
    // Start at top (90 degrees) and go clockwise
    std::vector<uint32_t> verts(totalVerts);
    const float startAngle = std::numbers::pi_v<float> / 2.0f; // top

    for (uint32_t i = 0; i < pts; ++i)
    {
        float outerAngle = startAngle + static_cast<float>(i) * 2.0f * step;
        float innerAngle = outerAngle + step;

        // Outer vertex
        {
            Vertex v;
            v.position = {geom.outerRadius * std::cos(outerAngle),
                          geom.outerRadius * std::sin(outerAngle), 0};
            v.normal  = {0, 0, 1};
            v.uv      = {(std::cos(outerAngle) + 1) * 0.5f, (std::sin(outerAngle) + 1) * 0.5f, 0};
            v.color   = geom.color;
            verts[i * 2] = builder.add_vertex(v);
        }

        // Inner vertex
        {
            Vertex v;
            v.position = {geom.innerRadius * std::cos(innerAngle),
                          geom.innerRadius * std::sin(innerAngle), 0};
            v.normal  = {0, 0, 1};
            v.uv      = {(std::cos(innerAngle) + 1) * 0.5f, (std::sin(innerAngle) + 1) * 0.5f, 0};
            v.color   = geom.color;
            verts[i * 2 + 1] = builder.add_vertex(v);
        }
    }

    // Triangle fan: center → outer[i] → inner[i] → outer[i+1]
    for (uint32_t i = 0; i < pts; ++i)
    {
        uint32_t outer = verts[i * 2];
        uint32_t inner = verts[i * 2 + 1];
        uint32_t nextOuter = verts[((i + 1) * 2) % totalVerts];

        builder.add_triangle(centerIdx, outer, inner);
        builder.add_triangle(centerIdx, inner, nextOuter);
    }

    auto result = builder.build();
    result.bounds = {{-geom.outerRadius, -geom.outerRadius, 0},
                     { geom.outerRadius,  geom.outerRadius, 0}};
    return result;
}

} // namespace exd::geometry

#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <cmath>
#include <numbers>

namespace exd::geometry
{

MeshData generate_regular_polygon_mesh(const RegularPolygonGeometry& geom)
{
    const uint32_t sides = std::max(3u, geom.sides);

    MeshBuilder builder;
    builder.reserve(sides + 1, sides * 3);

    // Center vertex
    Vertex cv;
    cv.position = {0, 0, 0};
    cv.normal   = {0, 0, 1};
    cv.uv       = {0.5f, 0.5f, 0};
    cv.color    = geom.color;
    const uint32_t centerIdx = builder.add_vertex(cv);

    // Perimeter vertices — start at top, go clockwise
    const float step = 2.0f * std::numbers::pi_v<float> / static_cast<float>(sides);
    const float startAngle = std::numbers::pi_v<float> / 2.0f; // top (90°)

    std::vector<uint32_t> permIndices(sides);
    for (uint32_t i = 0; i < sides; ++i)
    {
        float angle = startAngle + step * static_cast<float>(i);
        Vertex v;
        v.position = {geom.radius * std::cos(angle),
                      geom.radius * std::sin(angle), 0};
        v.normal  = {0, 0, 1};
        v.uv      = {(std::cos(angle) + 1) * 0.5f,
                     (std::sin(angle) + 1) * 0.5f, 0};
        v.color   = geom.color;
        permIndices[i] = builder.add_vertex(v);
    }

    // Triangle fan
    for (uint32_t i = 0; i < sides; ++i)
    {
        uint32_t j = (i + 1) % sides;
        builder.add_triangle(centerIdx, permIndices[i], permIndices[j]);
    }

    auto result = builder.build();
    result.bounds = {{-geom.radius, -geom.radius, 0},
                     { geom.radius,  geom.radius, 0}};
    return result;
}

} // namespace exd::geometry

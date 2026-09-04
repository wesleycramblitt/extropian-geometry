#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <algorithm>
#include <cmath>

namespace exd::geometry
{

MeshData generate_line_mesh(const LineGeometry& geom)
{
    const math::Vec3f dir   = geom.end - geom.start;
    const float       len   = dir.length();

    if (len < 1e-6f)
    {
        // Degenerate line — return empty mesh
        return {};
    }

    const math::Vec3f forward = dir / len;  // normalized direction
    // Perpendicular in XY plane: rotate forward by +90° around Z
    const math::Vec3f right = {-forward.y, forward.x, 0.0f};

    const float hw = geom.width * 0.5f;

    MeshBuilder builder;
    builder.reserve(4, 6);

    Vertex v0, v1, v2, v3;

    // Four corners of the line quad (CCW order for front-facing)
    // v0: start - right*HW
    // v1: start + right*HW
    // v2: end   + right*HW
    // v3: end   - right*HW
    v0.position = geom.start - right * hw;
    v1.position = geom.start + right * hw;
    v2.position = geom.end   + right * hw;
    v3.position = geom.end   - right * hw;

    // Normals point toward +Z
    v0.normal = {0.0f, 0.0f, 1.0f};
    v1.normal = {0.0f, 0.0f, 1.0f};
    v2.normal = {0.0f, 0.0f, 1.0f};
    v3.normal = {0.0f, 0.0f, 1.0f};

    // UVs: u maps along the line (0 at start, 1 at end), v maps across width
    v0.uv = {0.0f, 0.0f, 0.0f};
    v1.uv = {0.0f, 1.0f, 0.0f};
    v2.uv = {1.0f, 1.0f, 0.0f};
    v3.uv = {1.0f, 0.0f, 0.0f};

    v0.color = geom.color;
    v1.color = geom.color;
    v2.color = geom.color;
    v3.color = geom.color;

    const auto a = builder.add_vertex(v0);
    const auto b = builder.add_vertex(v1);
    const auto c = builder.add_vertex(v2);
    const auto d = builder.add_vertex(v3);

    builder.add_triangle(a, b, c);
    builder.add_triangle(a, c, d);

    auto result = builder.build();

    // Compute bounds manually for accuracy
    const float minX = std::min({v0.position.x, v1.position.x, v2.position.x, v3.position.x});
    const float minY = std::min({v0.position.y, v1.position.y, v2.position.y, v3.position.y});
    const float maxX = std::max({v0.position.x, v1.position.x, v2.position.x, v3.position.x});
    const float maxY = std::max({v0.position.y, v1.position.y, v2.position.y, v3.position.y});
    result.bounds = {{minX, minY, 0.0f}, {maxX, maxY, 0.0f}};

    return result;
}

} // namespace exd::geometry

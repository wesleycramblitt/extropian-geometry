#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <algorithm>
#include <cmath>

namespace exd::geometry
{

MeshData generate_rect_mesh(const RectangleGeometry& geom)
{
    const float hw = geom.size.x * 0.5f;
    const float hh = geom.size.y * 0.5f;

    // Degenerate case: zero size in either dimension
    if (hw < 1e-6f || hh < 1e-6f)
    {
        return {};
    }

    MeshBuilder builder;
    builder.reserve(4, 6);

    // Four corners in CCW order when viewed from Z+
    // v0: bottom-left, v1: bottom-right, v2: top-right, v3: top-left
    Vertex v0, v1, v2, v3;

    v0.position = {-hw, -hh, 0.0f};
    v1.position = { hw, -hh, 0.0f};
    v2.position = { hw,  hh, 0.0f};
    v3.position = {-hw,  hh, 0.0f};

    // Normals point toward +Z
    v0.normal = {0.0f, 0.0f, 1.0f};
    v1.normal = {0.0f, 0.0f, 1.0f};
    v2.normal = {0.0f, 0.0f, 1.0f};
    v3.normal = {0.0f, 0.0f, 1.0f};

    // UVs: [0,1] across the rectangle
    v0.uv = {0.0f, 0.0f, 0.0f};
    v1.uv = {1.0f, 0.0f, 0.0f};
    v2.uv = {1.0f, 1.0f, 0.0f};
    v3.uv = {0.0f, 1.0f, 0.0f};

    const auto a = builder.add_vertex(v0);
    const auto b = builder.add_vertex(v1);
    const auto c = builder.add_vertex(v2);
    const auto d = builder.add_vertex(v3);

    // Two triangles forming a quad (CCW winding)
    builder.add_triangle(a, b, c);
    builder.add_triangle(a, c, d);

    auto result = builder.build();
    result.bounds = {{-hw, -hh, 0.0f}, {hw, hh, 0.0f}};
    return result;
}

} // namespace exd::geometry

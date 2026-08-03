#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>

namespace exd::geometry
{

MeshData generate_polyline_mesh(const PolylineGeometry& geom)
{
    if (geom.points.size() < 2)
    {
        return {};
    }

    const float hw = geom.width * 0.5f;

    // Count segments
    size_t numSegments = geom.points.size() - 1;
    if (geom.closed)
    {
        ++numSegments;
    }

    // Pre-reserve: 4 vertices + 6 indices per segment
    MeshBuilder builder;
    builder.reserve(numSegments * 4, numSegments * 6);

    auto emit_segment = [&](const math::Vec3f& p0, const math::Vec3f& p1)
    {
        const math::Vec3f dir = p1 - p0;
        const float len = dir.length();

        // Skip degenerate segments
        if (len < 1e-6f)
        {
            return;
        }

        const math::Vec3f forward = dir / len;
        // Perpendicular in XY plane: rotate forward by +90° around Z
        const math::Vec3f right = {-forward.y, forward.x, 0.0f};

        Vertex v0, v1, v2, v3;

        // Four corners of the quad (CCW order for front-facing from Z+)
        // v0: p0 - right*HW   (bottom-left)
        // v1: p0 + right*HW   (top-left)
        // v2: p1 + right*HW   (top-right)
        // v3: p1 - right*HW   (bottom-right)
        v0.position = p0 - right * hw;
        v1.position = p0 + right * hw;
        v2.position = p1 + right * hw;
        v3.position = p1 - right * hw;

        v0.normal = {0.0f, 0.0f, 1.0f};
        v1.normal = {0.0f, 0.0f, 1.0f};
        v2.normal = {0.0f, 0.0f, 1.0f};
        v3.normal = {0.0f, 0.0f, 1.0f};

        // UVs: u along segment (0→1), v across width (0→1)
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
    };

    // Emit segments between consecutive points
    for (size_t i = 0; i < geom.points.size() - 1; ++i)
    {
        emit_segment(geom.points[i], geom.points[i + 1]);
    }

    // If closed, emit segment from last point back to first
    if (geom.closed && geom.points.size() >= 2)
    {
        emit_segment(geom.points.back(), geom.points.front());
    }

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
}

} // namespace exd::geometry

#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <algorithm>
#include <cmath>

namespace exd::geometry
{

MeshData generate_grid_mesh(const GridGeometry& geom)
{
    if (geom.rows == 0 && geom.columns == 0)
    {
        return {};
    }

    const float hw = geom.size.x * 0.5f;
    const float hh = geom.size.y * 0.5f;
    const float lineHW = geom.lineWidth * 0.5f;

    // Total lines: (rows + 1) horizontal + (columns + 1) vertical
    // Each line is a quad: 4 verts + 6 indices
    const uint32_t numHLines = geom.rows + 1;
    const uint32_t numVLines = geom.columns + 1;
    const uint32_t totalLines = numHLines + numVLines;

    MeshBuilder builder;
    builder.reserve(totalLines * 4, totalLines * 6);

    auto emit_line_quad = [&](const math::Vec3f& p0, const math::Vec3f& p1,
                              const math::Vec3f& right, float v0v, float v1v)
    {
        Vertex v0, v1, v2, v3;

        v0.position = p0 - right * lineHW;
        v1.position = p0 + right * lineHW;
        v2.position = p1 + right * lineHW;
        v3.position = p1 - right * lineHW;

        v0.normal = {0.0f, 0.0f, 1.0f};
        v1.normal = {0.0f, 0.0f, 1.0f};
        v2.normal = {0.0f, 0.0f, 1.0f};
        v3.normal = {0.0f, 0.0f, 1.0f};

        v0.uv = {0.0f, v0v, 0.0f};
        v1.uv = {0.0f, v1v, 0.0f};
        v2.uv = {1.0f, v1v, 0.0f};
        v3.uv = {1.0f, v0v, 0.0f};

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

    // ── Horizontal lines: span from left to right ──
    if (numHLines > 0)
    {
        const float yStep = (geom.rows > 0)
            ? (2.0f * hh) / static_cast<float>(geom.rows)
            : 0.0f;

        for (uint32_t i = 0; i <= geom.rows; ++i)
        {
            const float y = -hh + static_cast<float>(i) * yStep;

            const math::Vec3f p0 = {-hw, y, 0.0f};
            const math::Vec3f p1 = { hw, y, 0.0f};

            // Right direction for horizontal line is +Y (perpendicular to +X)
            const math::Vec3f right = {0.0f, 1.0f, 0.0f};

            // UV v maps to normalized Y position
            const float v = (y + hh) / (2.0f * hh);
            emit_line_quad(p0, p1, right, v, v);
        }
    }

    // ── Vertical lines: span from bottom to top ──
    if (numVLines > 0)
    {
        const float xStep = (geom.columns > 0)
            ? (2.0f * hw) / static_cast<float>(geom.columns)
            : 0.0f;

        for (uint32_t i = 0; i <= geom.columns; ++i)
        {
            const float x = -hw + static_cast<float>(i) * xStep;

            const math::Vec3f p0 = {x, -hh, 0.0f};
            const math::Vec3f p1 = {x,  hh, 0.0f};

            // Right direction for vertical line is +X (perpendicular to +Y)
            const math::Vec3f right = {1.0f, 0.0f, 0.0f};

            // UV v maps to normalized X position
            const float v = (x + hw) / (2.0f * hw);
            emit_line_quad(p0, p1, right, v, v);
        }
    }

    auto result = builder.build();
    result.bounds = {
        {-hw, -hh, 0.0f},
        { hw,  hh, 0.0f}
    };
    return result;
}

} // namespace exd::geometry

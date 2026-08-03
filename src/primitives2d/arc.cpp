#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace exd::geometry
{

MeshData generate_arc_mesh(const ArcGeometry& geom)
{
    const float sweep = geom.endAngle - geom.startAngle;

    // Near-zero sweep: no area to fill
    if (std::abs(sweep) < 1e-6f)
    {
        return {};
    }

    const float absSweep = std::abs(sweep);
    const float fraction = absSweep / (2.0f * std::numbers::pi_v<float>);
    const uint32_t segs = std::max(1u,
        static_cast<uint32_t>(static_cast<float>(geom.segments) * fraction));

    // Need at least 2 perimeter vertices for 1 triangle
    const uint32_t numPerimeter = segs + 1;

    MeshBuilder builder;
    builder.reserve(numPerimeter + 1, numPerimeter * 3);

    // Center vertex at origin
    Vertex cv;
    cv.position = {0.0f, 0.0f, 0.0f};
    cv.normal   = {0.0f, 0.0f, 1.0f};
    cv.uv       = {0.5f, 0.5f, 0.0f};
    cv.color    = geom.color;
    const uint32_t centerIdx = builder.add_vertex(cv);

    // Perimeter vertices from startAngle to endAngle
    std::vector<uint32_t> permIndices;
    permIndices.reserve(numPerimeter);

    const float step = sweep / static_cast<float>(segs);

    // Track bounds from perimeter vertices
    float minX = 0.0f, minY = 0.0f;
    float maxX = 0.0f, maxY = 0.0f;

    for (uint32_t i = 0; i <= segs; ++i)
    {
        const float angle = geom.startAngle + step * static_cast<float>(i);
        const float cx = std::cos(angle);
        const float sy = std::sin(angle);

        Vertex v;
        v.position = {
            geom.radius * cx,
            geom.radius * sy,
            0.0f
        };
        v.normal = {0.0f, 0.0f, 1.0f};
        // UV: map from [-r, r] to [0, 1]
        v.uv = {
            (cx + 1.0f) * 0.5f,
            (sy + 1.0f) * 0.5f,
            0.0f
        };
        v.color = geom.color;

        if (i == 0)
        {
            minX = maxX = v.position.x;
            minY = maxY = v.position.y;
        }
        else
        {
            minX = std::min(minX, v.position.x);
            minY = std::min(minY, v.position.y);
            maxX = std::max(maxX, v.position.x);
            maxY = std::max(maxY, v.position.y);
        }

        permIndices.push_back(builder.add_vertex(v));
    }

    // Triangle fan: center → perm[i] → perm[i+1]
    for (size_t i = 0; i + 1 < permIndices.size(); ++i)
    {
        builder.add_triangle(centerIdx, permIndices[i], permIndices[i + 1]);
    }

    auto result = builder.build();
    result.bounds = {{minX, minY, 0.0f}, {maxX, maxY, 0.0f}};
    return result;
}

} // namespace exd::geometry

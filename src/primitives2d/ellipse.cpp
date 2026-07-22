#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace exd::geometry
{

MeshData generate_ellipse_mesh(const EllipseGeometry& geom)
{
    const uint32_t segs = std::max(3u, geom.segments);

    MeshBuilder builder;
    builder.reserve(segs + 1, segs * 3);

    // Center vertex at origin
    Vertex cv;
    cv.position = {0.0f, 0.0f, 0.0f};
    cv.normal   = {0.0f, 0.0f, 1.0f};
    cv.uv       = {0.5f, 0.5f, 0.0f};
    const uint32_t centerIdx = builder.add_vertex(cv);

    // Perimeter vertices — CCW order starting from angle 0 (+X axis)
    std::vector<uint32_t> permIndices;
    permIndices.reserve(segs);

    const float step = 2.0f * std::numbers::pi_v<float> / static_cast<float>(segs);
    for (uint32_t i = 0; i < segs; ++i)
    {
        const float angle = step * static_cast<float>(i);
        const float cx = std::cos(angle);
        const float sy = std::sin(angle);

        Vertex v;
        v.position = {
            geom.radiusX * cx,
            geom.radiusY * sy,
            0.0f
        };
        v.normal = {0.0f, 0.0f, 1.0f};
        // UV: map from [-radiusX, radiusX] to [0, 1] and [-radiusY, radiusY] to [0, 1]
        v.uv = {
            (cx + 1.0f) * 0.5f,
            (sy + 1.0f) * 0.5f,
            0.0f
        };
        permIndices.push_back(builder.add_vertex(v));
    }

    // Triangle fan: center → perm[i] → perm[i+1]
    for (size_t i = 0; i < permIndices.size(); ++i)
    {
        const size_t j = (i + 1) % permIndices.size();
        builder.add_triangle(centerIdx, permIndices[i], permIndices[j]);
    }

    auto result = builder.build();
    result.bounds = {
        {-geom.radiusX, -geom.radiusY, 0.0f},
        { geom.radiusX,  geom.radiusY, 0.0f}
    };
    return result;
}

} // namespace exd::geometry

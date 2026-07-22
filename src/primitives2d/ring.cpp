#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace exd::geometry
{

MeshData generate_ring_mesh(const RingGeometry& geom)
{
    // Clamp / swap so inner < outer
    float outerR = geom.outerRadius;
    float innerR = geom.innerRadius;
    if (innerR >= outerR)
    {
        std::swap(innerR, outerR);
    }
    if (outerR < 1e-6f)
    {
        return {};
    }
    // Clamp inner to a small fraction of outer to avoid degenerate ring
    innerR = std::max(innerR, 1e-6f);

    const uint32_t segs = std::max(3u, geom.segments);

    MeshBuilder builder;
    builder.reserve(segs * 2, segs * 6);

    const float step = 2.0f * std::numbers::pi_v<float> / static_cast<float>(segs);

    // Build outer and inner ring vertices (CCW order)
    std::vector<uint32_t> outerIndices;
    std::vector<uint32_t> innerIndices;
    outerIndices.reserve(segs);
    innerIndices.reserve(segs);

    for (uint32_t i = 0; i < segs; ++i)
    {
        const float angle = step * static_cast<float>(i);
        const float c = std::cos(angle);
        const float s = std::sin(angle);

        // Outer vertex
        Vertex ov;
        ov.position = {outerR * c, outerR * s, 0.0f};
        ov.normal   = {0.0f, 0.0f, 1.0f};
        ov.uv       = {(c + 1.0f) * 0.5f, (s + 1.0f) * 0.5f, 0.0f};
        outerIndices.push_back(builder.add_vertex(ov));

        // Inner vertex
        Vertex iv;
        iv.position = {innerR * c, innerR * s, 0.0f};
        iv.normal   = {0.0f, 0.0f, 1.0f};
        iv.uv       = {(c + 1.0f) * 0.5f, (s + 1.0f) * 0.5f, 0.0f};
        innerIndices.push_back(builder.add_vertex(iv));
    }

    // Connect outer[i], outer[i+1], inner[i+1], inner[i] as two CCW triangles
    for (uint32_t i = 0; i < segs; ++i)
    {
        const uint32_t o_i     = outerIndices[i];
        const uint32_t o_next  = outerIndices[(i + 1) % segs];
        const uint32_t i_next  = innerIndices[(i + 1) % segs];
        const uint32_t i_i     = innerIndices[i];

        // Triangle 1: outer_i, outer_next, inner_next (CCW from Z+)
        builder.add_triangle(o_i, o_next, i_next);
        // Triangle 2: outer_i, inner_next, inner_i (CCW from Z+)
        builder.add_triangle(o_i, i_next, i_i);
    }

    auto result = builder.build();
    result.bounds = {
        {-outerR, -outerR, 0.0f},
        { outerR,  outerR, 0.0f}
    };
    return result;
}

} // namespace exd::geometry

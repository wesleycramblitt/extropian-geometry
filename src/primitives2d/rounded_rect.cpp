#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace exd::geometry
{

namespace
{

/// Emit arc vertices on the XY plane.
/// Sweeps from startAngle to endAngle; supports both positive (CCW)
/// and negative (CW) sweeps.
void emit_arc(std::vector<math::Vec3f>& out,
              const math::Vec3f& center,
              float radius,
              float startAngle,
              float endAngle,
              uint32_t fullCircleSegments)
{
    const float sweep     = endAngle - startAngle;
    const float absSweep  = std::abs(sweep);
    const float fraction  = absSweep / (2.0f * std::numbers::pi_v<float>);
    const uint32_t segs   = std::max(1u,
        static_cast<uint32_t>(static_cast<float>(fullCircleSegments) * fraction));
    const float step      = sweep / static_cast<float>(segs);

    for (uint32_t i = 0; i <= segs; ++i)
    {
        const float a = startAngle + step * static_cast<float>(i);
        out.push_back({
            center.x + radius * std::cos(a),
            center.y + radius * std::sin(a),
            0.0f
        });
    }
}

float clamp_radius(float r, float halfW, float halfH)
{
    return std::min(r, std::min(halfW, halfH));
}

} // anonymous namespace

MeshData generate_rounded_rect_mesh(const RoundedRectangleGeometry& geom)
{
    const float hw = geom.size.x * 0.5f;
    const float hh = geom.size.y * 0.5f;

    // Clamp radii so they don't exceed half-size
    const float rTL = clamp_radius(geom.radii.topLeft,     hw, hh);
    const float rTR = clamp_radius(geom.radii.topRight,    hw, hh);
    const float rBR = clamp_radius(geom.radii.bottomRight, hw, hh);
    const float rBL = clamp_radius(geom.radii.bottomLeft,  hw, hh);

    const uint32_t segs = std::max(1u, geom.cornerSegments);

    // Rectangle bounds on XY plane, centered at origin
    const float left   = -hw;
    const float right  =  hw;
    const float top    =  hh;
    const float bottom = -hh;

    // ── Build perimeter vertices in CW order ──
    // (clockwise when looking from Z+)
    std::vector<math::Vec3f> perimeter;

    // 1. Top edge: rightwards from after top-left corner to before top-right corner
    perimeter.push_back({left  + rTL, top, 0.0f});
    perimeter.push_back({right - rTR, top, 0.0f});

    // 2. Top-right corner: CW arc from top (π/2) to right (0)
    {
        const math::Vec3f center = {right - rTR, top - rTR, 0.0f};
        emit_arc(perimeter, center, rTR,
                 std::numbers::pi_v<float> / 2.0f,   //  90° (top)
                 0.0f,                                //   0° (right)
                 segs);                               // CW sweep: -π/2
    }

    // 3. Right edge: downwards
    perimeter.push_back({right, bottom + rBR, 0.0f});

    // 4. Bottom-right corner: CW arc from right (0) to bottom (-π/2)
    {
        const math::Vec3f center = {right - rBR, bottom + rBR, 0.0f};
        emit_arc(perimeter, center, rBR,
                 0.0f,                                            //   0° (right)
                 -std::numbers::pi_v<float> / 2.0f,               // -90° (bottom)
                 segs);
    }

    // 5. Bottom edge: leftwards
    perimeter.push_back({right - rBR, bottom, 0.0f});
    perimeter.push_back({left  + rBL, bottom, 0.0f});

    // 6. Bottom-left corner: CW arc from bottom (-π/2) to left (-π)
    {
        const math::Vec3f center = {left + rBL, bottom + rBL, 0.0f};
        emit_arc(perimeter, center, rBL,
                 -std::numbers::pi_v<float> / 2.0f,               // -90° (bottom)
                 -std::numbers::pi_v<float>,                      // -180° (left)
                 segs);
    }

    // 7. Left edge: upwards
    perimeter.push_back({left, top - rTL, 0.0f});

    // 8. Top-left corner: CW arc from left (-π) to top (-3π/2)
    {
        const math::Vec3f center = {left + rTL, top - rTL, 0.0f};
        emit_arc(perimeter, center, rTL,
                 -std::numbers::pi_v<float>,                      // -180° (left)
                 -3.0f * std::numbers::pi_v<float> / 2.0f,        // -270° (top)
                 segs);
    }

    // ── Build mesh: triangle fan from center ──
    MeshBuilder builder;
    builder.reserve(perimeter.size() + 1, perimeter.size() * 3);

    Vertex cv;
    cv.position = {0.0f, 0.0f, 0.0f};
    cv.normal   = {0.0f, 0.0f, 1.0f};
    cv.uv       = {0.5f, 0.5f, 0.0f};
    cv.color    = geom.color;
    const uint32_t centerIdx = builder.add_vertex(cv);

    std::vector<uint32_t> permIndices;
    permIndices.reserve(perimeter.size());
    for (const auto& p : perimeter)
    {
        Vertex v;
        v.position = p;
        v.normal   = {0.0f, 0.0f, 1.0f};
        v.uv = {
            (p.x + hw) / geom.size.x,
            (p.y + hh) / geom.size.y,
            0.0f
        };
        v.color = geom.color;
        permIndices.push_back(builder.add_vertex(v));
    }

    // Triangle fan.
    // Perimeter is CW, so (center, perm[i+1], perm[i]) produces CCW winding
    // for front-facing (Z+) triangles.
    for (size_t i = 0; i < permIndices.size(); ++i)
    {
        const size_t j = (i + 1) % permIndices.size();
        builder.add_triangle(centerIdx, permIndices[j], permIndices[i]);
    }

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
}

} // namespace exd::geometry

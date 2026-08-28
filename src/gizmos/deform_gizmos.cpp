#include <exd/geometry/gizmos.hpp>

#include "gizmo_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace exd::geometry
{

using namespace exd::geometry::detail;

namespace
{
constexpr float kPi = 3.14159265358979323846f;

// Bend-plane direction selection — same rule as DeformDescriptor::bendDirection.
math::Vec3f pick_bend_direction(const math::Vec3f& ax, const math::Vec3f& explicitDir)
{
    if (explicitDir.length() > 1e-6f)
        return explicitDir.normalized();

    math::Vec3f ref = {0, 1, 0};
    if (std::abs(ax.dot(ref)) > 0.999f)
        ref = {0, 0, 1};
    return (ref - ax * ref.dot(ax)).normalized();
}

// Normalize an axis, falling back to {0,1,0} for a zero-length input.
math::Vec3f normalize_axis(const math::Vec3f& ax)
{
    if (ax.length() < 1e-8f)
        return {0, 1, 0};
    return ax.normalized();
}

// Torus-arc head cone sitting beyond the arc end, pointing along the tangent.
MeshData arc_head(const math::Vec3f& zeroDir, const math::Vec3f& z1,
                  float radius, float thetaEnd, float arrowRadius, float arrowLength,
                  uint32_t tubeSegs, const math::Quat& color)
{
    math::Vec3f endPoint = zeroDir * (std::cos(thetaEnd) * radius)
                         + z1 * (std::sin(thetaEnd) * radius);
    math::Vec3f tEnd = -zeroDir * std::sin(thetaEnd) + z1 * std::cos(thetaEnd);
    math::Vec3f tip  = endPoint + tEnd * arrowLength;
    // No base cap: the cone buries into the arc tube.
    return build_cone(endPoint, tip, arrowRadius, tubeSegs, color, /*baseCap=*/false);
}
} // namespace

// ── Bend ────────────────────────────────────────────────────────────────────

GizmoParts generate_bend_gizmo(const BendGizmoGeometry& geometry)
{
    if (geometry.radius <= 0.0f)
        return {};

    const uint32_t arcSegs  = std::max<uint32_t>(8, geometry.arcSegments);
    const uint32_t tubeSegs = std::max<uint32_t>(6, geometry.tubeSegments);

    const math::Vec3f ax = normalize_axis(geometry.bendAxis);
    const math::Vec3f u  = pick_bend_direction(ax, geometry.bendDirection);
    const math::Vec3f z1 = ax.cross(u);

    // Grabbable sweep floor: keep at least 0.35 rad of arc on screen.
    float sweep = std::abs(geometry.angle);
    if (sweep < 0.35f)
        sweep = 0.35f;

    GizmoParts parts;

    // Part 0: thin axis line along bendAxis.
    if (geometry.shaftRadius > 0.0f)
    {
        parts.push_back(make_part(GizmoAxis::None, GizmoPartKind::Shaft, 0,
            build_capped_cylinder(ax * (-geometry.shaftLength * 0.5f),
                                  ax * ( geometry.shaftLength * 0.5f),
                                  geometry.shaftRadius, 10, geometry.color)));
    }

    // Part 1: arc tube following the bend spine circle (through origin, centered
    // at -radius*u). build_torus_arc centers the ring at the origin, so shift it
    // so the ring matches the deform_mesh spine circle.
    if (geometry.tubeRadius > 0.0f)
    {
        MeshData arc = build_torus_arc(ax, u, geometry.radius, geometry.tubeRadius,
                                       -sweep * 0.5f, sweep, arcSegs, tubeSegs, geometry.color);
        const math::Vec3f center = -geometry.radius * u;
        for (auto& v : arc.vertices)
            v.position += center;
        arc.bounds = compute_bounds(arc.vertices);

        parts.push_back(make_part(GizmoAxis::None, GizmoPartKind::Arc, 1, std::move(arc)));

        // Part 2: arrow head at the (translated) arc end.
        const float endA = sweep * 0.5f;
        math::Vec3f endPoint = u * ((std::cos(endA) - 1.0f) * geometry.radius)
                             + z1 * ( std::sin(endA)        * geometry.radius);
        math::Vec3f tEnd = -u * std::sin(endA) + z1 * std::cos(endA);
        math::Vec3f tip  = endPoint + tEnd * geometry.arrowLength;
        parts.push_back(make_part(GizmoAxis::None, GizmoPartKind::ArrowHead, 2,
            build_cone(endPoint, tip, geometry.arrowRadius, tubeSegs, geometry.color, /*baseCap=*/false)));
    }

    return parts;
}

// ── Twist ───────────────────────────────────────────────────────────────────

GizmoParts generate_twist_gizmo(const TwistGizmoGeometry& geometry)
{
    if (geometry.radius <= 0.0f)
        return {};
    if (geometry.tubeRadius <= 0.0f)
        return {};

    const uint32_t arcSegs  = std::max<uint32_t>(8, geometry.arcSegments);
    const uint32_t tubeSegs = std::max<uint32_t>(6, geometry.tubeSegments);

    const math::Vec3f ax = normalize_axis(geometry.twistAxis);
    const detail::AxisFrame fr = detail::make_frame(ax);
    const math::Vec3f zeroDir = fr.right;
    const math::Vec3f z1 = ax.cross(zeroDir);

    // Signed sweep: both arrows show the rotation direction; negative reverses it.
    const int   sign  = geometry.angle >= 0.0f ? 1 : -1;
    const float maxSweep = 0.45f * 2.0f * kPi; // ~2.8274 rad
    const float sweep = std::clamp(std::abs(geometry.angle), 0.35f, maxSweep);

    const float startA = (sign > 0) ? 0.0f : -sweep;      // Arc A
    const float startB = (sign > 0) ? kPi : kPi - sweep;  // Arc B (opposite side)

    GizmoParts parts;

    // Part 0: arc A.
    parts.push_back(make_part(GizmoAxis::None, GizmoPartKind::Arc, 0,
        build_torus_arc(ax, zeroDir, geometry.radius, geometry.tubeRadius,
                        startA, sweep, arcSegs, tubeSegs, geometry.color)));
    // Part 1: head A.
    parts.push_back(make_part(GizmoAxis::None, GizmoPartKind::ArrowHead, 1,
        arc_head(zeroDir, z1, geometry.radius, startA + sweep,
                 geometry.arrowRadius, geometry.arrowLength, tubeSegs, geometry.color)));
    // Part 2: arc B.
    parts.push_back(make_part(GizmoAxis::None, GizmoPartKind::Arc, 2,
        build_torus_arc(ax, zeroDir, geometry.radius, geometry.tubeRadius,
                        startB, sweep, arcSegs, tubeSegs, geometry.color)));
    // Part 3: head B.
    parts.push_back(make_part(GizmoAxis::None, GizmoPartKind::ArrowHead, 3,
        arc_head(zeroDir, z1, geometry.radius, startB + sweep,
                 geometry.arrowRadius, geometry.arrowLength, tubeSegs, geometry.color)));

    return parts;
}

// ── Taper ───────────────────────────────────────────────────────────────────

GizmoParts generate_taper_gizmo(const TaperGizmoGeometry& geometry)
{
    if (geometry.length <= 0.0f || geometry.frameThickness <= 0.0f)
        return {};

    const uint32_t tubeSegs = std::max<uint32_t>(6, geometry.tubeSegments);

    const math::Vec3f ax = normalize_axis(geometry.taperAxis);
    const detail::AxisFrame fr = detail::make_frame(ax);

    const float startScale = std::max(1e-3f, geometry.startScale);
    const float endScale   = std::max(1e-3f, geometry.endScale);
    const float startHalf  = geometry.baseFrameSize * startScale * 0.5f;
    const float endHalf    = geometry.baseFrameSize * endScale * 0.5f;

    // Corner signatures {±1, ±1} in the (right, up) in-plane basis.
    const std::array<std::pair<int, int>, 4> corners = {{
        { 1,  1}, {-1,  1}, {-1, -1}, { 1, -1},
    }};

    auto cornerPos = [&](float yPos, float half, int si, int sj)
    {
        return ax * yPos
             + fr.right * (half * static_cast<float>(si))
             + fr.up    * (half * static_cast<float>(sj));
    };

    // Square-frame mesh: 4 perimeter tubes + 4 corner knobs.
    auto buildFrame = [&](float yPos, float half)
    {
        MeshData m;
        for (int i = 0; i < 4; ++i)
        {
            const auto& c0 = corners[i];
            const auto& c1 = corners[(i + 1) % 4];
            m = detail::concat_meshes(std::move(m),
                detail::build_capped_cylinder(cornerPos(yPos, half, c0.first, c0.second),
                                              cornerPos(yPos, half, c1.first, c1.second),
                                              geometry.frameThickness, tubeSegs, geometry.color));
        }
        for (int i = 0; i < 4; ++i)
        {
            const auto& c = corners[i];
            MeshData knob = detail::build_icosphere(geometry.frameThickness, geometry.color, 1);
            const math::Vec3f center = cornerPos(yPos, half, c.first, c.second);
            for (auto& v : knob.vertices)
                v.position += center;
            knob.bounds = compute_bounds(knob.vertices);
            m = detail::concat_meshes(std::move(m), std::move(knob));
        }
        m.topology = PrimitiveTopology::Triangles;
        m.bounds   = compute_bounds(m.vertices);
        return m;
    };

    // Connectors: 4 edge tubes between the frame corners.
    MeshData connectors;
    for (int i = 0; i < 4; ++i)
    {
        const auto& c = corners[i];
        connectors = detail::concat_meshes(std::move(connectors),
            detail::build_capped_cylinder(cornerPos(-geometry.length * 0.5f, startHalf, c.first, c.second),
                                          cornerPos( geometry.length * 0.5f, endHalf,   c.first, c.second),
                                          geometry.connectorRadius, tubeSegs, geometry.color));
    }

    GizmoParts parts;
    parts.push_back(detail::make_part(GizmoAxis::None, GizmoPartKind::Frame, 0,
        buildFrame(-geometry.length * 0.5f, startHalf)));
    parts.push_back(detail::make_part(GizmoAxis::None, GizmoPartKind::Frame, 1,
        buildFrame( geometry.length * 0.5f, endHalf)));
    parts.push_back(detail::make_part(GizmoAxis::None, GizmoPartKind::Connector, 2,
        std::move(connectors)));

    return parts;
}

// ── Lattice ─────────────────────────────────────────────────────────────────

GizmoParts generate_lattice_gizmo(const LatticeCageGeometry& geometry)
{
    if (geometry.grid.x < 2 || geometry.grid.y < 2 || geometry.grid.z < 2)
        return {};
    if (geometry.size.x <= 0.0f || geometry.size.y <= 0.0f || geometry.size.z <= 0.0f)
        return {};

    const int nx = geometry.grid.x;
    const int ny = geometry.grid.y;
    const int nz = geometry.grid.z;

    auto nodePos = [&](int i, int j, int k)
    {
        const float fx = static_cast<float>(i) / static_cast<float>(nx - 1) - 0.5f;
        const float fy = static_cast<float>(j) / static_cast<float>(ny - 1) - 0.5f;
        const float fz = static_cast<float>(k) / static_cast<float>(nz - 1) - 0.5f;
        return math::Vec3f{fx * geometry.size.x, fy * geometry.size.y, fz * geometry.size.z};
    };

    GizmoParts parts;

    // One ControlPoint per grid node; partId = linear index (X fastest).
    for (int k = 0; k < nz; ++k)
    {
        for (int j = 0; j < ny; ++j)
        {
            for (int i = 0; i < nx; ++i)
            {
                MeshData point = detail::build_icosphere(geometry.pointRadius, geometry.pointColor, 2);
                const math::Vec3f center = nodePos(i, j, k);
                for (auto& v : point.vertices)
                    v.position += center;
                point.bounds = compute_bounds(point.vertices);

                const uint32_t partId = static_cast<uint32_t>(i + j * nx + k * nx * ny);
                parts.push_back(detail::make_part(GizmoAxis::None, GizmoPartKind::ControlPoint, partId,
                                                  std::move(point)));
            }
        }
    }

    // Connector: all grid edge tubes merged into one part (partId 0).
    MeshData edges;
    auto addEdge = [&](const math::Vec3f& a, const math::Vec3f& b)
    {
        edges = detail::concat_meshes(std::move(edges),
            detail::build_capped_cylinder(a, b, geometry.edgeRadius, 6, geometry.edgeColor));
    };

    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx - 1; ++i)
                addEdge(nodePos(i, j, k), nodePos(i + 1, j, k));

    for (int k = 0; k < nz; ++k)
        for (int i = 0; i < nx; ++i)
            for (int j = 0; j < ny - 1; ++j)
                addEdge(nodePos(i, j, k), nodePos(i, j + 1, k));

    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
            for (int k = 0; k < nz - 1; ++k)
                addEdge(nodePos(i, j, k), nodePos(i, j, k + 1));

    parts.push_back(detail::make_part(GizmoAxis::None, GizmoPartKind::Connector, 0,
                                      std::move(edges)));

    return parts;
}

} // namespace exd::geometry
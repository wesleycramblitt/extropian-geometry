#include <exd/geometry/gizmos.hpp>

#include "gizmo_internal.hpp"

#include <algorithm>
#include <cstdint>

namespace exd::geometry
{

using namespace exd::geometry::detail;

GizmoParts generate_rotation_gizmo(const RotationGizmoGeometry& geometry)
{
    if (geometry.radius <= 0.0f || geometry.ringThickness <= 0.0f)
        return {};

    constexpr float twoPi = 6.28318530717958647692f;

    const uint32_t ringSegs = std::max<uint32_t>(12, geometry.ringSegments);
    const uint32_t tubeSegs = std::max<uint32_t>(6, geometry.tubeSegments);

    const math::Vec3f axes[3]     = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const math::Vec3f zeroDirs[3] = {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}}; // X→+Y, Y→+Z, Z→+X

    GizmoParts parts;

    // 0-2 Ring X/Y/Z.
    for (uint32_t i = 0; i < 3; ++i)
    {
        const math::Vec3f& axis  = axes[i];
        const math::Vec3f& z0    = zeroDirs[i];
        const math::Quat&  color = geometry.colors[i];

        parts.push_back(make_part(static_cast<GizmoAxis>(i), GizmoPartKind::Ring, i,
            build_torus_arc(axis, z0, geometry.radius, geometry.ringThickness,
                            0.0f, twoPi, ringSegs, tubeSegs, color)));
    }

    // 3-5 Knob X/Y/Z (grab sphere at the ring's angle-0 point, ON the ring circle).
    if (geometry.knob)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            const math::Quat& color = geometry.colors[i];
            const math::Vec3f z0    = zeroDirs[i];

            MeshData knob = build_icosphere(geometry.knobRadius, color, 2);
            const math::Vec3f offset = geometry.radius * z0;
            for (auto& v : knob.vertices)
                v.position += offset;
            knob.bounds = compute_bounds(knob.vertices);

            parts.push_back(make_part(static_cast<GizmoAxis>(i), GizmoPartKind::Knob, 3 + i,
                                      std::move(knob)));
        }
    }

    return parts;
}

} // namespace exd::geometry
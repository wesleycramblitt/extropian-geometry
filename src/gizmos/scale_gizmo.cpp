#include <exd/geometry/gizmos.hpp>

#include "gizmo_internal.hpp"

#include <algorithm>
#include <cstdint>

namespace exd::geometry
{

using namespace exd::geometry::detail;

GizmoParts generate_scale_gizmo(const ScaleGizmoGeometry& geometry)
{
    if (geometry.length <= 0.0f)
        return {};
    if (geometry.shaftRadius <= 0.0f && geometry.cubeSize <= 0.0f)
        return {};

    const uint32_t slices = std::max<uint32_t>(6, geometry.slices);
    const float length    = geometry.length;

    const math::Vec3f axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    const bool hasCube  = geometry.cubeSize > 0.0f;
    const bool hasShaft = geometry.shaftRadius > 0.0f;

    GizmoParts parts;

    // 0-2 Shaft X/Y/Z.
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (!hasShaft)
            break;

        const math::Vec3f& dir   = axes[i];
        const math::Quat&  color = geometry.colors[i];

        if (hasCube && geometry.cubeSize >= length)
            continue; // cube spans the whole axis: shaft omitted

        const float shaftEnd = hasCube ? (length - geometry.cubeSize) : length;
        if (shaftEnd <= 1e-6f)
            continue;

        parts.push_back(make_part(static_cast<GizmoAxis>(i), GizmoPartKind::Shaft, i,
                                  build_capped_cylinder({0, 0, 0}, dir * shaftEnd,
                                                        geometry.shaftRadius, slices, color)));
    }

    // 3-5 Handle (cube) X/Y/Z.
    if (hasCube)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            const math::Vec3f& dir   = axes[i];
            const math::Quat&  color = geometry.colors[i];
            const float half = geometry.cubeSize * 0.5f;

            math::Vec3f center;
            if (geometry.cubeSize >= length)
                center = dir * (length * 0.5f); // cube spans the whole axis
            else
                center = dir * (length - geometry.cubeSize * 0.5f);

            parts.push_back(make_part(static_cast<GizmoAxis>(i), GizmoPartKind::Handle, 3 + i,
                                      build_box(center, {half, half, half}, color)));
        }
    }

    return parts;
}

} // namespace exd::geometry
#include <exd/geometry/gizmos.hpp>

#include "gizmo_internal.hpp"

#include <algorithm>
#include <cstdint>

namespace exd::geometry
{

using namespace exd::geometry::detail;

GizmoParts generate_translation_gizmo(const TranslationGizmoGeometry& geometry)
{
    if (geometry.length <= 0.0f)
        return {};
    if (geometry.shaftRadius <= 0.0f && geometry.coneRadius <= 0.0f)
        return {};

    const uint32_t slices = std::max<uint32_t>(6, geometry.slices);
    const float length    = geometry.length;

    const math::Vec3f axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    // Cone head length is clamped to at most half the axis length.
    const float coneLen = std::min(geometry.coneLength, length * 0.5f);
    const bool  hasCone = (coneLen > 0.0f && geometry.coneRadius > 0.0f);
    const bool  hasShaft = geometry.shaftRadius > 0.0f;

    GizmoParts parts;

    // 0-2 Shaft X/Y/Z.
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (!hasShaft)
            break;

        const math::Vec3f& dir   = axes[i];
        const math::Quat&  color = geometry.colors[i];
        const float shaftEnd = hasCone ? (length - coneLen) : length;

        if (shaftEnd > 1e-6f)
            parts.push_back(make_part(static_cast<GizmoAxis>(i), GizmoPartKind::Shaft, i,
                                      build_capped_cylinder({0, 0, 0}, dir * shaftEnd,
                                                            geometry.shaftRadius, slices, color)));
    }

    // 3-5 Handle (cone) X/Y/Z.
    if (hasCone)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            const math::Vec3f& dir   = axes[i];
            const math::Quat&  color = geometry.colors[i];
            const float shaftEnd = length - coneLen;

            MeshData handle;
            if (geometry.coneFillet)
                handle = build_cone_with_fillet(dir * shaftEnd, dir, dir * length,
                                                geometry.coneRadius, slices, color);
            else
                handle = build_cone(dir * shaftEnd, dir * length,
                                    geometry.coneRadius, slices, color);

            parts.push_back(make_part(static_cast<GizmoAxis>(i), GizmoPartKind::Handle, 3 + i,
                                      std::move(handle)));
        }
    }

    return parts;
}

} // namespace exd::geometry
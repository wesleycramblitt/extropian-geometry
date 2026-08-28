#pragma once

#include <exd/geometry/part.hpp>
#include <exd/math/vec3.hpp>
#include <exd/math/quat.hpp>

#include <cstdint>
#include <vector>

namespace exd::geometry {

/// Loft / skin between cross-sections. Sections are planar or non-planar
/// point rings in world space; the surface skins corresponding points with
/// quads (turbine-blade style). All sections MUST have the same point count.
/// Orientation/twist is authored by the caller's section layout (rotate a
/// section's ring to twist the loft); no frame machinery.
struct LoftGeometry
{
    std::vector<std::vector<math::Vec3f>> sections; // >= 2 sections, equal counts, >= 3 points
    bool   capped = true;     // triangulated start/end caps
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

/// Skinned mesh: wall quads between sections (+ optional triangular caps).
/// Degenerate input (fewer than 2 sections, mismatched or < 3 point counts,
/// coincident consecutive sections) → empty MeshData.
MeshData generate_loft_mesh(const LoftGeometry& geometry);

/// Tier-2 part generator: patches "wall" (skin), "cap_start" (first section,
/// normal opposed to the spine), "cap_end" (last section) — only when capped.
Part generate_loft_part(const LoftGeometry& geometry);

/// Resample a closed ring polygon by arc length to exactly `count` points
/// (for feeding sections of unequal sizes into the loft). count < 3 or
/// points.size() < 3 → empty. First point stays at the original first point.
std::vector<math::Vec3f> resample_ring(
    const std::vector<math::Vec3f>& points, uint32_t count);

} // namespace exd::geometry
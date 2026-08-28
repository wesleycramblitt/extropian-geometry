#pragma once

#include <exd/geometry/part.hpp>
#include <exd/geometry/turbine.hpp>

#include <cstdint>

namespace exd::geometry::detail {

/// Per-blade triangle counts inside a blade-row mesh (all blades identical).
struct BladeRowBuildInfo
{
    uint32_t skinPerBlade    = 0;
    uint32_t hubCapPerBlade  = 0;
    uint32_t shroudCapPerBlade = 0;
    uint32_t stridePerBlade  = 0;   // skin + hub + shroud
};

/// Blade-row mesh builder shared by the turbine and compressor recipes.
/// Output must remain BIT-IDENTICAL to today's generate_blade_row_mesh.
/// When `info` is non-null the per-blade counts are recorded.
MeshData build_blade_row_impl(const BladeRow& row, const FlowPath& flow,
                              uint32_t revolve_segments,
                              BladeRowBuildInfo* info);

/// Meridional profile (x = r, y = z) for a flow-path surface, exactly as
/// generate_flow_path_mesh derives it from the control points (SAME spline
/// evaluation/refinement). Returns the profile points in the same order
/// (increasing z). Empty input → empty result.
std::vector<math::Vec3f> build_meridional_profile(const std::vector<math::Vec2f>& points);

/// Lowercase role name for a blade row ("stator", "rotor", "nozzle", "diffuser").
const char* blade_row_role_name(BladeRowType type);

} // namespace exd::geometry::detail
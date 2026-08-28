#pragma once

#include <exd/geometry/types.hpp>
#include <exd/geometry/part.hpp>
#include <exd/math/vec2.hpp>
#include <exd/math/vec3.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace exd::geometry
{

// ═══════════════════════════════════════════════════════════════════════════
//  Universal parametric turbine model.
//
//  A turbine is described as a set of axisymmetric flow-path surfaces (hub +
//  shroud splines in the (z, r) meridional plane) plus a sequence of blade
//  rows. The same abstraction covers axial, radial, and mixed-flow machines;
//  what changes is how the hub/shroud radii evolve along z.
//
//  Every parameter is a TurbineParam: a value with bounds, a unit, and a
//  locked/optimizable flag, so an optimizer can sweep the design space without
//  a per-machine-type schema.
//
//  Orientation: generated meshes place the machine centered on the origin with
//  its rotating axis along WORLD -Z, so a camera looking down -Z sees the
//  turbine head-on when it is placed at the origin.
// ═══════════════════════════════════════════════════════════════════════════

/// A single bounded, unit-annotated parameter. `locked == true` marks a value
/// that must not be varied by an optimizer.
struct TurbineParam
{
    float value = 0.0f;
    float min   = 0.0f;
    float max   = 0.0f;
    std::string unit;
    bool locked = false;
};

enum class BladeRowType { Stator, Rotor, Nozzle, Diffuser };
enum class StackingRule { LeadingEdge, TrailingEdge, Centroid, QuarterChord };
enum class TipFeature   { None, Clearance, Squealer, Shroud };

/// Solid center-body (hub) profile families. Different shapes map onto
/// different machine kinds: wind-turbine spinner, bullet-shaped high-speed
/// nose, plain cylindrical core, double-tapered spindle, or a thin disc rotor.
enum class HubShape { None, Spinner, Bullet, Cylinder, Tapered, FlatDisk };

/// Axisymmetric flow path: the rotating-axis (z, r, θ) frame is implicit; the
/// hub and shroud are monotone splines r(z) given by their control points.
struct FlowPath
{
    std::vector<math::Vec2f> hub_points;      // (z, r) control points, increasing z
    std::vector<math::Vec2f> shroud_points;   // (z, r) control points, increasing z

    // Domain inlet / outlet stations (z, r); empty = derive from hub/shroud ends.
    std::vector<math::Vec2f> inlet_station;
    std::vector<math::Vec2f> outlet_station;

    // Tip clearance is first-class: it materially affects efficiency.
    TurbineParam tip_clearance{0.001f, 0.0f, 0.02f, "m", false};
    // Endwall features (data; optional geometry).
    TurbineParam fillet_radius{0.0f, 0.0f, 0.05f, "m", true};
    TurbineParam surface_roughness{0.0f, 0.0f, 0.001f, "m", true};
};

/// A 2D blade section at one span fraction (0 = hub, 1 = shroud).
struct BladeSection
{
    float span = 0.0f;

    TurbineParam chord{0.05f, 0.001f, 0.5f, "m", false};
    // Stagger rotates the section about its LE in the (axial, tangential)
    // plane. ANY angle is valid: |stagger| > 90 flips the section (reversed
    // or feathered blades), 90 puts it edge-on. Sense convention: for
    // |stagger| < 90 deg the camber's convex (suction) side faces +theta
    // (positive rotation about +Z). A rotor spinning counterclockwise when
    // viewed from the upwind end (+Z) with the convex side at +theta
    // extracts energy from an axial flow (turbine sense, as long as the
    // relative flow still runs LE -> TE); reversing the spin makes the same
    // blade set behave as a fan.
    TurbineParam stagger{0.0f, -360.0f, 360.0f, "deg", false};
    TurbineParam inlet_metal_angle{0.0f, -90.0f, 90.0f, "deg", false};
    TurbineParam exit_metal_angle{0.0f, -90.0f, 90.0f, "deg", false};

    // Thickness-to-chord ratio and its chordwise location.
    TurbineParam max_thickness{0.10f, 0.001f, 0.9f, "t/c", false};
    TurbineParam max_thickness_location{0.30f, 0.05f, 0.9f, "x/c", false};

    TurbineParam leading_edge_radius{0.002f, 0.0f, 0.02f, "m", false};
    TurbineParam trailing_edge_radius{0.001f, 0.0f, 0.01f, "m", false};

    // Optional camber / thickness distribution control points (x/c, y/c).
    // Empty -> derived from the metal angles (camber) and NACA thickness.
    std::vector<math::Vec2f> camber_line;
    std::vector<math::Vec2f> thickness_distribution;
};

/// A stator / nozzle / rotor / diffuser row.
struct BladeRow
{
    BladeRowType type = BladeRowType::Stator;

    // Rows are revolved about +Z; a positive rotation (viewed from +Z) is
    // counterclockwise, which pairs with a positive-cambered section into
    // the turbine sense described on BladeSection::stagger.
    TurbineParam blade_count{24, 1, 200, "", false};
    TurbineParam rotational_speed{0, 0, 100000, "rpm", true};  // rotor only

    // Leading / trailing edge locations on hub and shroud, as (z, r).
    math::Vec2f leading_edge_hub{0.0f, 0.0f};
    math::Vec2f leading_edge_shroud{0.0f, 0.0f};
    math::Vec2f trailing_edge_hub{0.0f, 0.0f};
    math::Vec2f trailing_edge_shroud{0.0f, 0.0f};

    // Spanwise sections (0..1). If empty, sections are synthesized at
    // 0 / 25 / 50 / 75 / 100% span from the row's LE/TE geometry.
    std::vector<BladeSection> sections;

    StackingRule stacking = StackingRule::Centroid;
    TurbineParam sweep{0.0f, -1.0f, 1.0f, "m", true};  // axial displacement along span
    TurbineParam lean{0.0f, -1.0f, 1.0f, "m", true};   // circumferential along span

    TipFeature tip_feature = TipFeature::None;
    TurbineParam root_fillet{0.0f, 0.0f, 0.05f, "m", true};

    uint32_t chordwise_points = 24;   // points per section profile
};

/// Solid hub / center body sitting at the rotor plane (z = 0, +Z forward).
/// Shape-specific fields:
///   • root_radius  — radius at the rotor plane, where the blade roots attach
///   • front_length — axial extent forward of the rotor plane (+Z)
///   • aft_length   — axial extent behind the rotor plane (-Z)
///   • aft_radius   — radius of the trailing end (Tapered only; 0 = pointed)
///   • nose_power   — Bullet nose sharpness: 1 = cone, <1 = pointed bullet
/// The meridional profile is revolved about the machine axis (Z).
struct HubDefinition
{
    HubShape shape = HubShape::Spinner;
    float root_radius  = 0.35f;   // blade-root radius                    [m]
    float front_length = 0.60f;   // nose extent forward of rotor plane   [m]
    float aft_length   = 0.40f;   // body extent behind rotor plane       [m]
    float aft_radius   = 0.0f;    // trailing-end radius (Tapered; 0 = point) [m]
    float nose_power   = 0.65f;   // nose curve exponent (Bullet)         [-]
    uint32_t profile_points = 16; // samples along the curved portions
};

/// A complete machine: one flow path plus any number of blade rows.
struct TurbineDefinition
{
    FlowPath flow_path;
    std::vector<BladeRow> blade_rows;
    /// Optional solid center body (HubShape::None -> no hub mesh).
    HubDefinition hub;
    uint32_t revolve_segments = 64;
};

// ── Generators ──────────────────────────────────────────────────────────────

/// Revolve the hub and shroud splines into axisymmetric surface meshes
/// (a merged annulus of two open surfaces).
MeshData generate_flow_path_mesh(const FlowPath& flow, uint32_t revolve_segments = 64);

/// Build a 2D blade-section profile: a closed polygon in (x, y) with x along
/// the chord (0..chord) and y the tangential offset. CCW order.
std::vector<math::Vec2f> generate_blade_section_profile(
    const BladeSection& section, float chord_length, uint32_t points = 48);

/// Loft the spanwise sections into a single closed blade, positioned in the
/// cylindrical (z, r, θ) frame defined by the flow path. Returns one blade
/// (no copy-rotation); stagger, stacking, sweep, lean, and tip clearance are
/// applied.
MeshData generate_blade_row_mesh(const BladeRow& row, const FlowPath& flow,
                                 uint32_t revolve_segments = 64);

/// Build the solid center body (hub) as a revolved, capped profile in the
/// same frame as the blade rows: rotor plane at z = 0, axis along Z.
MeshData generate_hub_mesh(const HubDefinition& hub, uint32_t revolve_segments = 64);

/// Assemble the whole machine: flow path plus every blade row, Z-folded.
MeshData generate_turbine_mesh(const TurbineDefinition& turbine);

/// Assemble the machine as an Assembly of named, patched Parts — the
/// simulation contract: complex engineering generators MUST expose patched
/// parts. Parts: "hub" (patch "surface"), "flow_path" (patch "surface"),
/// and one Part per blade row named "<role>_<rowIndex>" (e.g. "rotor_0",
/// "stator_1") with patches "blade_surface", "hub_cap", "shroud_cap" (all
/// blades of the row folded into these three patches).
Assembly generate_turbine_assembly(const TurbineDefinition& turbine);

} // namespace exd::geometry

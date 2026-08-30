#pragma once

#include <exd/geometry/types.hpp>
#include <exd/geometry/part.hpp>
#include <exd/geometry/mechanism.hpp>

#include <cstdint>

namespace exd::geometry
{

// ═══════════════════════════════════════════════════════════════════════════
//  Single-cylinder reciprocating steam engine (recipe, V1).
//
//  A state-parametric machine assembly: the mechanism (inline crank-slider)
//  is evaluated at generation time for the requested crank angle, so a solver
//  or animation layer sweeps crank_angle_deg without touching this code.
//
//  Layout (world frame, SI meters):
//    • Crankshaft along world +Z through the crank centre C = (crank_x, 0, 0).
//    • Cylinder axis along +X, lying in the plane z = rod_plane_z. The
//      crosshead slides on the line (x, 0, rod_plane_z).
//    • Crank pin orbits about C with radius crank_radius, spanning z ∈
//      [pin_z_start, pin_z_start + pin_length]; θ = crank_angle_deg turns
//      counterclockwise about +Z.
//        pin(θ)   = (crank_x + r·cos θ, r·sin θ, pin_z_start + ½·pin_length)
//        x_c(θ)   = crank_x + r·cos θ + sqrt(L² − r²·sin²θ)   (crosshead)
//        x_pk(θ)  = x_c(θ) − piston_rod_length                (piston crown)
//    • The rod plane (cylinder axis, piston rod, crosshead) sits at
//      z = rod_plane_z, ABOVE the pin's z span — so at TDC/BDC the pin can
//      never pierce the rod. The conrod therefore runs skewed (out of the
//      rod plane) from the crosshead down to the pin, like a real engine's
//      offset linkage.
//    • The flywheel (with a V-groove pulley rim — the mechanical power
//      takeoff) and the crankshaft sit at the crank centre; the pin emerges
//      from the flywheel's +z face and carries the conrod big end.
//
//  Mechanism (joints, parent → child, parent-frame anchor/axis):
//    shaft      Continuous  world → crankshaft    anchor (crank_x,0,0), +Z
//    fly_fix    Fixed       crankshaft → flywheel anchor (0,0,0)
//    pin_fix    Fixed       flywheel → crank_pin  anchor (rc, 0, 0.06)
//    conrod_pin Revolute    crank_pin → conrod    anchor (0,0,0), +Z
//    piston_sl  Prismatic   cylinder → piston     anchor (x_c(0),0,rod_plane_z), +X
//    cross_fix  Fixed       piston → crosshead    anchor (0,0,0)
//    conrod_cs  Revolute    crosshead → conrod    anchor (0,0,0), +Z (LOOP:
//                second incoming joint on conrod — exported as a connect)
//  Driver: `shaft` (a motor on the crank for simulation sweeps).
//
//  Parts produced by generate_steam_engine_assembly (V1 contract):
//    cylinder    — blind bore with a recessed chamber: outer wall, solid
//                  head disc, crank-end annulus, chamber bore wall and
//                  chamber bottom; patches "wall", "bore", "cap_head",
//                  "cap_crank". The chamber mouth at the crank end (r < bore)
//                  is an EXTERIOR region (torus-hole style): the surface is a
//                  closed manifold with NO boundary loops, and the piston rod
//                  passes through the mouth as a separate interpenetrating
//                  part (assembly placement, not CSG).
//    steam_chest — box on top of the cylinder (patches +x −x +y −y +z −z).
//    steam_inlet / steam_exhaust — capped tubular stubs on the chest top
//                  (patches "surface", "cap_start", "cap_end").
//    piston      — crown disc, body, step ring, rod to the crosshead;
//                  patches "crown" (pressure face) and "wall".
//    crosshead   — sliding block at x_c(θ); box patches.
//    conrod      — circular-section rod posed from the crosshead to the pin
//                  center (skewed out of the rod plane); patches "wall",
//                  "cap_start" (small end), "cap_end" (big end).
//    flywheel    — lathe disc + V-groove rim at the crank centre; patches
//                  "rim", "face_start", "face_end".
//    crank_pin   — capped cylinder pinned through the flywheel at
//                  (crank_x + r·cos θ, r·sin θ, rod_plane_z); patches
//                  "surface", "cap_start", "cap_end".
//    crankshaft  — capped journal through the flywheel hub along +Z;
//                  patches "journal", "cap_start", "cap_end".
//
//  All parts are individually closed watertight solids, oriented outward
//  (positive signed volume; verified by the boolean closed-manifold gate in
//  the tests). Parts may intersect each other (the pin passes through the
//  flywheel disc; the journal is embedded in the wheel hub) — assembly
//  placement, not boolean union.
//
//  Validation (invalid input → empty Assembly / empty MeshData, the repo
//  convention): crank_angle_deg NaN/Inf; crank_radius <= 0; conrod_length <=
//  crank_radius; cylinder_outer_radius <= 0; cylinder_bore_radius <= 0 or >=
//  outer; piston_radius <= 0 or >= bore; piston_rod_length <= piston_length;
//  flywheel_rim_radius <= 0, groove radius <= 0 or >= rim; rod_plane_z <=
//  pin_z_start + pin_length (the pin would cross the rod plane).
//  Deterministic: identical descriptor → identical mesh every call.
// ═══════════════════════════════════════════════════════════════════════════

/// State-parametric single-cylinder steam engine descriptor. SI units
/// (radii/lengths in meters), all fields defaulted.
struct SteamEngineDefinition
{
    // ── State ──
    /// Mechanism pose evaluated at generation time [deg, 0..360]. Positive
    /// turns the crank counterclockwise viewed from +Z; θ = 90 puts the pin
    /// at +y. Not normalized internally — any value is accepted.
    float crank_angle_deg = 0.0f;

    // ── Mechanism ──
    float crank_radius      = 0.10f;   // [m] pin orbit radius
    float conrod_length     = 0.45f;   // [m] crosshead → pin; must exceed crank_radius
    float piston_rod_length = 1.10f;   // [m] crosshead centre → piston crown
    float crank_center_x    = 0.45f;   // [m] crank centre along the cylinder axis

    // ── Cylinder ──
    float cylinder_length      = 0.34f;   // [m] head plane → crank-end plane
    float cylinder_outer_radius = 0.09f;  // [m]
    float cylinder_bore_radius = 0.07f;   // [m] inner wall (rod aperture at r <= 0.02)
    float cylinder_crank_end_x = -0.02f;  // [m] crank-end face plane; head at − length

    // ── Piston + rod ──
    float piston_radius = 0.065f;  // [m] < cylinder_bore_radius
    float piston_length = 0.05f;   // [m] crown → rod step
    float rod_radius    = 0.008f;  // [m] piston rod

    // ── Crosshead ──
    float crosshead_size      = 0.05f;  // [m] block extent in x and y
    float crosshead_thickness = 0.04f;  // [m] block extent in z

    // ── Conrod ──
    float conrod_radius = 0.018f;  // [m] circular section

    // ── Flywheel (V-groove pulley disc, the power takeoff) ──
    float flywheel_rim_radius   = 0.21f;   // [m] outer radius
    float flywheel_thickness    = 0.06f;   // [m] total axial extent
    float flywheel_groove_radius = 0.185f; // [m] V-groove bottom radius
    float flywheel_groove_half_width = 0.010f; // [m] rim face → groove slant

    // ── Crankshaft + pin ──
    float shaft_radius      = 0.04f;  // [m] < flywheel_hub_radius
    float shaft_half_length = 0.10f;  // [m] journal extent either side of the flywheel
    float pin_radius        = 0.014f; // [m]
    float pin_z_start       = 0.030f; // [m] pin z extent start (on the flywheel +z face)
    float pin_length        = 0.06f;  // [m]

    // ── Steam chest + ports ──
    float chest_width   = 0.12f;   // [m] x extent, centered on chest_x_center
    float chest_height  = 0.08f;   // [m] y extent, sitting on the cylinder top
    float chest_depth   = 0.06f;   // [m] z extent
    float chest_x_center = -0.26f; // [m] chest centre along the cylinder axis
    float port_radius   = 0.018f;  // [m]
    float port_height   = 0.06f;   // [m] above the chest top
    float port_spacing  = 0.06f;   // [m] between inlet and exhaust axes

    // ── Shared ──
    float    rod_plane_z     = 0.105f;  // [m] rod/crosshead plane — ABOVE the pin's z span
    uint32_t revolve_segments = 64;     // lathe subdivision count (clamped to >= 3)
};

/// The recipe contract (roadmap §3): one descriptor yields the rendered
/// Assembly, the body-local parts, AND the Mechanism — constraints travel
/// with the geometry. `mechanism` declares the joints/couplings for
/// to_mjcf()/to_urdf(); `body` are the parts in their local frames (origin =
/// joint anchor, exactly what the exporters and evaluate_poses() expect);
/// `assembly` is the world-posed render (the recipe solves the slider-crank
/// loop analytically; the exported mechanism carries the same loop as
/// constraints for the simulator).
struct SteamEngineResult
{
    Assembly assembly;
    Mechanism mechanism;
    std::vector<Part> body;
};

SteamEngineResult generate_steam_engine(const SteamEngineDefinition& engine);

/// The world-posed assembly (renderable); equals generate_steam_engine(...).
///assembly.
Assembly generate_steam_engine_assembly(const SteamEngineDefinition& engine);

/// Convenience: flatten(generate_steam_engine_assembly(...)).mesh
/// (empty if the assembly is empty).
MeshData generate_steam_engine_mesh(const SteamEngineDefinition& engine);

} // namespace exd::geometry

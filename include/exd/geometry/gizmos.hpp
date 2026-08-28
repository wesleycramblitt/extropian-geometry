#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>
#include <exd/math/quat.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace exd::geometry {

// ── Gizmo part taxonomy ──

/// Interaction axis of a gizmo part. Values are documented to map by NAME to
/// the editor pick-target enum in extropian-render (exd::render::interaction::GizmoAxis);
/// do not rely on numeric values across libraries.
enum class GizmoAxis : uint8_t
{
    X = 0,
    Y = 1,
    Z = 2,
    None = 0xFF, // generic part (deform gizmos with arbitrary axes)
};

/// Role of a gizmo part. Use `kind` to apply per-role pick slop (e.g. shafts are
/// thin, so downstream raycasters should inflate their hit radius).
enum class GizmoPartKind : uint8_t
{
    Shaft,        // thick axis line (cylinder)
    Handle,       // translate: cone tip; scale: cube tip
    Ring,         // rotation: full thick torus circle
    Knob,         // rotation: grab sphere on the ring
    Arc,          // bend/twist: thick tube arc
    ArrowHead,    // bend/twist: cone at arc end
    Frame,        // taper: square tube frame at an extent end
    Connector,    // taper: edge tubes; lattice: grid edge tubes
    ControlPoint, // lattice: icosphere knob
};

/// A single independently-pickable section of a gizmo.
/// `partId` is stable per generator and documented in each generator's header
/// comment; downstream picking keys on (axis, kind, partId).
struct GizmoPart
{
    GizmoAxis     axis   = GizmoAxis::None;
    GizmoPartKind kind   = GizmoPartKind::Shaft;
    uint32_t      partId = 0;
    MeshData      mesh;
};

using GizmoParts = std::vector<GizmoPart>;

/// Concatenate all part meshes into one MeshData (same topology contract as
/// merge_meshes). Empty input → empty MeshData.
MeshData merge_gizmo_parts(const GizmoParts& parts);

/// Copy of all parts matching the given kind, optionally restricted to an axis.
GizmoParts filter_gizmo_parts(const GizmoParts& parts, GizmoPartKind kind,
                              GizmoAxis axis = GizmoAxis::None);

// ── Translation ──

/// Translate gizmo. Parts (in order): 0-2 Shaft X/Y/Z, 3-5 Handle (cone) X/Y/Z.
struct TranslationGizmoGeometry
{
    float   length      = 1.0f;   // total axis length (origin → tip)
    float   shaftRadius = 0.02f;
    float   coneRadius  = 0.07f;
    float   coneLength  = 0.18f;
    uint32_t slices     = 24;     // cross-section segments, clamped to >= 6
    bool    coneFillet  = true;   // rounded bead torus at cone base (merged into Handle part)
    std::array<math::Quat, 3> colors = {{
        {1.0f, 0.0f, 0.0f, 1.0f}, // X red
        {0.0f, 1.0f, 0.0f, 1.0f}, // Y green
        {0.0f, 0.0f, 1.0f, 1.0f}, // Z blue
    }};
};
GizmoParts generate_translation_gizmo(const TranslationGizmoGeometry& geometry);

// ── Scale ──

/// Scale gizmo. Parts (in order): 0-2 Shaft X/Y/Z, 3-5 Handle (cube) X/Y/Z.
struct ScaleGizmoGeometry
{
    float   length      = 1.0f;
    float   shaftRadius = 0.02f;
    float   cubeSize    = 0.12f;  // cube handle edge length at each axis end
    uint32_t slices     = 24;
    std::array<math::Quat, 3> colors = {{{1,0,0,1},{0,1,0,1},{0,0,1,1}}};
};
GizmoParts generate_scale_gizmo(const ScaleGizmoGeometry& geometry);

// ── Rotation ──

/// Rotation gizmo. Parts (in order): 0-2 Ring X/Y/Z (thick torus), 3-5 Knob
/// X/Y/Z. Ring "angle 0" convention (knob azimuth): X ring at +Y,
/// Y ring at +Z, Z ring at +X (cyclic X→Y→Z→X) so no two knobs coincide.
struct RotationGizmoGeometry
{
    float    radius        = 1.0f;
    float    ringThickness = 0.035f; // torus tube radius
    uint32_t ringSegments  = 96;     // around the ring, clamped to >= 12
    uint32_t tubeSegments  = 16;     // tube cross-section, clamped to >= 6
    bool     knob          = true;   // grab sphere at ring angle 0
    float    knobRadius    = 0.04f;  // grab sphere radius
    std::array<math::Quat, 3> colors = {{{1,0,0,1},{0,1,0,1},{0,0,1,1}}};
};
GizmoParts generate_rotation_gizmo(const RotationGizmoGeometry& geometry);

// ── Deform gizmos (mirror DeformDescriptor semantics) ──

/// Bend gizmo. Parts: 0 Shaft (axis line along bendAxis), 1 Arc (thick tube arc
/// of radius `radius` in the (bendAxis, bendDirection) plane), 2 ArrowHead at
/// the arc end. The arc spans the object's full bend range: from
/// -angle/2 to +angle/2, matching the reworked deform_mesh bend. bendDirection
/// follows the same auto rule as DeformDescriptor::bendDirection (0 → auto).
struct BendGizmoGeometry
{
    math::Vec3f bendAxis      = {0, 1, 0};
    math::Vec3f bendDirection = {0, 0, 0}; // auto when zero-length
    float       radius        = 1.0f;      // arc radius = bendRadius
    float       angle         = 1.5707963f; // total bend angle radians
    float       shaftLength   = 0.6f;      // thin axis line extent (±/2)
    float       shaftRadius   = 0.015f;
    float       tubeRadius    = 0.04f;     // arc tube radius
    uint32_t    arcSegments   = 72;        // along the arc, clamped to >= 8
    uint32_t    tubeSegments  = 12;        // arc tube cross-section, clamped to >= 6
    float       arrowRadius   = 0.06f;
    float       arrowLength   = 0.12f;
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};
GizmoParts generate_bend_gizmo(const BendGizmoGeometry& geometry);

/// Twist gizmo. Parts: 0 Arc A, 1 ArrowHead A, 2 Arc B, 3 ArrowHead B (drawn on
/// opposite sides of the axis, both arrows showing the same rotation direction).
/// Each arc spans `angle` (clamped to [0.35 rad minimum visual sweep, 0.45*2pi])
/// so the handles stay grabbable at angle 0. Negative angle reverses direction.
struct TwistGizmoGeometry
{
    math::Vec3f twistAxis    = {0, 1, 0};
    float       radius       = 1.0f;
    float       angle        = 1.5707963f; // current twist angle (radians)
    float       tubeRadius   = 0.03f;
    uint32_t    arcSegments  = 64;   // per arc, clamped to >= 8
    uint32_t    tubeSegments = 12;   // clamped to >= 6
    float       arrowRadius  = 0.05f;
    float       arrowLength  = 0.1f;
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};
GizmoParts generate_twist_gizmo(const TwistGizmoGeometry& geometry);

/// Taper gizmo. Parts: 0 Frame (start, t=0), 1 Frame (end, t=1), 2 Connector
/// (4 edge tubes joining frame corners).
/// CONTRACT: deform_mesh() normalizes deformation coordinates by the object's
/// GLOBAL max half-extent and centers on the bounds center. Set `length` to
/// 2*maxExtent of the target object for the frames to sit at the true
/// taper endpoints. Frame half-size = baseFrameSize * scale / 2.
struct TaperGizmoGeometry
{
    math::Vec3f taperAxis   = {0, 1, 0};
    float       length      = 1.0f;      // extent along taperAxis
    float       startScale  = 1.0f;
    float       endScale    = 0.5f;
    float       baseFrameSize = 0.3f;    // frame side at scale = 1
    float       frameThickness = 0.03f;  // frame tube radius
    float       connectorRadius = 0.015f;
    uint32_t    tubeSegments = 10;       // clamped to >= 6
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};
GizmoParts generate_taper_gizmo(const TaperGizmoGeometry& geometry);

/// Lattice cage (FFD-style; deform_mesh FFD is planned, so this widget is
/// forward-looking — its visuals are NOT validated against deform output yet).
/// Parts: one ControlPoint per grid node with partId = linear index
/// (i + j*nx + k*nx*ny, X fastest), then 1 Connector part (partId 0) with all
/// grid edge tubes merged.
struct LatticeCageGeometry
{
    math::Vec3i grid = {3, 3, 3};         // nodes per axis; each must be >= 2
    math::Vec3f size = {1.0f, 1.0f, 1.0f}; // cage extent (centered at origin)
    float       pointRadius = 0.045f;
    float       edgeRadius  = 0.012f;
    math::Quat  pointColor = {0.9f, 0.9f, 0.9f, 1.0f};
    math::Quat  edgeColor  = {0.6f, 0.6f, 0.6f, 1.0f};
};
GizmoParts generate_lattice_gizmo(const LatticeCageGeometry& geometry);

} // namespace exd::geometry
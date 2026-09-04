#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/mat4.hpp>

#include <functional>
#include <span>
#include <string>
#include <vector>

namespace exd::geometry {

/// A labelled set of faces — the boundary-condition unit for downstream
/// solvers (CFD/FEA packages call these "patches"). `faces` holds triangle
/// ORDINALS into the owning mesh's index buffer (indices.size()/3).
/// Explicit ordinals rather than ranges: mesh operations remap indices, and
/// explicit lists survive remapping by construction. Generators that emit
/// contiguous runs document the layout and often build via make_patch_range.
/// CAE role of a face set (Phase A, CADModel). Additive semantics on top of
/// the legacy construction-relative patch names ("wall", "cap_start", "+x"…).
/// `Unspecified` keeps legacy patches valid (no declared role).
enum class PatchSemantic
{
    Unspecified,   // no declared role
    Wall,          // solid boundary
    Inlet,
    Outlet,
    Symmetry,
    Fixed,         // structural fixture
    Sliding,
    Interface,     // coupling between domains (see CADModel::interfaces)
    Free           // unconstrained/open (e.g. far-field)
};

struct Patch
{
    std::string name;
    std::vector<uint32_t> faces;
    // CAE/multiphysics additions (Phase A, CADModel). Defaulted so every
    // legacy initializer and make_patch_range() stays source-compatible.
    PatchSemantic semantic   = PatchSemantic::Unspecified;
    float         mesh_size  = 0.0f;   // target cell/edge size hint [m]; 0 = solver default
    std::string   material;            // per-patch material override; empty = part's material
};

/// Coarse motion class for a part (scene layer animation + sim export hints).
/// Body modeling kind for simulation export (deformable-body roadmap).
enum class PartKind
{
    Rigid,       // the default: jointed rigid body
    Deformable   // soft body: boundary mesh + constitutive params; the
                 // simulator tetrahedralizes the surface (MJCF 3.x does this
                 // internally) and welds it to rigid bodies via the part's
                 // contact patches. FK treats it as a rigid placeholder;
                 // the deformed state comes back from the simulator.
};

enum class PartMotion
{
    Stationary,    // welded to the world / parent (static base)
    Rotating,      // e.g. shafts, wheels, flywheels
    Reciprocating, // e.g. pistons, crossheads, sliders
    Oscillating,   // e.g. conrods at both ends free, rockers
};

/// Per-part solver/interaction metadata (roadmap §7 "Part metadata").
/// Deliberately minimal: motion for the scene layer, density for mass
/// properties, `contact` is the EXPLICIT opt-in flag that marks a part's
/// patches for export as collision surfaces (simulations only see parts with
/// contact = true; nothing is assumed).
struct PartMeta
{
    PartKind    kind     = PartKind::Rigid;   // Rigid / Deformable (soft body)
    PartMotion  motion   = PartMotion::Stationary;
    std::string material;                 // empty = default (steel-like)
    float       density  = 1000.0f;       // kg/m³ (water default; override per material)
    bool        contact  = false;         // opt-in collision export of this part's patches
    // Deformable constitutive params (used when kind == Deformable)
    float       youngs_modulus = 0.0f;    // Pa; 0 = simulator default
    float       poisson        = 0.0f;    // −;  0 = simulator default
};

/// A named component: its own local-space mesh, BC patches, and solver
/// metadata. The unit of separation for simulation (rotor vs. casing)
/// and the unit the gizmo layer picks and transforms.
struct Part
{
    std::string name;
    MeshData mesh;
    std::vector<Patch> patches;
    PartMeta meta;                        // motion/density/contact for solvers
};

/// A machine: named Parts kept separate (own meshes, own local patch
/// indices). The correct granularity for per-part materials/motion and for
/// per-part interaction.
struct Assembly
{
    std::vector<Part> parts;
    Bounds bounds;
};

// ── Construction helpers ──

/// Wrap a MeshData into a Part with no patches.
Part as_part(std::string name, MeshData mesh);

/// Patch over a contiguous run of faces [firstFace, firstFace + faceCount).
/// No validation (caller guarantees the range fits the mesh).
Patch make_patch_range(std::string name, uint32_t firstFace, uint32_t faceCount);

/// Tag every face f in [0, indices.size()/3) for which pred(f) returns true
/// (predicate receives triangle ordinals; capture the part's mesh in the
/// lambda to inspect geometry). Appends faces to the patch of the same name,
/// creating it if absent. Assumes Triangles topology.
void tag_faces(Part& part, const std::string& name,
               const std::function<bool(uint32_t faceIndex)>& pred);

// ── Patch-preserving operations ──
//
// Preserved: transform_part (indices unchanged), deform_mesh applied to
// part.mesh (indices unchanged — document this in deform.hpp). Remapped:
// flatten (triangle offsets). Lost/undefined: future boolean/weld ops
// (callers re-tag).

/// Transform a Part's mesh; patch face ordinals are unchanged (wholly owned
/// by the mesh's index buffer, which transform_mesh preserves).
Part transform_part(const Part& part, const math::Mat4& transform,
                    bool transformNormals = true);

/// Concatenate parts into an Assembly; each part keeps its own mesh and its
/// LOCAL patch ordinals. bounds covers all part meshes.
Assembly merge_parts(std::span<const Part> parts);

/// Fold an Assembly into a single Part (merge_meshes + remap patch ordinals
/// by cumulative triangle offsets). Patch names are prefixed with the owning
/// part's name ("rotor_0.blade_surface") to stay unique; parts with empty
/// names keep their patch names unprefixed. A single-part assembly is
/// returned unchanged. Empty assembly → Part{}.
Part flatten(const Assembly& assembly);

} // namespace exd::geometry
#pragma once

#include <exd/geometry/mechanism.hpp>
#include <exd/geometry/cad_model.hpp>

#include <map>
#include <span>
#include <string>

namespace exd::geometry
{

// ═══════════════════════════════════════════════════════════════════════════
//  Constraint export — serialize {Assembly parts + Mechanism} for physics
//  simulators (roadmap D9).
//
//  • to_mjcf() is the PRIMARY export: gears/belts/rack survive via
//    <equality> joint constraints and tendons; joints/limits/actuation are
//    native; inertials are emitted from mesh_properties() (D10); the MuJoCo
//    compiler can tetrahedralize deformable boundary meshes (PartKind::Deformable).
//  • to_urdf() is the SECONDARY export: widest importer coverage; gears map
//    to <mimic> (position-level), kinematic loops cannot be expressed.
//
//  Frame contract: parts MUST be body-local (their local origin is their
//  joint anchor), exactly as evaluate_poses()/MJCF expect. Meshes are
//  emitted as one Wavefront OBJ per part (the bundle's `meshes` map).
//  Deterministic: same inputs → byte-identical output.
// ═══════════════════════════════════════════════════════════════════════════

struct ExportOptions
{
    float default_density = 1000.0f;   // kg/m³ when part.meta.density == 0
    bool  emit_inertials  = true;      // URDF requires them; MJCF takes ours (D10)
    bool  emit_contact    = true;      // parts with meta.contact=true → collision group
    std::string model_name = "model";
};

/// A serialized model: one format's XML plus one OBJ mesh per part.
struct ExportBundle
{
    std::string xml;
    std::map<std::string, std::string> meshes;   // part name → OBJ text
};

/// Wavefront OBJ (positions + normals; triangle indices, 1-based).
std::string to_obj(const MeshData& mesh);

/// Primary export: MuJoCo MJCF model XML (reference: mujoco.readthedocs.io).
ExportBundle to_mjcf(const Mechanism& mech, std::span<const Part> parts,
                     const ExportOptions& options = {});

/// Secondary export: URDF robot description XML (REP-103 SI units).
ExportBundle to_urdf(const Mechanism& mech, std::span<const Part> parts,
                     const ExportOptions& options = {});

/// CADModel overloads: per-part density is resolved through the model's
/// MaterialDB (parts with an unresolved/empty `meta.material` keep their
/// declared density, falling back to `options.default_density`). Mechanism is
/// read from `model.mechanism`; both exporters keep all other behaviour.
ExportBundle to_mjcf(const CADModel& model, const ExportOptions& options = {});
ExportBundle to_urdf(const CADModel& model, const ExportOptions& options = {});


// ═══════════════════════════════════════════════════════════════════════════
//  CAE adapters (Phase B, docs/cad-model.md §5) — deterministic writers over
//  MeshData / CADModel. Same input → byte-identical output.
//
//  • STL: binary is the CAE-default tessellation carrier (ascii for humans).
//  • Gmsh .msh: every surface triangle is in a Physical group — first matching
//    patch ("part.patch") else the part ("part"); maps 1:1 onto the IR.
//  • VTK/VTU: per-cell PartID/PatchID scalars for region-aware post-processing.
//  • STEP: AP203/AP214 faceted solid B-rep (watertight manifold BREP) — one
//    advanced BREP per part; imports into mainstream CAD/CAE. Exact AP242
//    tessellated entities deferred to the OCCT gate (docs/cad-model.md D17).
// ═══════════════════════════════════════════════════════════════════════════

/// Wavefront OBJ (positions + normals; triangle indices, 1-based).
/// Concatenated per part for models.
std::string to_obj(const CADModel& model);

/// STL ascii — "solid <name>" per part.
std::string to_stl_ascii(const MeshData& mesh);
std::string to_stl_ascii(const CADModel& model);

/// STL binary (80-byte header, uint32 count, per-triangle 3×float32 vertices +
/// 1×float32 normal + uint16 attribute). Little-endian IEEE754. CAE default.
std::string to_stl_binary(const MeshData& mesh);
std::string to_stl_binary(const CADModel& model);

/// Gmsh .msh 2.2 (ascii). Triangles topology only; other topologies skipped.
/// Physical surfaces: "part" and "part.patch"; vertices deduplicated per part.
std::string to_msh(const CADModel& model);

/// VTK legacy unstructured grid (ascii) + VTK XML .vtu (ascii arrays).
/// Scalars: PartID (0-based part index) and PatchID (1-based global patch
/// enumeration, 0 = face with no patch).
std::string to_vtk(const CADModel& model);
std::string to_vtu(const CADModel& model);

/// STEP faceted solid B-rep export options.
struct StepOptions
{
    std::string  schema      = "AUTOMOTIVE_DESIGN";       // FILE_SCHEMA name (AP242 via OCCT sibling later)
    std::string  model_name  = "model";
};

/// ISO 10303-21 text; one ADVANCED_BREP_SHAPE_REPRESENTATION per part.
/// Only watertight (closed manifold) parts are emitted (the boolean gate's
/// definition of solid); other parts are skipped.
std::string to_step_faceted(const CADModel& model, const StepOptions& options = {});

} // namespace exd::geometry

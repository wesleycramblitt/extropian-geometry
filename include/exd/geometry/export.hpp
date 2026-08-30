#pragma once

#include <exd/geometry/mechanism.hpp>

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

} // namespace exd::geometry

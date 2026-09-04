#pragma once

#include <exd/geometry/cad_model.hpp>

#include <string>

namespace exd::geometry
{

// ═══════════════════════════════════════════════════════════════════════════
//  Analytic BREP import/export — optional OpenCASCADE (OCCT) module (D17/D18).
//
//  The unified CADModel IR handles tessellated formats here; STEP/IGES/BREP
//  are analytic-kernel territory and should not be hand-rolled. When built
//  with `-DEXD_GEOMETRY_ENABLE_OCCT=ON` this module reads and writes analytic
//  solids through OCCT; otherwise a compiling stub reports the missing
//  backend (WASM-clean default preserved).
//
//  Import: solids → BRepMesh tessellation → body-local surface parts
//    (watertight, smooth normals) → CADModel.
//  Export: watertight parts (geometry's closed_manifold_gate) → sewn solids
//    → one STEP file (STEPControl_Writer).
// ═══════════════════════════════════════════════════════════════════════════

/// Whether the OCCT backend is compiled in (EXD_GEOMETRY_HAS_OCCT == 1).
bool occt_available();

/// OpenCASCADE version string, or "none (stub build)".
std::string occt_version();

enum class BrepImportFormat
{
    Step,
    Iges,
    Brep
};

struct BrepImportResult
{
    bool ok = false;
    std::string error;                  // actionable message when !ok
    CADModel model;                     // populated when ok
};

/// Import an analytic B-rep file as a CADModel (each SOLID → one Part, named
/// "solid_1…" — XCAF product-name mapping is a follow-up).
BrepImportResult import_brep(const std::string& path, BrepImportFormat format);

/// Convenience: pick the format from the file extension
/// (.step/.stp, .iges/.igs, anything else → Brep).
BrepImportResult import_brep_file(const std::string& path);

/// Export watertight CADModel parts into one analytic STEP file. Non-watertight
/// parts are skipped. Returns false + error message on failure.
bool export_brep_step(const CADModel& model, const std::string& path, std::string& error);
bool export_brep_step(const CADModel& model, const std::string& path);

} // namespace exd::geometry

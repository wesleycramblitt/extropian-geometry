#pragma once

#include <exd/geometry/cad_model.hpp>
#include <exd/geometry/types.hpp>

#include <string>

namespace exd::geometry
{

// ═══════════════════════════════════════════════════════════════════════════
//  Import — the inverse of the exporters (docs/cad-model.md §6, Phase C).
//  Every importer produces CADModel (single-part models wrap the MeshData).
//  In-repo scope is tessellated/lightweight formats:
//     • OBJ     — positions (+ optional normals), triangle/fan faces
//     • STL     — ascii or binary (auto-detected by content)
//     • Gmsh    — .msh 2.2 ascii; PhysicalSurfaces → Patches 1:1, elementary
//                 regions → Parts (round-trips to_msh exactly).
//  Analytic BREP (STEP/IGES/BREP) is NOT parsed here — it is owned by the
//  OCCT-based extropian-cad sibling (D18). All importers are deterministic
//  and dependency-free.
// ═══════════════════════════════════════════════════════════════════════════

/// Parse Wavefront OBJ (0-based triangle indices; quads fan-triangulated).
/// Unknown/unsupported entities are ignored.
MeshData parse_obj(const std::string& text);

/// Parse STL ascii or binary (auto-detected: leading "solid" → ascii, else
/// binary when (size − 84) is a multiple of 50). Triangle soup — no welding.
MeshData parse_stl(const std::string& data);

/// Single-part imports (convenience over the parsers).
CADModel import_obj(const std::string& text, const std::string& name = "obj_import");
CADModel import_stl(const std::string& data, const std::string& name = "stl_import");

/// Gmsh .msh 2.2 (ascii). Triangle elements (type 2) only; other element
/// types are ignored. Physical surface groups map onto patches:
///   • "part.patch"  → patch "patch" of part "part"
///   • "part"        → part-level (unpatched faces)
///   • unnamed faces → part "part_<elemtag>"
/// Part name precedence: dot-name prefix > part-level group name > "part_N".
CADModel import_msh(const std::string& text, const std::string& name = "msh_import");

} // namespace exd::geometry

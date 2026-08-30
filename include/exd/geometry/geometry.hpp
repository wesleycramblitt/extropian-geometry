#pragma once

// ── Geometry types (shared across ecosystem) ──
#include <exd/geometry/types.hpp>

// ── Construction ──
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>

// ── Parts & assemblies ──
#include <exd/geometry/part.hpp>

// ── Loft ──
#include <exd/geometry/loft.hpp>

// ── Generators ──
#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/primitives3d.hpp>

// ── Vector graphics ──
#include <exd/geometry/path.hpp>

// ── Text ──
#include <exd/geometry/text.hpp>
#include <exd/geometry/font.hpp>

// ── SDF Blending ──
#include <exd/geometry/blend.hpp>

// ── Curves ──
#include <exd/geometry/spline.hpp>

// ── Parametric machines ──
#include <exd/geometry/turbine.hpp>

// ── Machines ──
#include <exd/geometry/compressor.hpp>
#include <exd/geometry/steam_engine.hpp>

// ── Motion graph (connectivity core) ──
#include <exd/geometry/mechanism.hpp>

// ── Constraint export (MJCF / URDF) ──
#include <exd/geometry/export.hpp>

// ── Noise / procedural terrain ──
#include <exd/geometry/noise.hpp>
#include <exd/geometry/terrain.hpp>

// ── Advanced generators ──
#include <exd/geometry/extrusion.hpp>
#include <exd/geometry/heightmap.hpp>
#include <exd/geometry/deform.hpp>

// ── Gizmos ──
#include <exd/geometry/gizmos.hpp>

# extropian-geometry

Renderer-independent mesh generation and geometric computation for the Extropian
ecosystem. Depends only on `extropian-core` (math types, mesh types); text
shaping additionally uses FreeType + HarfBuzz.

## Contents

- **2D primitives** — rect, rounded rect, circle, ellipse, arc, ring, line,
  polyline, arrow, grid, star, regular polygon.
- **3D primitives** — sphere, box, cylinder, plane, capsule, icosahedron,
  torus, cone, tube, disk, arrow, axes, billboard, ellipsoid.
- **Mesh ops** — merge, transform, bounds; incremental `MeshBuilder`.
- **Parts & patches** — `Part`/`Patch`/`Assembly` with labelled face sets for boundary conditions; native patches on cylinder/cone/box/extrusion/lathe; turbine assemblies with per-row patched parts.
- **Loft / processing** — loft/skin between sections, polygon triangulation (concave + holes), vertex welding, normal recomputation.
- **Machines** — compressor recipe: staged axial machine as a patched Assembly (casing, spinner, rotor/stator rows with boundary patches); single-cylinder **steam engine recipe**: state-parametric crank-slider assembly (cylinder with blind-bore chamber, steam chest + ports, piston, crosshead, conrod, flywheel with V-groove pulley takeoff, crank pin, crankshaft) returned as {Assembly, Mechanism} for any crank angle.
- **Connectivity & sim export** — motion graph: fixed/revolute/continuous/prismatic joints with limits + compliance, gear/belt/rack couplings, tree FK and validation; mass/inertia properties (`mesh_properties`); **MJCF** (primary: equality joints for gears/welds, actuation, deformables) and **URDF** (mimic gears) exporters — constraints travel with the geometry for MuJoCo/Drake/Gazebo/PyBullet-class consumers.
- **Unified model (CADModel)** — one format-agnostic in-memory IR per part/machine: parts, patches (+ CAE semantics/mesh targets), in-repo material registry, multiphysics regions/domains/interfaces, physics-tagged BCs and body loads, symmetry/cyclic/rotating frames, plus the mechanism. File formats are adapters over it (see [docs/cad-model.md](docs/cad-model.md)).
- **Design docs** — domain-recipe roadmap in [docs/recipes.md](docs/recipes.md); connectivity/sim-export design record in [docs/mechanisms.md](docs/mechanisms.md); unified-model/CAE design record in [docs/cad-model.md](docs/cad-model.md).
- **Booleans** — exact CSG union/subtract/intersect on closed meshes (watertight-gated; coplanar overlap is a documented V1 limitation).
- **Path / blend / SDF** — path following, signed-distance-field blending,
  marching cubes, extrusion, heightmaps, deformation.
- **Gizmos** — per-part translation/rotation/scale interaction gizmos plus
  bend/twist/taper/lattice deform gizmos (GizmoParts).
- **Text** — FreeType/HarfBuzz glyph meshing and font handling.

## Parametric shape strategy

The library currently provides deterministic generators for common primitives,
extrusions, lathes, tubes, SDF blends, deformations, terrain, and specialized
models such as turbines. It is not yet a complete general-purpose solid
modeling kernel.

The intended direction is a layered system:

1. **Generic geometry kernel** — profiles, curves, paths, parametric surfaces,
   scalar fields, frames, sweeps, lofts, tessellation, and mesh operations.
2. **Construction operators** — extrusion, revolve, arbitrary profile sweep,
   loft/skin, implicit-field extraction, and deformation.
3. **Domain recipes** — turbine, heart, wing, tree, shell, gear, and similar
   models expressed using the generic kernel.

Consequently, a complex object does not necessarily need a bespoke mesher. A
stylized heart could be composed from SDF primitives, while an anatomically
plausible heart would likely use a domain-specific `HeartGeometry` descriptor
built on shared loft, implicit-surface, deformation, and mesh-processing tools.
Specialized models remain appropriate when the parameters have meaningful
domain semantics, as they do for turbine blade rows, span, stagger, and tip
clearance.

The main gaps before the library can create most arbitrary 3D forms are
generalized profile sweep, loft/skin, parametric-surface tessellation, custom
scalar fields, robust polygon triangulation, mesh validation/diagnostics,
normal recomputation, vertex welding, and mesh booleans. See
[`docs/plan.md`](docs/plan.md) for the detailed assessment and roadmap.

## Build

```bash
./build.sh          # library
./test.sh           # library + unit tests
./demo.sh           # optional 3D viewer (fetches extropian-render)
```

Override the core dependency path for local development:

```bash
EXD_CORE_DIR=../extropian-core ./build.sh
```

The library target is `exd-geometry` (alias `exd::geometry`), headers under
`exd/geometry/...`.

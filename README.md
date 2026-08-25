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
- **Path / blend / SDF** — path following, signed-distance-field blending,
  marching cubes, extrusion, heightmaps, deformation.
- **Text** — FreeType/HarfBuzz glyph meshing and font handling.

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

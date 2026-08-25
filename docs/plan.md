# Extropian Geometry — Mesh Generation Library

> Renderer-independent mesh generation and geometric computation.
> Pure CPU math. Compiles to WASM with zero changes.
>
> **Updated 2026-08 — semantic illustration engine.** geometry is the geometry
> layer of the 2D "semantic illustration engine" (see `modules/scene` and
> `modules/scene_renderer`). New priorities: a **first-class TextEngine** (measurement,
> rich spans), a **Math typesetting AST** (§16), and a strong **vector-path
> system** (§12) — these are what make dense poster-quality explainers possible.
> The WebGL path replaces the deprecated `extropian-web-ui` DOM renderer; text
> renders through glyph-atlas/SDF meshes here, not DOM.

## 1. Purpose

Geometry is a **pure computational library**. It generates `exd::geometry::MeshData` from structured descriptors. It has no GPU dependency, no ECS dependency, and no renderer dependency.

Geometry answers:

> Given a geometric description, what is the resulting mesh?

Geometry does NOT answer:

- How to display the mesh (renderer's job)
- How to compose the mesh into a UI component (the `ui` module's job — dual-platform)
- How to lay out UI elements (the `layout` module / compiler's job)
- What the mesh means semantically (composer's job)

## 2. Position in the Architecture

```
Application/upstream producer → VisualDocument → scene_renderer
│
├── ui (sibling module) ───────┐  (button, panel, slider meshes)
│   UI component meshes        │
│                              │
└── geometry ◄─────────────────┘  (THIS MODULE)
    │  MeshData, Vertex, Bounds types
    │  2D/3D primitive generators
    │  Path2D tessellation
    │  Text shaping + glyph mesh gen
    │  Mesh operations (merge, transform)
    │
    └── extropian-core
        Vec3f, Quat, math utilities
```

Geometry is a **leaf dependency**. The `ui` module depends on it. The desktop path
(and the planned WASM/WebGL path) use it for mesh generation. It compiles to WASM
with zero changes — pure math, no platform dependencies.

## 3. Design Principles

- **Pure functions.** Descriptor in, `MeshData` out. No side effects.
- **No allocations hidden in state.** Caller owns the output `MeshData`.
- **No GPU.** No OpenGL headers, no Vulkan, no buffer objects.
- **No ECS.** No registry, no components, no systems.
- **Deterministic.** Same descriptor → same mesh every time.
- **Configurable quality.** Segment counts, tessellation tolerances.
- **Smooth by default.** Default segment counts target medium-to-high-definition
  meshes suitable for direct display at typical viewport sizes. A 64-segment
  circle at 500px radius has chord deviation under 0.5 pixels — below the
  threshold of visible faceting on any display. Renderers should not need to
  subdivide or post-process shapes for smoothness.
- **2D shapes are 3D-ready.** All 2D primitives live on the XY plane (Z=0)
  with CCW winding and +Z normals, making them valid profile inputs for
  `generate_extrusion_mesh()` without rebuilding geometry. This is the
  contract the `ui` module relies on for extruded components like Panel.
- **Depends only on core.** `Vec3f`, `Quat`, and basic math utilities.

### 3a. Segment Count Guidelines

Default segment counts are chosen so that faceting is invisible at normal
viewport sizes. Callers can override for performance or extreme zoom:

| Shape | Default | When to increase |
|---|---|---|
| Circle / Ellipse / Ring / Arc | 64 | >500px radius at high DPI |
| Rounded rect corners | 32 per corner | >250px corner radius |
| 3D cylinder / cone / capsule slices | 64 | >500px diameter |
| 3D sphere lat/long | 32/64 | >500px diameter |
| Torus major/minor | 64/32 | Large torus close to camera |
| Capsule stacks | 16 | Fine for most cases |

These defaults produce meshes in the range of ~5–15KB per primitive — well
within budget for scenes with hundreds of shapes, even on WASM. Segment count
is the simplest, most portable smoothness mechanism: no special shaders, no
parallel API surface, works identically everywhere.

## 4. Primitive Types

### 2D Primitives

| Primitive | Geometry Descriptor | Generator |
|---|---|---|
| Rectangle | `RectangleGeometry` | `generate_rect_mesh()` |
| Rounded Rectangle | `RoundedRectangleGeometry` | `generate_rounded_rect_mesh()` |
| Circle | `CircleGeometry` | `generate_circle_mesh()` |
| Ellipse | `EllipseGeometry` | `generate_ellipse_mesh()` |
| Arc | `ArcGeometry` | `generate_arc_mesh()` |
| Ring | `RingGeometry` | `generate_ring_mesh()` |
| Line | `LineGeometry` | `generate_line_mesh()` |
| Polyline | `PolylineGeometry` | `generate_polyline_mesh()` |
| Arrow | `ArrowGeometry` | `generate_arrow_mesh()` |
| Star | `StarGeometry` | `generate_star_mesh()` |
| Regular Polygon | `RegularPolygonGeometry` | `generate_regular_polygon_mesh()` |
| Grid | `GridGeometry` | `generate_grid_mesh()` |
| Path2D | `Path2D` | `Path2D::tessellateFill/Stroke()` |

**Planned additions (2D):**

| Primitive | Description |
|---|---|
| Capsule 2D (stadium) | Rounded pill shape |
| Pie / Wedge | Arc with inner radius (donut slice) |
| Superellipse | Squircle with adjustable exponent |
| Cross / Plus | Boolean union of two rects |
| Tear drop | Marker shape |
| Crescent / Lune | Moon phase / icon |
| Spiral | Archimedean or logarithmic spiral curve |
| Rounded-rect profile | 2D CCW outline for extrusion input (eliminates manual vertex construction in the `ui` module) |

### 3D Primitives

| Primitive | Geometry Descriptor | Generator |
|---|---|---|
| Sphere | `SphereGeometry` | `generate_sphere_mesh()` |
| Box | `BoxGeometry` | `generate_box_mesh()` |
| Cylinder | `CylinderGeometry` | `generate_cylinder_mesh()` |
| Plane | `PlaneGeometry` | `generate_plane_mesh()` |
| Capsule | `CapsuleGeometry` | `generate_capsule_mesh()` |
| Torus | `TorusGeometry` | `generate_torus_mesh()` |
| Cone | `ConeGeometry` | `generate_cone_mesh()` |
| Tube | `TubeGeometry` | `generate_tube_mesh()` |
| Disk | `DiskGeometry` | `generate_disk_mesh()` |
| Ellipsoid | `EllipsoidGeometry` | `generate_ellipsoid_mesh()` |
| Icosahedron | (free function) | `generate_icosahedron_mesh()` |
| Arrow3D | `Arrow3DGeometry` | `generate_arrow3d_mesh()` |
| Axes (RGB) | `AxesGeometry` | `generate_axes_mesh()` |
| Billboard | `BillboardGeometry` | `generate_billboard_mesh()` |

**Planned additions (3D):**

| Primitive | Description |
|---|---|
| Pyramid / Frustum | Square base tapering to point or top face |
| Prism | N-sided polygon extruded along Z |
| Chamfered Box | Box with beveled edges |
| Rounded Box | Box with filleted edges (SDF or mesh-based) |
| Torus Knot | (p,q) knot on torus surface |
| Superellipsoid | Ellipsoid with adjustable squareness exponent |
| Wedge | Half-box (triangular prism) |
| Parametric Surface | Arbitrary (u,v) → Vec3 function |

### Grids & Structured Geometry

Beyond single primitives — grid systems for data visualization, game maps,
procedural layouts, and scientific rendering.

| Grid Type | Description |
|---|---|
| 3D Cartesian Grid | Array of cells with configurable spacing |
| Hexagonal Grid (2D) | Hex-tiled game maps, honeycomb patterns |
| Triangular Grid (2D) | Subdivision surfaces, LOD terrain |
| Polar Grid | Concentric rings + radial spokes |
| Spherical Grid | Latitude/longitude tessellation of sphere |
| Radial Layout | Points/objects arranged around center |
| Point Cloud / Scatter | Structured array of point sprites |

### Extended Geometry

| Type | Descriptor | Generator |
|---|---|---|
| Extrusion | `ExtrusionGeometry` | `generate_extrusion_mesh()` |
| Lathe | `LatheGeometry` | `generate_lathe_mesh()` |
| Helix | `HelixGeometry` | `generate_helix_mesh()` |
| Heightmap | `Heightmap` | `generate_heightmap_mesh()` |
| Blend SDF | `BlendGeometry` | `generate_blend_mesh()` |
| Deform | `DeformDescriptor` | `deform_mesh()` |
| Text | `TextVisualDescriptor` | `generate_text_mesh()` |

**Planned additions (extended):**

| Type | Description |
|---|---|
| Loft / Skin | Interpolate mesh surface between multiple profile cross-sections |
| Sweep along arbitrary path | Generalized Tube — sweep any profile along any 3D curve |
| Multi-LOD terrain | Quadtree-chunked heightmap with variable resolution |
| Procedural noise heightmap | Generate heightmap data from noise functions (Perlin, simplex, Worley) |

## 5. Mesh Operations

Operations that consume `MeshData` and produce new `MeshData`.
All are pure functions — input unchanged, new output returned.

**Implemented:**

| Operation | Function | Description |
|---|---|---|
| Merge | `merge_meshes()` | Concatenate multiple meshes with index offset |
| Transform | `transform_mesh()` | Apply 4x4 matrix to positions and normals |
| Bounds | `compute_bounds()` | Compute AABB from vertex positions |
| Build | `MeshBuilder` | Incremental construction (add vertex, triangle, quad) |

**Planned:**

| Operation | Priority | Description |
|---|---|---|
| Subdivide | HIGH | Catmull-Clark (quads) and Loop (triangles) subdivision |
| Recompute normals | HIGH | Smooth (angle-weighted), flat, and area-weighted face normals |
| Vertex weld | HIGH | Merge vertices within distance threshold, remap indices |
| Mesh boolean (CSG) | HIGH | Exact union, subtract, intersect on triangle meshes |
| Simplify / decimate | MEDIUM | Reduce triangle count while preserving shape (LOD) |
| Convex hull | MEDIUM | QuickHull or gift-wrapping |
| UV projection | MEDIUM | Planar, cylindrical, spherical automatic UV unwrap |
| Tangent generation | MEDIUM | Compute tangent + bitangent from UVs for normal mapping |
| Edge detection | MEDIUM | Identify boundary edges, hard edges (by angle), non-manifold edges |
| Mesh slice | MEDIUM | Bisect mesh along a plane, produce cross-section outline |
| Mesh statistics | MEDIUM | Volume, surface area, centroid, moment of inertia |
| Remesh | LOW | Isotropic or curvature-adaptive retriangulation |
| Delaunay / Voronoi | LOW | 2D triangulation and Voronoi tessellation of point sets |
| Mesh repair | LOW | Hole filling, normal flipping, non-manifold cleanup |


## 6. Spatial Queries

Lightweight analytical queries that do not require full mesh generation.

**Planned:**

| Query | Description |
|---|---|
| Ray-mesh intersection | Hit test against MeshData, returns distance + triangle |
| Ray-primitive intersection | Analytical hit test against descriptor (sphere, box, etc.) without mesh gen |
| Closest point on mesh | Distance from point to nearest surface point |
| Point containment | Is a point inside a closed mesh? |
| Sphere overlap | Does a sphere intersect a mesh? |
| Mesh AABB tree | Build bounding volume hierarchy for accelerated queries |

## 7. Types

Mesh types are **not defined here** — they live in **extropian-core** so they are shared
across geometry, render, and every other consumer.

- `exd::core::Vertex`, `exd::core::MeshData`, `exd::core::PrimitiveTopology`, `exd::core::Bounds`
  → `extropian-core/include/exd/core/mesh_types.hpp`
- Geometry re-exports them as `exd::geometry::Vertex`, `exd::geometry::MeshData`, etc.
  (see `include/exd/geometry/types.hpp`) for source compatibility.
- Render re-exports them as `exd::render::Vertex`, `exd::render::Mesh`, etc.
  (see `extropian-render/include/exd/render/graphics/mesh.hpp`).

## 8. Platform & Compilation

- Compiles on any C++23 compiler
- **Compiles to WASM via Emscripten with zero changes** — pure CPU math, no GPU, no ECS
- No platform-specific headers or dependencies
- Depends only on `extropian-core` for `Vec3`, `Quat`, and math utilities

## 9. File Layout

```
include/exd/geometry/
├── types.hpp                # Vertex, MeshData, Bounds types
├── mesh_builder.hpp         # Incremental mesh construction
├── mesh_ops.hpp             # Merge, transform, bounds, (planned: subdivide, weld, boolean, etc.)
├── primitives2d.hpp         # 2D geometry descriptors + generators
├── primitives3d.hpp         # 3D geometry descriptors + generators
├── path.hpp                 # Path2D (SVG-like vector paths) + tessellation
├── text.hpp                 # TextVisualDescriptor, TextShaper, glyph mesh gen
├── font.hpp                 # FontAtlas (FreeType rasterization)
├── blend.hpp                # BlendGeometry (SDF blending + marching cubes)
├── extrusion.hpp            # Extrusion, Lathe, Helix
├── deform.hpp               # DeformDescriptor (bend, twist, taper, noise)
├── heightmap.hpp            # Heightmap → terrain mesh
└── geometry.hpp             # Umbrella header (includes all of the above)

src/
├── mesh_builder.cpp
├── mesh_ops.cpp
├── primitives2d/            # One .cpp per 2D primitive
│   ├── rect.cpp, rounded_rect.cpp, circle.cpp, ellipse.cpp, arc.cpp
│   ├── ring.cpp, line.cpp, polyline.cpp, arrow.cpp, grid.cpp
│   └── star.cpp, regular_polygon.cpp
├── primitives3d/            # One .cpp per 3D primitive
│   ├── sphere.cpp, box.cpp, cylinder.cpp, cone.cpp, capsule.cpp
│   ├── torus.cpp, tube.cpp, disk.cpp, plane.cpp, ellipsoid.cpp
│   ├── icosahedron.cpp, arrow3d.cpp, axes.cpp, billboard.cpp
├── path/
│   └── path.cpp
├── text/
│   ├── text.cpp, font.cpp, glyph_mesh.cpp
├── blend/
│   ├── blend.cpp, sdf_primitives.cpp, marching_cubes.cpp
├── extrusion/
│   └── extrusion.cpp
├── heightmap/
│   └── heightmap.cpp
└── deform/
    └── deform.cpp

demo/
├── CMakeLists.txt
└── demo_shapes.cpp         # Visual gallery exercising all generators

tests/
├── CMakeLists.txt
├── snapshot/               # Golden image / binary regression tests (planned)
└── unit/
    ├── primitives2d_test.cpp, primitives3d_test.cpp
    ├── mesh_builder_test.cpp, mesh_ops_test.cpp
    ├── blend_test.cpp, path_test.cpp
    ├── advanced_test.cpp   # Extrusion, lathe, helix, heightmap, deform, SDF ops
    └── text_test.cpp       # Conditional on ENABLE_TEXT
```

## 10. Build & Distribution

- **C++23 required** (uses `std::span`, `std::numbers`, etc.)
- **CMake 3.21+**, Ninja generator recommended
- **Dependencies:**
  - `extropian-core` (FetchContent) — math types, `MeshData`, `Vertex`
  - FreeType + HarfBuzz (system pkg-config) — optional, gated by `ENABLE_TEXT=ON`
- **Library target:** `exd-geometry` (static, aliased `exd::geometry`)
- **Test framework:** doctest (via extropian-core)
- **Demo:** Optional, links `extropian-render` for OpenGL visualization

**Planned improvements:**

| Item | Description |
|---|---|
| `install()` target | Install headers, library, and CMake config for `find_package()` use |
| Package config | `exd-geometryConfig.cmake` for downstream consumption |
| Shared library option | `BUILD_SHARED_LIBS` support |
| CMakePresets.json | Debug/Release/WASM presets |
| WASM toolchain | Emscripten CMake toolchain file + build preset |
| CI/CD | GitHub Actions: lint, build, test, WASM smoke |
| Code hygiene | clang-format, clang-tidy, compiler warning flags |

## 11. Deformation Pipeline

Deformation operators apply post-processing to generated meshes.
All are pure functions — new `MeshData` returned, input unchanged.
Multiple deformations can be chained by calling `deform_mesh()` repeatedly.

**Implemented in `DeformDescriptor`:**

| Operator | Parameters |
|---|---|
| Bend | `bendAngle`, `bendRadius`, `bendAxis` |
| Twist | `twistAngle`, `twistAxis` |
| Taper | `taperStart`, `taperEnd`, `taperAxis` |
| Noise | `noiseAmplitude`, `noiseFrequency`, `noiseSeed` |

**Planned additions:**

| Operator | Description |
|---|---|
| Displacement | Offset vertices along normals by scalar field |
| Inflate / Deflate | Push vertices along normals with signed distance |
| Shear | Matrix shear transform |
| Bulge | Radial inflation around an axis |
| FFD (Free-Form Deformation) | Lattice cage with control points |
| Normal update | Automatically recompute normals after deformation (currently manual) |

## 12. Path System

`Path2D` provides an SVG-like API for 2D vector paths.

**Implemented:**

| Feature | Status |
|---|---|
| `moveTo`, `lineTo` | Done |
| `quadraticTo` (quadratic Bezier) | Done |
| `cubicTo` (cubic Bezier) | Done |
| `arcTo` (circular arc, center + angles) | Done |
| `close()` | Done |
| Fill tessellation (triangle mesh) | Done |
| Stroke tessellation (line quads) | Done |
| `LineJoin::Miter/Round/Bevel` | Done |
| `LineCap::Butt/Round/Square` | Done |
| `FillRule::NonZero` | Done |
| `FillRule::EvenOdd` | Done (struct exists, untested) |
| Revision tracking | Done |

**Planned:**

| Feature | Priority | Description |
|---|---|---|
| Dash pattern rendering | HIGH | `dashPattern` + `dashOffset` fields exist, rendering not implemented |
| Elliptical arc | MEDIUM | SVG-compatible arcTo with sweep-flag and large-arc-flag |
| Path boolean | MEDIUM | Union, subtract, intersect between two Path2Ds before tessellation |
| Path offset | MEDIUM | Inset/outset by distance (contour toolpath) |
| Path simplification | MEDIUM | Ramer–Douglas–Peucker polyline simplification |
| Path length | MEDIUM | Arc-length computation |
| Path from polygon | LOW | Build path from vertex list with configurable joins |

## 13. Text System

Geometry provides the **infrastructure layer** for text: font loading, glyph
rasterization, text shaping, and glyph mesh generation. Higher-level composition
(e.g., UI components with labels, multi-line text blocks) lives in
the `ui` module's `UITextFactory`.

**Implemented (ENABLE_TEXT=ON):**

- `FontAtlas` — FreeType-backed glyph rasterization into RGBA8 texture atlas
- Font loading from file, memory buffer, and system defaults (Sans/Serif/Mono)
- `TextShaper` — HarfBuzz-backed text shaping (glyph positions, layout)
- `TextStyle` — font ID, size, weight, alignment, line height, letter spacing
- `GlyphPlacement` — per-glyph position, size, UV rect
- `generate_glyph_mesh()` — single glyph quad from GlyphPlacement
- `generate_text_mesh()` — combined mesh from ShapedText (single-line)

**Planned (geometry layer):**

| Feature | Priority | Description |
|---|---|---|
| SDF font atlas | MEDIUM | Signed distance field glyphs for resolution-independent text |
| RTL / bidirectional shaping | MEDIUM | Arabic, Hebrew — requires HarfBuzz direction flags in TextShaper |
| Text on path | LOW | Curved glyph placement along a `Path2D` — mesh generation, not layout |

**Out of scope (belongs in the `ui` module):**

| Feature | Description |
|---|---|
| Multi-line text layout | Line breaking, paragraph flow — UI composition concern |
| Rich text (multi-style runs) | `UITextFactory` can call geometry's shaper per-style run |
| UI component text assembly | Button labels, dropdown text, etc. — `UITextFactory` in the `ui` module |
| Atlas overflow handling | ui layer monitors atlas, creates new atlas if full |

### 13b. TextEngine — measurement & rich spans (2D priority, 2026-08)

> The single most important subsystem for the poster path. If text looks bad,
> everything looks bad. This upgrades the current shaper/glyph-mesh layer into a
> full measurement + layout API, so other systems (annotations, connectors,
> layout) can attach to **exact** text geometry.

`TextEngine` is an additive facade over the working `FontAtlas`, `TextShaper`,
`generate_text_mesh()`, and `ui::UITextFactory` stack. It does not replace font
loading, shaping, atlas management, or glyph mesh generation. Its purpose is to
make measurement and semantic layout use the exact same shaping implementation
as rendering.

The public API is not just `drawText(text, x, y)`:

```cpp
namespace exd::geometry {

struct TextRun {                       // rich text is first-class
    std::string text;
    std::string style;                 // resolved typography role (body/emphasis/caption/…)
    std::optional<std::string> semantic_id;  // semantic fragment anchor (e.g. "laplacian")
};

struct TextBlock {
    std::vector<TextRun> runs;
    std::string alignment;             // left | center | right | justify
    float max_width = 0.0f;            // 0 = unbounded
};

struct TextMetrics {
    math::Vec2f size;                  // exact block bounds
    float baseline;
    std::vector<math::Bounds3> run_bounds;  // per-run bounds (for hover/attach)
};

// measure: no mesh, just metrics (fast, layout passes use this)
TextMetrics measure(const TextBlock& block);

// layout: wrap/break into lines, resolve run bounds
TextMetrics layout(const TextBlock& block, float max_width);

// per-semantic-fragment glyph bounds (annotation anchors):
//   equation.laplacian.bounds.topCenter
math::Bounds3 glyphBounds(const TextBlock& block, std::string_view semantic_id);

} // namespace exd::geometry
```

Responsibilities (promoted from "ui concern" to first-class here):
- glyph metrics, kerning, line breaking, wrapping, alignment, baseline handling
- glyph-atlas generation (FreeType raster) **and** SDF/MSDF atlas (resolution-independent)
- rich spans with semantic ids → enables highlight/hover/animate per fragment
- measurement + `glyphBounds(spanId)` so annotations can point at `α` inside an equation

**Delivery constraint:** bitmap-atlas rendering is sufficient for the first 2D
engine slice. SDF/MSDF is a later quality improvement. The initial milestone is
correct measurement, wrapping, baselines, rich-run bounds, and reuse of the
existing font factory.

Text is **not** a bitmap blob — every run carries a semantic id that the scene
graph (`modules/scene`) and annotation engine resolve to exact bounds.

## 14. Non-Goals

- No GPU calls or buffer creation (renderer)
- No ECS components (core)
- No UI components (the `ui` module)
- No semantic meaning (composer)
- No visual document compilation (the compiler / scene_renderer)
- No physics simulation or collision response
- No file format import/export (OBJ, glTF, STL, etc.) — converters belong elsewhere
- No implicit mesh validation — generators return empty MeshData on degenerate input but do not diagnose why

## 15. Design Decisions & Conventions

**Color representation.** Vertex color is stored as `math::Quat` with the mapping
`{w=R, x=G, y=B, z=A}`. This is an artifact of reusing the Quat type as a generic
RGBA vector. A dedicated `Color` or `Vec4f` type may replace this in the future.

**Descriptor + Generator pattern.** Every parametric shape follows the same pattern:
an immutable geometry descriptor struct (with all-defaulted fields) is passed to a
free function that returns `MeshData` by value. Descriptors are never mutated by
generators. This keeps the API flat, avoids virtual dispatch, and makes chaining
trivial.

**PIMPL for heavy dependencies.** `Path2D` and `FontAtlas` hide their implementation
behind `std::unique_ptr<Impl>`. This prevents FreeType, HarfBuzz, and tessellation
library headers from leaking into the public API, keeping compile times fast and
WASM builds clean.

**SDF blending vs. mesh CSG.** SDF blending (`generate_blend_mesh`) uses marching
cubes on signed distance fields — fast to build, naturally smooth at seams, but
approximate and resolution-limited. Planned mesh boolean operations will provide
exact CSG on triangle meshes for precision use cases.

**Deformation normals.** `deform_mesh()` transforms vertex positions but does not
recompute normals. Callers must chain with a planned normal-recompute operation
or handle normals in the renderer. This is deliberate — it keeps deformation cheap
for renderers that recompute normals in the vertex shader.

## 16. Math Typesetting (AST) — 2D priority

> **2026-08.** For STEM explainers, math is nearly as important as text. Rather
> than treating LaTeX as an opaque string, geometry owns a **math AST** whose
> nodes carry semantic ids, so the engine resolves exact rendered bounds per
> fragment — `α`, `∂u/∂t`, `∇²u` — the same way `TextEngine` does for rich spans.

```cpp
namespace exd::geometry {

// Recursive math AST — each node computes its own bounds
struct MathNode {
    enum class Kind {
        Number, Identifier, Symbol,        // leaves
        Fraction, Superscript, Subscript,  // stacking
        Root, Parentheses, Bracket,        // enclosure
        Sum, Integral,                     // large operators
        Matrix, Vector,                    // layout grids
        Binary, Unary,                     // operators (+, −, ×, ∂, ∇, …)
        Derivative, Limit, Cases,          // composed
    };
    Kind kind;
    std::string value;                     // leaf text ("α", "∂", "∇²u")
    std::optional<std::string> semantic_id; // "laplacian", "timeDerivative", …
    std::vector<MathNode> children;
};

struct MathMetrics {
    math::Vec2f size;
    float baseline;
    std::unordered_map<std::string, math::Bounds3> fragment_bounds; // semantic_id → bounds
};

// parse LaTeX-like source → AST (or accept AST directly)
MathNode parse(const std::string& source);

// recursive bounds resolution: children → parent box
MathMetrics layout(const MathNode& node, const MathStyle& style);

// AST → meshes (glyphs from TextEngine, delimiters/brackets from Path2D)
MeshData render(const MathNode& node, const MathStyle& style);

} // namespace exd::geometry
```

This gives the application or upstream producer a direct mapping
**semantic element → math AST node → exact rendered bounds**, so Composer can
manipulate `laplacian`, `alpha`, or `timeDerivative` directly (highlight,
annotate, animate) without string surgery.

Required constructs (full set): fractions, superscripts, subscripts, roots,
operators, parentheses/brackets, summations, integrals, matrices, vectors,
Greek symbols, derivatives, limits, cases. Delimiter sizing and large-operator
limits are recursive (child bounds drive parent glyph scale).

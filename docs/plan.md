# Extropian Geometry — Project Plan

> This library should live in its own repository: `extropian-geometry`.
>
> It is extracted from Canvas to provide a reusable, renderer-independent mesh generation and geometric computation library.

---

## 1. Purpose

Geometry is a **pure computational library**. It generates `exd::geom::MeshData` from structured descriptors. It has no GPU dependency, no ECS dependency, and no renderer dependency. It depends only on `extropian-core` for math and data types.

Geometry answers:

> Given a geometric description, what is the resulting mesh?

Geometry does NOT answer:

- How to display the mesh (renderer's job)
- How to compose the mesh into a document (canvas's job)
- What the mesh means semantically (composer's job)

---

## 2. Position in the Architecture

```
composer ──→ canvas ──→ renderer ──→ app ──→ core
                 │
                 └──→ geometry ──→ core
```

Geometry is a leaf dependency. Canvas depends on it. Renderer does NOT depend on it (renderer only needs `exd::geom` data types from core, not the generation functions). Other projects can use geometry directly without pulling in canvas or renderer.

---

## 3. Design Principles

- **Pure functions.** Descriptor in, `MeshData` out. No side effects.
- **No allocations hidden in state.** Caller owns the output `MeshData`.
- **No GPU.** No OpenGL headers, no Vulkan, no buffer objects.
- **No ECS.** No registry, no components, no systems.
- **Deterministic.** Same descriptor → same mesh every time.
- **Configurable quality.** Segment counts, tessellation tolerances.
- **Independent of canvas/renderer.** Only core dependency.

---

## 4. Repository Structure

```
extropian-geometry/
├── CMakeLists.txt
├── docs/
│   └── plan.md
├── include/
│   └── exd/geometry/
│       ├── geometry.hpp          # umbrella header
│       ├── mesh_builder.hpp      # MeshBuilder (incremental indexed construction)
│       ├── primitives2d.hpp      # 2D mesh generators
│       ├── primitives3d.hpp      # 3D mesh generators
│       ├── path.hpp              # Path2D, tessellation (fill + stroke)
│       ├── path_types.hpp        # PathCommand, StrokeStyle, FillRule, LineJoin, LineCap
│       ├── text.hpp              # Text shaping abstraction, glyph mesh generation
│       ├── text_types.hpp        # FontId, TextStyle, GlyphPlacement, ShapedText
│       └── ui/                   # higher-level UI primitives
│           ├── button.hpp
│           ├── slider.hpp
│           ├── panel.hpp
│           └── ...
├── src/
│   ├── mesh_builder.cpp
│   ├── primitives2d.cpp
│   ├── primitives3d.cpp
│   ├── path.cpp
│   ├── text.cpp
│   └── ui/
│       └── ...
└── tests/
    ├── unit/
    ├── snapshot/
    └── performance/
```

---

## 5. Modules

### 5.1 mesh_builder

Incremental indexed mesh construction. Used internally by generators and available to users.

```cpp
class MeshBuilder {
public:
    uint32_t add_vertex(const Vertex& v);      // returns auto-assigned index
    void add_triangle(uint32_t a, uint32_t b, uint32_t c);
    void add_quad(uint32_t a, uint32_t b, uint32_t c, uint32_t d);
    void add_line(uint32_t a, uint32_t b);

    MeshData build(PrimitiveTopology topology = Triangles) const;
    void reset();
    size_t vertex_count() const;
    size_t index_count() const;
};
```

### 5.2 primitives3d

Free functions that generate meshes from geometry descriptors.

```cpp
MeshData generate_box_mesh(const BoxGeometry& desc);
MeshData generate_plane_mesh(const PlaneGeometry& desc);
MeshData generate_sphere_mesh(const SphereGeometry& desc);
MeshData generate_icosahedron_mesh(const IcosahedronGeometry& desc);
MeshData generate_ellipsoid_mesh(const EllipsoidGeometry& desc);
MeshData generate_cylinder_mesh(const CylinderGeometry& desc);
MeshData generate_cone_mesh(const ConeGeometry& desc);
MeshData generate_capsule_mesh(const CapsuleGeometry& desc);
MeshData generate_torus_mesh(const TorusGeometry& desc);
MeshData generate_tube_mesh(const TubeGeometry& desc);
MeshData generate_disk_mesh(const DiskGeometry& desc);
MeshData generate_arrow_mesh(const ArrowGeometry& desc);
MeshData generate_axes_mesh(const AxesGeometry& desc);
MeshData generate_billboard_mesh(const BillboardGeometry& desc);
```

Example descriptor:

```cpp
struct SphereGeometry {
    float       radius = 0.5f;
    uint32_t    latitude_segments = 16;
    uint32_t    longitude_segments = 32;
    bool        generate_normals = true;
    bool        generate_texcoords = true;
};
```

### 5.3 primitives2d

```cpp
MeshData generate_rect_mesh(const RectGeometry& desc);
MeshData generate_rounded_rect_mesh(const RoundedRectGeometry& desc);
MeshData generate_circle_mesh(const CircleGeometry& desc);
MeshData generate_ellipse_mesh(const EllipseGeometry& desc);
MeshData generate_arc_mesh(const ArcGeometry& desc);
MeshData generate_ring_mesh(const RingGeometry& desc);
MeshData generate_line_mesh(const LineGeometry& desc);
MeshData generate_polyline_mesh(const PolylineGeometry& desc);
MeshData generate_arrow_mesh(const Arrow2DGeometry& desc);
MeshData generate_grid_mesh(const Grid2DGeometry& desc);
```

These produce 3D meshes on the XY or XZ plane, ready for GPU upload. No separate 2D-only mesh type.

### 5.4 path

SVG-like vector path construction and tessellation.

```cpp
class Path2D {
public:
    Path2D& move_to(Vec2f p);
    Path2D& line_to(Vec2f p);
    Path2D& quadratic_to(Vec2f control, Vec2f end);
    Path2D& cubic_to(Vec2f c0, Vec2f c1, Vec2f end);
    Path2D& arc_to(const ArcDescriptor& arc);
    Path2D& close();

    MeshData tessellate_fill(FillRule rule = FillRule::NonZero,
                             float tolerance = 0.25f) const;
    MeshData tessellate_stroke(const StrokeStyle& style,
                               float tolerance = 0.25f) const;
};
```

Compilation: path commands → curve flattening → fill tessellation → `MeshData`.

Stroke tessellation also handles joins, caps, and dash patterns.

### 5.5 text

Text shaping abstraction and glyph mesh generation.

```cpp
struct ShapedText {
    std::vector<GlyphPlacement> glyphs;
    Vec2f size;
    Bounds2D bounds;
};

class TextShaper {
public:
    virtual ~TextShaper() = default;
    virtual ShapedText shape(std::string_view text,
                              const TextStyle& style,
                              float max_width = 0.0f) const = 0;
};

// Glyph to mesh (for SDF or bitmap atlases)
MeshData generate_glyph_quad_mesh(const GlyphPlacement& glyph,
                                   float atlas_width, float atlas_height);
MeshData generate_text_mesh(const std::vector<GlyphPlacement>& glyphs,
                             float atlas_width, float atlas_height);
```

Backend adapters (HarfBuzz + FreeType) live behind the `TextShaper` interface.

### 5.6 ui (future)

Higher-level composable UI primitives built from 2D geometry:

```
generate_button_mesh(ButtonDescriptor) → MeshData
generate_slider_mesh(SliderDescriptor) → MeshData
generate_panel_mesh(PanelDescriptor)   → MeshData
```

These are pure geometry generators — no layout, no interaction, no styling. They produce `MeshData` that canvas can compose and style.

---

## 6. Dependencies

```
geometry → core  (exd::geom types, exd::math)

geometry does NOT depend on:
  - canvas
  - renderer
  - app
  - composer
  - OpenGL / Vulkan / any GPU API
```

---

## 7. What Moves Here From Canvas

| From Canvas | To Geometry |
|---|---|
| `primitives3d.hpp` + `.cpp` (all generators) | `primitives3d.hpp` + `.cpp` |
| `primitives2d.hpp` + `.cpp` (all generators) | `primitives2d.hpp` + `.cpp` |
| `path.hpp` + `.cpp` (Path2D, tessellation) | `path.hpp` + `.cpp` |
| `text.hpp` + `.cpp` (shaping, glyph mesh) | `text.hpp` + `.cpp` |
| `mesh_builder.hpp` + `.cpp` | `mesh_builder.hpp` + `.cpp` |

| From Renderer | To Geometry |
|---|---|
| `primitive_mesh_system.cpp` (mesh generation logic — cube vertices/indices) | `primitives3d.cpp` (generator functions) |

The renderer's `PrimitiveMeshSystem` retains its ECS wiring (read component → call generator → upload via MeshManager). Only the geometry generation itself moves.

---

## 8. Testing

- **Unit tests**: every generator with known inputs produces expected vertex/index counts, correct topology, valid normals
- **Snapshot tests**: deterministic mesh hashes for common descriptors — change detection for regressions
- **Path tests**: known SVG paths produce expected triangulations
- **Text tests**: shaped text produces correct glyph placements and quad meshes
- **Performance tests**: generation throughput for large meshes

---

## 9. Milestones

| M0 | Repository foundation: CMake, core dependency, test framework, CI |
| M1 | MeshBuilder + 3D primitive generators (sphere, box, cylinder, plane) |
| M2 | Remaining 3D primitives (capsule, torus, tube, cone, ellipsoid, etc.) |
| M3 | 2D primitive generators (rect, circle, arc, ring, line, arrow, grid) |
| M4 | Path2D: commands, Bézier flattening, fill tessellation |
| M5 | Path2D: stroke tessellation (joins, caps, dashes) |
| M6 | Text: shaping interface, glyph mesh generation |
| M7 | UI primitives (button, slider, panel) |

---

## 10. Non-Goals

- Layout algorithms (belongs in canvas)
- Styling / theming (belongs in canvas)
- GPU upload (belongs in canvas via MeshCache + renderer's MeshManager)
- Interaction handling (belongs in canvas)
- Picking / hit testing (belongs in canvas)
- Serialization (belongs in consuming libraries)
- ECS integration (geometry has no ECS dependency)

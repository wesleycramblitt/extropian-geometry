# Extropian Geometry — Project Plan

> Renderer-independent mesh generation and geometric computation library.
>
> Depends only on `extropian-core` for math types (`Vec3f`, `Quat`).

---

## 1. Purpose

Geometry is a **pure computational library**. It generates `exd::geometry::MeshData` from structured descriptors. It has no GPU dependency, no ECS dependency, and no renderer dependency.

Geometry answers:

> Given a geometric description, what is the resulting mesh?

Geometry does NOT answer:

- How to display the mesh (renderer's job)
- How to compose the mesh into a UI component (extropian-ui's job)
- How to lay out UI elements (application's job)
- What the mesh means semantically

---

## 2. Position in the Architecture

```
Higher-Level App (canvas / game engine)
│
├── extropian-ui ──────────┐
│   UI component meshes    │
│   (Button, Panel, etc.)  │
│                          │
└── extropian-geometry ◄───┘  (THIS REPO)
    │  MeshData, Vertex, Bounds types
    │  2D/3D primitive generators
    │  Path2D tessellation
    │  Text shaping + glyph mesh gen
    │  Mesh operations (merge, transform)
    │
    └── extropian-core
        Vec3f, Quat, math utilities
```

Geometry is a **leaf dependency**. `extropian-ui` depends on it. Other projects (renderers, tools, exporters) can use geometry directly without pulling in ui or a windowing system.

**Key change from earlier drafts:** UI components (`button.hpp`, `slider.hpp`, `panel.hpp`) have been moved OUT of geometry into the separate `extropian-ui` repository. Geometry provides only the building blocks — not the composed UI elements.

---

## 3. Design Principles

- **Pure functions.** Descriptor in, `MeshData` out. No side effects.
- **No allocations hidden in state.** Caller owns the output `MeshData`.
- **No GPU.** No OpenGL headers, no Vulkan, no buffer objects.
- **No ECS.** No registry, no components, no systems.
- **Deterministic.** Same descriptor → same mesh every time.
- **Configurable quality.** Segment counts, tessellation tolerances.
- **Depends only on core.**

---

## 4. Repository Structure

```
extropian-geometry/
├── cmakelists.txt
├── docs/
│   └── plan.md
├── include/
│   └── exd/geometry/
│       ├── geometry.hpp          # umbrella header
│       ├── types.hpp             # Vertex, MeshData, Bounds, enums
│       ├── mesh_builder.hpp      # MeshBuilder (incremental indexed construction)
│       ├── mesh_ops.hpp          # merge_meshes, transform_mesh, compute_bounds
│       ├── primitives2d.hpp      # 2D descriptor structs + generator decls
│       ├── primitives3d.hpp      # 3D descriptor structs + generator decls
│       ├── path.hpp              # Path2D, tessellation (fill + stroke)
│       ├── text.hpp              # Text shaping, glyph mesh generation
│       └── font.hpp              # FontAtlas, GlyphCache
├── src/
│   ├── mesh_builder.cpp
│   ├── mesh_ops.cpp
│   ├── primitives2d/
│   │   ├── rounded_rect.cpp
│   │   ├── circle.cpp
│   │   ├── line.cpp
│   │   ├── arc.cpp
│   │   ├── ring.cpp
│   │   ├── polyline.cpp
│   │   ├── arrow.cpp
│   │   └── grid.cpp
│   ├── primitives3d/
│   │   ├── sphere.cpp
│   │   ├── box.cpp
│   │   ├── cylinder.cpp
│   │   ├── plane.cpp
│   │   ├── capsule.cpp
│   │   ├── icosahedron.cpp
│   │   ├── torus.cpp
│   │   ├── cone.cpp
│   │   ├── tube.cpp
│   │   └── disk.cpp
│   ├── path/
│   │   └── path.cpp
│   └── text/
│       ├── text.cpp              # shaping (HarfBuzz)
│       ├── font.cpp              # font loading (FreeType)
│       └── glyph_mesh.cpp        # glyph → mesh generation
└── tests/
    ├── unit/
    ├── snapshot/
    └── performance/
```

---

## 5. Modules

### 5.1 Types (`types.hpp`)

Core data types shared across the ecosystem.

```cpp
struct Vertex {
    math::Vec3f position;
    math::Vec3f normal;
    math::Vec3f uv;
    math::Quat  tangent;
    math::Quat  color;      // RGBA packed in quaternion
};

enum class PrimitiveTopology { Points, Lines, LineStrip, Triangles, TriangleStrip };

struct Bounds { math::Vec3f min; math::Vec3f max; };

struct MeshData {
    std::vector<Vertex>    vertices;
    std::vector<uint32_t>  indices;
    PrimitiveTopology      topology;
    Bounds                 bounds;  // computed automatically or set by caller
};
```

---

### 5.2 MeshBuilder (`mesh_builder.hpp`)

Incremental indexed mesh construction. Used internally by generators and available to callers.

```cpp
class MeshBuilder {
public:
    void reserve(size_t vertexCount, size_t indexCount);

    uint32_t add_vertex(const Vertex& v);
    void add_triangle(uint32_t a, uint32_t b, uint32_t c);
    void add_quad(uint32_t a, uint32_t b, uint32_t c, uint32_t d);
    void add_line(uint32_t a, uint32_t b);

    MeshData build(PrimitiveTopology topology = PrimitiveTopology::Triangles) &&;
    void clear();

    size_t vertex_count() const;
    size_t index_count() const;
};
```

---

### 5.3 MeshOps (`mesh_ops.hpp`) — NEW, HIGH PRIORITY

Utility functions for mesh composition. Needed by `extropian-ui` to combine sub-component meshes.

```cpp
// Combine multiple meshes into one. Appends vertices/indices with offset adjustment.
MeshData merge_meshes(std::span<const MeshData> meshes);

// Apply a 4x4 transform matrix to all vertex positions (and optionally normals).
MeshData transform_mesh(const MeshData& mesh, const math::Mat4f& transform,
                        bool transformNormals = true);

// Compute bounds from vertex positions.
Bounds compute_bounds(std::span<const Vertex> vertices);
```

---

### 5.4 2D Primitives (`primitives2d.hpp`)

Free functions generating 2D meshes on the XY plane (Z=0). These are the building blocks for all UI components.

```cpp
MeshData generate_rounded_rect_mesh(const RoundedRectangleGeometry& desc);
MeshData generate_circle_mesh(const CircleGeometry& desc);
MeshData generate_line_mesh(const LineGeometry& desc);
MeshData generate_arc_mesh(const ArcGeometry& desc);
MeshData generate_ring_mesh(const RingGeometry& desc);
MeshData generate_polyline_mesh(const PolylineGeometry& desc);
MeshData generate_arrow_mesh(const Arrow2DGeometry& desc);
MeshData generate_grid_mesh(const Grid2DGeometry& desc);
```

Example descriptors:

```cpp
struct RoundedRectangleGeometry {
    math::Vec3f size = {1.0f, 1.0f, 0.0f};
    CornerRadii corners;         // per-corner radii
    uint32_t    cornerSegments = 8;
};

struct CircleGeometry {
    float    radius = 0.5f;
    uint32_t segments = 32;
};

struct LineGeometry {
    math::Vec3f start = {0.0f, 0.0f, 0.0f};
    math::Vec3f end   = {1.0f, 0.0f, 0.0f};
    float       width = 0.02f;
};
```

**Implementation status:** Stubs exist. All generators currently return empty `MeshData{}`. This is the highest-priority gap — `extropian-ui` cannot implement anything real until these are done.

---

### 5.5 3D Primitives (`primitives3d.hpp`)

```cpp
MeshData generate_sphere_mesh(const SphereGeometry& desc);
MeshData generate_box_mesh(const BoxGeometry& desc);
MeshData generate_plane_mesh(const PlaneGeometry& desc);
MeshData generate_icosahedron_mesh(const IcosahedronGeometry& desc);
MeshData generate_ellipsoid_mesh(const EllipsoidGeometry& desc);
MeshData generate_cylinder_mesh(const CylinderGeometry& desc);
MeshData generate_cone_mesh(const ConeGeometry& desc);
MeshData generate_capsule_mesh(const CapsuleGeometry& desc);
MeshData generate_torus_mesh(const TorusGeometry& desc);
MeshData generate_tube_mesh(const TubeGeometry& desc);
MeshData generate_disk_mesh(const DiskGeometry& desc);
MeshData generate_arrow_mesh(const Arrow3DGeometry& desc);
MeshData generate_axes_mesh(const AxesGeometry& desc);
MeshData generate_billboard_mesh(const BillboardGeometry& desc);
```

**Implementation status:** Sphere and Box are fully implemented. Cylinder, Plane, Capsule, Icosahedron are stubs. Remaining are not yet started.

---

### 5.6 Path2D (`path.hpp`)

SVG-like vector path construction and tessellation. Enables complex shapes (hearts, stars, custom icons, text outlines).

```cpp
class Path2D {
public:
    Path2D& move_to(math::Vec2f p);
    Path2D& line_to(math::Vec2f p);
    Path2D& quadratic_to(math::Vec2f control, math::Vec2f end);
    Path2D& cubic_to(math::Vec2f c0, math::Vec2f c1, math::Vec2f end);
    Path2D& arc_to(const ArcDescriptor& arc);
    Path2D& close();

    MeshData tessellate_fill(FillRule rule = FillRule::NonZero,
                             float tolerance = 0.25f) const;
    MeshData tessellate_stroke(const StrokeStyle& style,
                               float tolerance = 0.25f) const;

    uint32_t revision() const;  // increments on mutation, for caching
};
```

Pipeline: path commands → curve flattening → triangulation → `MeshData`.

Stroke tessellation handles line joins, caps, and dash patterns.

**Implementation status:** Path command recording and revision tracking are implemented. `tessellateFill()` and `tessellateStroke()` return empty meshes (stubs).

---

### 5.7 Text (`text.hpp`, `font.hpp`)

Text shaping abstraction and glyph mesh generation.

```cpp
// ── Types ──

using FontId = uint64_t;
using GlyphId = uint32_t;

enum class FontWeight { Thin = 100, /* ... */ Black = 900 };
enum class TextAlignment { Left, Center, Right };

struct TextStyle {
    FontId       font;
    float        size = 16.0f;
    FontWeight   weight = FontWeight::Normal;
    TextAlignment alignment = TextAlignment::Left;
};

struct GlyphPlacement {
    GlyphId      glyphId;
    math::Vec2f  position;     // baseline origin
    math::Vec2f  size;         // quad dimensions
    math::Vec2f  uvOffset;     // atlas UV offset
    math::Vec2f  uvSize;       // atlas UV size
    float        advance;      // horizontal advance for next glyph
};

struct ShapedText {
    std::vector<GlyphPlacement> glyphs;
    math::Vec2f  boundsSize;
};

// ── Font Loading ──

class FontAtlas {
public:
    // Load a font file, return a FontId for use in TextStyle
    FontId load_font(const std::string& path, int faceIndex = 0);
    // Access atlas texture data for GPU upload
    std::span<const uint8_t> atlas_data() const;
    math::Vec2f atlas_size() const;
};

// ── Text Shaping ──

class TextShaper {
public:
    virtual ~TextShaper() = default;

    // Shape text with given style, return glyph placements
    virtual ShapedText shape(std::string_view text,
                             const TextStyle& style,
                             float maxWidth = 0.0f) const = 0;
};

// ── Glyph Mesh Generation ──

// Generate a quad mesh for a single glyph (positioned, UV-mapped)
MeshData generate_glyph_mesh(const GlyphPlacement& glyph,
                             const FontAtlas& atlas);

// Generate a mesh for an entire shaped text run
MeshData generate_text_mesh(const ShapedText& shaped,
                            const FontAtlas& atlas);
```

Backend: HarfBuzz for shaping, FreeType for glyph metrics/atlas rasterization.

**Implementation status:** Types and enums are defined. No implementation yet — all functions return empty meshes or are not compiled.

---

## 6. Dependencies

```
extropian-geometry → extropian-core  (math::Vec3f, math::Quat, math::Mat4f)

extropian-geometry does NOT depend on:
  - extropian-ui (ui depends on geometry, not the reverse)
  - Any GPU API (OpenGL, Vulkan, Metal, DirectX)
  - Any windowing system
  - Any ECS framework
  - Any renderer
```

For text support, geometry will gain optional dependencies on:
- **FreeType** — font file parsing, glyph metrics, atlas rasterization
- **HarfBuzz** — text shaping (Unicode, kerning, ligatures, bidirectional text)

These should be gated behind a CMake option (`ENABLE_TEXT=ON` by default) so that headless/embedded builds can exclude them.

---

## 7. What extropian-ui Needs From Geometry

`extropian-ui` is the primary consumer of geometry. Here is the dependency map:

| UI Component | Geometry Functions Needed |
|---|---|
| **Button** | `generate_rounded_rect_mesh()`, `generate_text_mesh()`, `merge_meshes()` |
| **Panel** | `generate_rounded_rect_mesh()` |
| **Slider** | `generate_line_mesh()`, `generate_circle_mesh()`, `merge_meshes()` |
| **Checkbox** | `generate_rounded_rect_mesh()`, `generate_line_mesh()` (checkmark) |
| **TextLabel** | `generate_text_mesh()`, optionally `generate_rounded_rect_mesh()` |
| **Graph2D** | `generate_grid_mesh()`, `generate_polyline_mesh()`, `generate_line_mesh()`, `generate_text_mesh()`, `merge_meshes()`, `transform_mesh()` |
| **Graph3D** | `generate_axes_mesh()`, `generate_grid_mesh()`, 3D surface primitives, `generate_billboard_mesh()` |
| **ProgressBar** | `generate_rounded_rect_mesh()` (background + fill) |
| **Scrollbar** | `generate_line_mesh()`, `generate_rounded_rect_mesh()` |
| **Tooltip** | `generate_rounded_rect_mesh()`, `generate_text_mesh()` |
| **Shape (Heart/Star/etc.)** | `Path2D::tessellateFill()`, `Path2D::tessellateStroke()` |

---

## 8. Testing

- **Unit tests**: every generator with known inputs produces expected vertex/index counts, correct topology, valid normals
- **Snapshot tests**: deterministic mesh hashes for common descriptors — change detection for regressions
- **Path tests**: known SVG paths produce expected triangulations
- **Text tests**: shaped text produces correct glyph placements and quad meshes
- **Performance tests**: generation throughput for large meshes

---

## 9. Milestones

| Milestone | Status | Description |
|---|---|---|
| **M0** | ✅ Done | Repository foundation: CMake, core dependency, test framework |
| **M1** | ✅ Done | MeshBuilder + 3D primitives: Sphere (UV + Icosphere), Box, Cylinder, Plane, Capsule, Icosahedron |
| **M2** | ✅ Done | Remaining 3D primitives: Torus, Cone, Tube, Disk, Arrow, Axes, Billboard, Ellipsoid |
| **M3** | ✅ Done | **MeshOps: `merge_meshes()`, `transform_mesh()`, `compute_bounds()`** |
| **M4** | ✅ Done | **2D primitives: rounded_rect, circle, line** |
| **M5** | ✅ Done | Remaining 2D primitives: ellipse, arc, ring, polyline, arrow, grid, rect |
| **M6** | ✅ Done | Path2D: Bézier flattening, `tessellateFill()` (ear clipping, NonZero + EvenOdd) |
| **M7** | ✅ Done | Path2D: `tessellateStroke()` (Butt/Square/Round caps, Miter/Round/Bevel joins, dash patterns) |
| **M8** | ✅ Done | Text: FontAtlas (FreeType), TextShaper interface + HarfBuzz backend |
| **M9** | ✅ Done | Text: glyph mesh generation, alignment, lineHeight, letterSpacing, maxWidth wrapping |
| **M10** | ✅ Done | SDF Blending: smooth-min union, marching cubes extraction (sphere, box, capsule, cylinder, cone, torus) |
| **M11** | ✅ Done | Comprehensive test coverage: 22 test executables, 170+ test cases |
| **M12** | ⬜ Future | Lathe / Extrusion (revolve 2D profile, sweep along 3D curve) |
| **M13** | ⬜ Future | CSG boolean operations (union, subtract, intersect) |
| **M14** | ⬜ Future | Mesh content hashing for snapshot testing / cache deduplication |
| **M15** | ⬜ Future | Mesh simplification / LOD generation |

---

## 10. Non-Goals

- Layout algorithms (belongs in higher-level application)
- UI component composition (belongs in `extropian-ui`)
- Styling / theming (belongs in renderer)
- GPU upload (belongs in renderer's MeshCache/MeshManager)
- Interaction handling / hit testing (belongs in application)
- Serialization (belongs in consuming libraries)
- ECS integration (geometry has no ECS dependency)
- Physics or collision detection
- Mesh simplification / LOD generation

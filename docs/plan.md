# Extropian Geometry — Mesh Generation Library

> Renderer-independent mesh generation and geometric computation.
> Pure CPU math. Compiles to WASM with zero changes.

## 1. Purpose

Geometry is a **pure computational library**. It generates `exd::geometry::MeshData` from structured descriptors. It has no GPU dependency, no ECS dependency, and no renderer dependency.

Geometry answers:

> Given a geometric description, what is the resulting mesh?

Geometry does NOT answer:

- How to display the mesh (renderer's job)
- How to compose the mesh into a UI component (extropian-ui's job, desktop only)
- How to lay out UI elements (canvas / canvas-web's job)
- What the mesh means semantically (composer's job)

## 2. Position in the Architecture

```
Higher-Level App (canvas / canvas-web)
│
├── extropian-ui ──────────┐  (desktop only: button, panel, slider meshes)
│   UI component meshes    │
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

Geometry is a **leaf dependency**. `extropian-ui` depends on it. The WebGL canvas inset in canvas-web uses it for mesh generation. It compiles to WASM with zero changes — pure math, no platform dependencies.

## 3. Design Principles

- **Pure functions.** Descriptor in, `MeshData` out. No side effects.
- **No allocations hidden in state.** Caller owns the output `MeshData`.
- **No GPU.** No OpenGL headers, no Vulkan, no buffer objects.
- **No ECS.** No registry, no components, no systems.
- **Deterministic.** Same descriptor → same mesh every time.
- **Configurable quality.** Segment counts, tessellation tolerances.
- **Depends only on core.** `Vec3f`, `Quat`, and basic math utilities.

## 4. Primitive Types

### 2D Primitives

| Primitive | Geometry Descriptor | Generator |
|---|---|---|
| Rectangle | `RectangleGeometry` | `generate_rectangle_mesh()` |
| Rounded Rectangle | `RoundedRectangleGeometry` | `generate_rounded_rectangle_mesh()` |
| Circle | `CircleGeometry` | `generate_circle_mesh()` |
| Ellipse | `EllipseGeometry` | `generate_ellipse_mesh()` |
| Arc | `ArcGeometry` | `generate_arc_mesh()` |
| Ring | `RingGeometry` | `generate_ring_mesh()` |
| Line | `LineGeometry` | `generate_line_mesh()` |
| Polyline | `PolylineGeometry` | `generate_polyline_mesh()` |
| Arrow | `ArrowGeometry` | `generate_arrow_mesh()` |
| Grid | `GridGeometry` | `generate_grid_mesh()` |
| Path2D | `Path2D` | `generate_path2d_mesh()` |

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
| Billboard | `BillboardGeometry` | `generate_billboard_mesh()` |

### Extended Geometry

| Type | Descriptor | Generator |
|---|---|---|
| Extrusion | `ExtrusionGeometry` | `generate_extrusion_mesh()` |
| Lathe | `LatheGeometry` | `generate_lathe_mesh()` |
| Helix | `HelixGeometry` | `generate_helix_mesh()` |
| Heightmap | `Heightmap` | `generate_heightmap_mesh()` |
| Blend SDF | `BlendGeometry` | `generate_blend_mesh()` |
| Text | `TextVisualDescriptor` | `generate_text_mesh()` |

## 5. Types

```cpp
struct Vertex {
    Vec3f position;
    Vec3f normal;
    Vec3f uv;
    Vec4f tangent;
    Vec4f color;
};

struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    PrimitiveTopology topology;  // Triangles, Lines, Points
    Bounds bounds;
};
```

## 6. Platform & Compilation

- Compiles on any C++23 compiler
- **Compiles to WASM via Emscripten with zero changes** — pure CPU math, no GPU, no ECS
- No platform-specific headers or dependencies
- Depends only on `extropian-core` for `Vec3`, `Quat`, and math utilities

## 7. File Layout

```
include/exd/geometry/
├── types.hpp                # Vertex, MeshData, Bounds types
├── primitives2d.hpp         # 2D geometry descriptors + generators
├── primitives3d.hpp         # 3D geometry descriptors + generators
├── path.hpp                 # Path2D descriptor + tessellation
├── text.hpp                 # TextVisualDescriptor + shaping
├── blend.hpp                # BlendGeometry (SDF blending)
├── extrusion.hpp            # ExtrusionGeometry, LatheGeometry, HelixGeometry
├── heightmap.hpp            # Heightmap
└── mesh_ops.hpp             # Merge, transform, subdivide, etc.

src/
├── primitives2d.cpp, primitives3d.cpp
├── path.cpp, text.cpp, blend.cpp
├── extrusion.cpp, heightmap.cpp
└── mesh_ops.cpp
```

## 8. Non-Goals

- No GPU calls or buffer creation (renderer)
- No ECS components (core)
- No UI components (extropian-ui, desktop only)
- No semantic meaning (composer)
- No visual document compilation (canvas)

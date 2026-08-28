#pragma once

#include <exd/geometry/gizmos.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <cstdint>

namespace exd::geometry::detail {

// ── Internal gizmo mesh helpers ──
//
// Private (not installed). Every returned MeshData has topology = Triangles and
// computed bounds. uv convention: (u along length/arc normalized 0..1,
// v around circumference, 0); tangent {1,0,0,1}.

struct AxisFrame { math::Vec3f dir, right, up; };   // orthonormal; up = dir.cross(right)

/// Orthonormal frame for `dir`. Ref {0,1,0}; if |dir·ref| > 0.999 use {0,0,1};
/// right = ref.cross(dir).normalized(); up = dir.cross(right).
AxisFrame make_frame(const math::Vec3f& dir);

/// Capped cylinder from `from` to `to` (from != to, radius > 0): 2 rings (slices+1 each)
/// + 2 cap centers. Side normals radial; cap normals ±dir. Indices: side quads + 2 cap fans.
MeshData build_capped_cylinder(const math::Vec3f& from, const math::Vec3f& to,
                               float radius, uint32_t slices, const math::Quat& color);

/// Cone: base ring (slices+1) at baseCenter + tip vertex (+ optional base cap center
/// with normal -dir). Side normals per axes.cpp formula.
MeshData build_cone(const math::Vec3f& baseCenter, const math::Vec3f& tip,
                    float baseRadius, uint32_t slices, const math::Quat& color,
                    bool baseCap = true);

/// Cone with fillet bead: full torus ring (axis = dir) at baseCenter, major radius
/// = baseRadius - filletR, minor = filletR, filletR = clamp(0.25*baseRadius, ...);
/// merged into the cone mesh (append vertices/indices, offset indices).
MeshData build_cone_with_fillet(const math::Vec3f& baseCenter, const math::Vec3f& dir,
                                const math::Vec3f& tip, float baseRadius,
                                uint32_t slices, const math::Quat& color);

/// Torus arc: ring circle lies in the plane with normal `axis`, center at origin.
/// angle 0 direction = `zeroDir` (must be perpendicular to axis; caller passes a
/// unit vector). Start angle `start`, sweep `sweep` radians (2pi = full torus).
/// Vertex: C(θ) = R*(zeroDir*cosθ + (axis×zeroDir)*sinθ); tube point:
/// C(θ) + minorR*(N(θ)*cosφ + axis*sinφ) with N(θ) = C(θ)/R (tube cross-section
/// circle in the plane spanned by ring normal and axis). majorSegs stations, minorSegs
/// tube segments. uv.x = θ/sweep, uv.y = φ/2π.
MeshData build_torus_arc(const math::Vec3f& axis, const math::Vec3f& zeroDir,
                         float majorR, float minorR, float start, float sweep,
                         uint32_t majorSegs, uint32_t minorSegs, const math::Quat& color);

/// Axis-aligned box 24 verts / 36 idx, half-extents per axis (faces: pos/neg each axis).
MeshData build_box(const math::Vec3f& center, const math::Vec3f& halfSize, const math::Quat& color);

/// Wrapper over public generate_icosahedron_mesh(radius, subdivisions, color).
/// Returns an origin-centered sphere of the given radius.
MeshData build_icosphere(float radius, const math::Quat& color, uint32_t subdivisions = 2);

/// Merge two MeshData (append + index offset). Used to fold fillets into parts.
MeshData concat_meshes(const MeshData& a, const MeshData& b);

/// Convenience: build a GizmoPart with bounds/topology already set.
GizmoPart make_part(GizmoAxis axis, GizmoPartKind kind, uint32_t partId, MeshData&& mesh);

} // namespace exd::geometry::detail
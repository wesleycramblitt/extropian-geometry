#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/mat3.hpp>
#include <exd/math/mat4.hpp>

#include <span>

namespace exd::geometry
{

/// Merge multiple meshes into a single mesh.
/// Vertices are concatenated; indices are offset by the cumulative vertex count
/// of preceding meshes. Topology must be the same across all inputs.
MeshData merge_meshes(std::span<const MeshData> meshes);

/// Apply a 4x4 transform to all vertex positions (and optionally normals).
/// Returns a new MeshData; the input is unchanged.
MeshData transform_mesh(const MeshData& mesh,
                        const math::Mat4& transform,
                        bool transformNormals = true);

/// Compute axis-aligned bounding box from vertex positions.
Bounds compute_bounds(std::span<const Vertex> vertices);

// ── Added: triangulation / welding / normals ──

/// Triangulate a simple polygon (ear clipping) with optional holes, all
/// planar in arbitrary 3D orientation. `outline` and each `hole` are vertex
/// rings in consistent order (any winding: input is normalized internally,
/// outline vs hole winding must OPPOSE each other as given — we normalize by
/// signed area so this is handled). Returns triangle indices referring to the
/// combined vertex array [outline..., hole0..., hole1...]. < 3 outline
/// vertices, or holes degenerate → empty result.
std::vector<uint32_t> triangulate_polygon(
    const std::vector<math::Vec3f>& outline,
    const std::vector<std::vector<math::Vec3f>>& holes = {});

/// Weld vertices closer than `epsilon` (hash grid, first-wins: the earliest
/// vertex keeps position/normal/uv/color; later duplicates remap to it).
/// Indices are remapped; triangle ORDINALS (order/count) are unchanged, so
/// Part patches remain valid. epsilon <= 0 → returns a copy unchanged.
MeshData weld_vertices(const MeshData& mesh, float epsilon);

enum class NormalMode
{
    Flat,   // one normal per face, duplicated vertices per face
    Smooth  // angle-weighted per-vertex average over incident faces
};

// ── Boolean (CSG) ──

enum class BooleanOp
{
    Union,     // a ∪ b
    Subtract,  // a − b
    Intersect  // a ∩ b
};

/// Boolean operation on two closed, consistently oriented triangle meshes.
/// Returns an empty MeshData when: either input is empty; either input fails
/// the closed-manifold gate (boundary edges, non-manifold edges, or
/// inconsistent directed edge orientation — checked after internal
/// position-canonicalization, so un-welded generator output is fine); either
/// input is a zero-volume shell; the two inputs have COPLANAR face overlap
/// (documented V1 limitation — returns {} rather than emitting holes);
/// or the assembled result fails the same watertight gate (float failures
/// return {} instead of garbage).
/// Patch contract: MeshData-level op. Part patches refer to input triangle
/// ordinals and do NOT survive; callers re-tag results via tag_faces.
/// Complexity: O(Ta·Tb) splitting + O(sub-polygons·T) classification (naive;
/// AABB broad-phase and BVH parity queries are V2). Input self-intersections
/// are out of contract. All internal tolerances are scale-relative to the
/// bounding diagonal `diag` (1e-6·diag plane/on-surface, 1e-7·diag position
/// canonicalization).
/// weldEpsilon: 0 → auto (1e-4 × the max input diagonal — the auto-weld
/// distance is approximated by the input bounds since the result bounds are
/// not known until after assembly); otherwise used directly as the
/// post-assembly weld distance AND as the classification nudge via
/// eps_class = weldEpsilon × 0.5.
MeshData boolean_mesh(const MeshData& a, const MeshData& b, BooleanOp op,
                      float weldEpsilon = 0.0f);

/// Recompute normals. Flat: vertices split per face. Smooth: vertices kept,
/// normals averaged (angle-weighted by corner angle). Assumes Triangles
/// topology. Empty input → empty output.
MeshData recompute_normals(const MeshData& mesh, NormalMode mode = NormalMode::Smooth);

// ── Mass / inertial properties (simulator inputs) ──

/// Inertial properties of a closed solid, computed from the tessellation.
/// Meaningful for watertight, consistently-oriented meshes (the boolean
/// closed-manifold gate and the recipes' signed-volume checks qualify them);
/// the computation itself does not gate and works on any consistent mesh.
/// All values SI: volume m³, area m², mass kg, centroid m, inertia kg·m².
struct MassProperties
{
    float volume       = 0.0f;   // signed-volume magnitude (m³)
    float surface_area = 0.0f;   // (m²)
    float mass         = 0.0f;   // density × volume (kg)
    math::Vec3f centroid;        // geometric centroid (m; origin for empty)
    math::Mat3  inertia;         // inertia tensor about the centroid (kg·m²)
};

/// Compute MassProperties from a closed triangle mesh via Mirtich tetrahedron
/// decomposition (double accumulation). `density` defaults to water (1000
/// kg/m³). Empty or degenerate input → zeroed properties (mass = 0.
MassProperties mesh_properties(const MeshData& mesh, float density = 1000.0f);

} // namespace exd::geometry

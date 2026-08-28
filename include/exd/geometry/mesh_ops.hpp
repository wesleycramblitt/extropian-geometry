#pragma once

#include <exd/geometry/types.hpp>
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

/// Recompute normals. Flat: vertices split per face. Smooth: vertices kept,
/// normals averaged (angle-weighted by corner angle). Assumes Triangles
/// topology. Empty input → empty output.
MeshData recompute_normals(const MeshData& mesh, NormalMode mode = NormalMode::Smooth);

} // namespace exd::geometry

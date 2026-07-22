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

} // namespace exd::geometry

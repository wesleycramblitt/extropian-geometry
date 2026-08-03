#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

#include <vector>

namespace exd::geometry
{

/// Heightmap → 3D terrain mesh.
/// The heightmap is a 2D grid of height values (row-major: heightData[y * width + x]).
struct Heightmap
{
    std::vector<float> heightData;
    uint32_t width = 0;
    uint32_t height = 0;
    math::Vec3f size = {1.0f, 1.0f, 1.0f};   // world-space extent (X, Y=height scale, Z)
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};

MeshData generate_heightmap_mesh(const Heightmap& hm);

} // namespace exd::geometry

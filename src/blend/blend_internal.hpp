#pragma once

#include <exd/geometry/types.hpp>

#include <vector>

namespace exd::geometry
{

/// Internal: 3D grid of pre-computed SDF values for marching cubes.
struct SdfGrid
{
    int nx = 0, ny = 0, nz = 0;
    math::Vec3f origin{0, 0, 0};
    float cellSize = 0.05f;
    std::vector<float> values;

    float at(int ix, int iy, int iz) const
    {
        if (ix < 0 || ix >= nx || iy < 0 || iy >= ny || iz < 0 || iz >= nz)
            return 1e10f;
        return values[static_cast<size_t>(iz) * (nx * ny)
                    + static_cast<size_t>(iy) * nx
                    + static_cast<size_t>(ix)];
    }
};

/// Extract isosurface from a pre-computed SDF grid via marching cubes.
MeshData extract_isosurface(const SdfGrid& grid, bool generateNormals);

/// Evaluate the signed distance from a world-space point to a single primitive.
float evaluate_primitive_sdf(const math::Vec3f& worldPoint,
                             const struct BlendPrimitive& prim);

} // namespace exd::geometry

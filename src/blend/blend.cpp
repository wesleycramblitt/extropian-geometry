#include <exd/geometry/blend.hpp>
#include <exd/geometry/mesh_ops.hpp>
#include "blend_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace exd::geometry
{

namespace
{

// ============================================================================
// Smooth-min blending
// ============================================================================

/// Polynomial smooth minimum.
/// k controls the blend radius — larger k = softer blend.
/// k=0 recovers the exact minimum (sharp CSG union).
static float smin(float a, float b, float k)
{
    if (k <= 0.0f) return std::min(a, b);
    float h = std::clamp(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);
    return a * h + b * (1.0f - h) - k * h * (1.0f - h);
}

/// Evaluate the combined signed distance field at a world-space point.
/// Handles Union (smooth-min), Subtract (max(a, -b)), and Intersect (max of all).
static float evaluate_combined_sdf(const math::Vec3f& p,
                                   const std::vector<BlendPrimitive>& prims,
                                   float blendK,
                                   BlendOp op)
{
    if (prims.empty()) return 1e10f;

    switch (op)
    {
    case BlendOp::Union: {
        float d = evaluate_primitive_sdf(p, prims[0]);
        for (size_t i = 1; i < prims.size(); ++i)
            d = smin(d, evaluate_primitive_sdf(p, prims[i]), blendK);
        return d;
    }
    case BlendOp::Subtract: {
        // First primitive minus all others
        float d = evaluate_primitive_sdf(p, prims[0]);
        for (size_t i = 1; i < prims.size(); ++i)
            d = std::max(d, -evaluate_primitive_sdf(p, prims[i]));
        return d;
    }
    case BlendOp::Intersect: {
        float d = evaluate_primitive_sdf(p, prims[0]);
        for (size_t i = 1; i < prims.size(); ++i)
            d = std::max(d, evaluate_primitive_sdf(p, prims[i]));
        return d;
    }
    }
    return 1e10f;
}

// ============================================================================
// Bounds computation
// ============================================================================

/// Compute the AABB that encloses all primitives, expanded by blend radius.
static void compute_grid_bounds(const std::vector<BlendPrimitive>& prims,
                                float blendK,
                                float cellSize,
                                math::Vec3f& outMin,
                                math::Vec3f& outMax,
                                int& outNx, int& outNy, int& outNz)
{
    if (prims.empty())
    {
        outMin = outMax = {0, 0, 0};
        outNx = outNy = outNz = 0;
        return;
    }

    // Conservative AABB: sample each primitive's bounding sphere/box + padding
    float inf = std::numeric_limits<float>::max();
    float minX = inf, minY = inf, minZ = inf;
    float maxX = -inf, maxY = -inf, maxZ = -inf;

    for (const auto& prim : prims)
    {
        // Compute rough bounding radius for the primitive in local space
        float halfExt = 1.0f;
        switch (prim.kind)
        {
        case BlendPrimitiveKind::Sphere:
            halfExt = prim.radius;
            break;
        case BlendPrimitiveKind::Box:
            halfExt = prim.halfExtent;
            break;
        case BlendPrimitiveKind::Capsule:
            halfExt = prim.radius + prim.height * 0.5f;
            break;
        case BlendPrimitiveKind::Cylinder:
            halfExt = std::max(prim.radius, prim.height * 0.5f);
            break;
        case BlendPrimitiveKind::Cone:
            halfExt = std::max(prim.radius, prim.height * 0.5f);
            break;
        case BlendPrimitiveKind::Torus:
            halfExt = prim.radius2 + prim.radius;
            break;
        }

        // Account for scale — use max scale component as conservative bound
        float s = std::max({std::abs(prim.scale.x), std::abs(prim.scale.y), std::abs(prim.scale.z)});
        halfExt *= s;

        minX = std::min(minX, prim.position.x - halfExt);
        minY = std::min(minY, prim.position.y - halfExt);
        minZ = std::min(minZ, prim.position.z - halfExt);
        maxX = std::max(maxX, prim.position.x + halfExt);
        maxY = std::max(maxY, prim.position.y + halfExt);
        maxZ = std::max(maxZ, prim.position.z + halfExt);
    }

    // Expand by blend radius
    float pad = std::max(blendK, cellSize * 2.0f);
    minX -= pad; minY -= pad; minZ -= pad;
    maxX += pad; maxY += pad; maxZ += pad;

    outMin = {minX, minY, minZ};
    outMax = {maxX, maxY, maxZ};

    outNx = static_cast<int>(std::ceil((maxX - minX) / cellSize)) + 1;
    outNy = static_cast<int>(std::ceil((maxY - minY) / cellSize)) + 1;
    outNz = static_cast<int>(std::ceil((maxZ - minZ) / cellSize)) + 1;
}

/// Fill the grid by evaluating the combined SDF at each grid point.
static void fill_grid(SdfGrid& grid,
                      const std::vector<BlendPrimitive>& prims,
                      float blendK,
                      BlendOp op)
{
    for (int iz = 0; iz < grid.nz; ++iz)
    {
        for (int iy = 0; iy < grid.ny; ++iy)
        {
            for (int ix = 0; ix < grid.nx; ++ix)
            {
                math::Vec3f p = {
                    grid.origin.x + ix * grid.cellSize,
                    grid.origin.y + iy * grid.cellSize,
                    grid.origin.z + iz * grid.cellSize
                };
                grid.values[iz * (grid.nx * grid.ny) + iy * grid.nx + ix] =
                    evaluate_combined_sdf(p, prims, blendK, op);
            }
        }
    }
}

} // anonymous namespace

// ============================================================================
// Public entry point
// ============================================================================

MeshData generate_blend_mesh(const BlendGeometry& geometry)
{
    const auto& prims = geometry.primitives;
    if (prims.empty())
        return {};

    // Validate: at least one primitive with positive volume
    bool hasVolume = false;
    for (const auto& p : prims)
    {
        switch (p.kind)
        {
        case BlendPrimitiveKind::Sphere:
        case BlendPrimitiveKind::Capsule:
        case BlendPrimitiveKind::Cylinder:
        case BlendPrimitiveKind::Cone:
            if (p.radius > 0.0f) hasVolume = true;
            break;
        case BlendPrimitiveKind::Box:
            if (p.halfExtent > 0.0f) hasVolume = true;
            break;
        case BlendPrimitiveKind::Torus:
            if (p.radius > 0.0f && p.radius2 > 0.0f) hasVolume = true;
            break;
        }
        if (hasVolume) break;
    }
    if (!hasVolume)
        return {};

    // Compute grid bounds
    math::Vec3f gridMin, gridMax;
    int nx, ny, nz;
    float cellSize = std::max(geometry.cellSize, 0.001f);
    compute_grid_bounds(prims, geometry.blendRadius, cellSize,
                        gridMin, gridMax, nx, ny, nz);

    // Clamp grid to reasonable size (prevent OOM on bad inputs)
    constexpr int kMaxGridDim = 256;
    if (nx > kMaxGridDim || ny > kMaxGridDim || nz > kMaxGridDim)
        return {}; // input would produce enormous mesh

    if (nx < 2 || ny < 2 || nz < 2)
        return {};

    // Allocate and fill grid
    SdfGrid grid;
    grid.nx = nx;
    grid.ny = ny;
    grid.nz = nz;
    grid.origin = gridMin;
    grid.cellSize = cellSize;
    grid.values.resize(static_cast<size_t>(nx) * ny * nz);

    fill_grid(grid, prims, geometry.blendRadius, geometry.op);

    // Extract isosurface
    auto mesh = extract_isosurface(grid, geometry.generateNormals);
    mesh.bounds = compute_bounds(mesh.vertices);
    return mesh;
}

} // namespace exd::geometry

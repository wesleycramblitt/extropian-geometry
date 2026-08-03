#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>
#include <exd/math/quat.hpp>

#include <cstdint>
#include <vector>

namespace exd::geometry
{

// ── SDF Primitive Kinds ──

enum class BlendPrimitiveKind
{
    Sphere,
    Box,
    Capsule,
    Cylinder,
    Cone,
    Torus
};

// ── Blend Primitive ──

/// A single SDF primitive contributing to a blended shape.
///
/// Each primitive defines a signed distance field. Multiple primitives
/// are combined via smooth-min blending and the resulting isosurface
/// is extracted via marching cubes.
///
/// The `id` field is preserved through the pipeline so that canvas-level
/// code can map sub-meshes back to their source primitives for picking
/// and incremental updates.  Set to 0 if identity tracking is not needed.
struct BlendPrimitive
{
    BlendPrimitiveKind kind = BlendPrimitiveKind::Sphere;

    // Transform — applied to the implicit surface
    math::Vec3f position = {0.0f, 0.0f, 0.0f};
    math::Quat  rotation = {1.0f, 0.0f, 0.0f, 0.0f}; // identity
    math::Vec3f scale    = {1.0f, 1.0f, 1.0f};

    // Kind-specific parameters (meaning depends on `kind`)
    float radius      = 0.5f;   // Sphere, Capsule, Cylinder, Cone, Torus(minor)
    float radius2     = 1.0f;   // Torus major radius
    float height      = 1.0f;   // Capsule, Cylinder, Cone (along local +Y)
    float halfExtent  = 0.5f;   // Box half-size (all axes equal; use scale for non-uniform)

    // Stable identifier for canvas-level mapping (0 = anonymous)
    uint32_t id = 0;
};

// ── Blend operation ──

enum class BlendOp
{
    Union,       // smooth-min of all primitive SDFs (default)
    Subtract,    // first primitive minus all others (max(a, -b))
    Intersect,   // intersection of all primitives (max of all SDFs)
};

// ── Blend Geometry Descriptor ──

struct BlendGeometry
{
    std::vector<BlendPrimitive> primitives;

    BlendOp op          = BlendOp::Union;
    float blendRadius   = 0.15f;  // smooth-min blend radius (ignored for Subtract/Intersect)
    float cellSize      = 0.05f;  // marching cubes voxel edge length
    bool  generateNormals = true;  // compute normals from SDF gradient
};

// ── Generator ──

/// Generate a seamless blended mesh from multiple SDF primitives.
///
/// Primitives are combined via smooth-min blending and the resulting
/// isosurface is extracted via marching cubes.  The output is a single
/// triangle mesh with per-vertex normals (computed from the SDF gradient
/// when `generateNormals` is true).
///
/// Returns an empty MeshData if `primitives` is empty or if any primitive
/// has zero effective volume (e.g. zero radius, zero height).
MeshData generate_blend_mesh(const BlendGeometry& geometry);

} // namespace exd::geometry

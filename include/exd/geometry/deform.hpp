#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

namespace exd::geometry
{

// ── Deformation operators ──

/// Descriptor for mesh deformation operations.
/// All deformations are applied in local space (centered at origin, then
/// optionally recentered).  Multiple deformations can be chained by calling
/// deform_mesh() repeatedly.
struct DeformDescriptor
{
    // Bend
    bool   bend    = false;
    float  bendAngle = 0.0f;       // radians, total bend angle
    float  bendRadius = 1.0f;      // radius of the bend arc
    math::Vec3f bendAxis = {0, 1, 0}; // axis to bend around

    // Twist
    bool   twist   = false;
    float  twistAngle = 0.0f;      // radians, total twist
    math::Vec3f twistAxis = {0, 1, 0};

    // Taper
    bool   taper   = false;
    float  taperStart = 1.0f;      // scale at bottom of tapered axis
    float  taperEnd   = 0.5f;      // scale at top
    math::Vec3f taperAxis = {0, 1, 0};

    // Noise
    bool   noise   = false;
    float  noiseAmplitude = 0.1f;
    float  noiseFrequency = 2.0f;
    uint32_t noiseSeed  = 0;
};

/// Apply deformation to a mesh.  Returns a new MeshData; input unchanged.
/// Normals are NOT recomputed by default — call compute_bounds for bounds only.
MeshData deform_mesh(const MeshData& mesh, const DeformDescriptor& desc);

} // namespace exd::geometry

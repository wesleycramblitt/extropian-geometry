#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

#include <vector>

namespace exd::geometry
{

// ── Extrusion ──

/// Extrude a closed 2D profile (outline vertices in XY plane, CCW order)
/// along the Z axis to produce a 3D mesh.
struct ExtrusionGeometry
{
    std::vector<math::Vec3f> profile;  // 2D outline vertices (Z=0), closed loop
    float depth = 1.0f;                // extrusion distance along +Z
    bool capped = true;                // fill front and back faces
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

MeshData generate_extrusion_mesh(const ExtrusionGeometry& geometry);

// ── Lathe / Revolve ──

enum class LatheAxis { X, Y, Z };

/// Revolve a 2D profile curve (points in XY plane) around an axis to produce a 3D mesh.
/// Profile points should be ordered along the curve from bottom to top (for Y axis revolves).
struct LatheGeometry
{
    std::vector<math::Vec3f> profile;   // 2D curve points in XY plane
    LatheAxis axis = LatheAxis::Y;
    uint32_t segments = 64;             // rotational subdivisions
    float startAngle = 0.0f;            // radians, 0 = full revolve
    float endAngle   = 2.0f * 3.14159265358979323846f;
    bool capped = true;                 // fill end caps for partial revolves
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

MeshData generate_lathe_mesh(const LatheGeometry& geometry);

// ── Helix / Spring ──

/// Sweep a closed 2D profile (outline vertices in XY plane) along a helical path.
struct HelixGeometry
{
    std::vector<math::Vec3f> profile;   // 2D outline vertices (Z=0), closed loop
    float radius = 1.0f;                // helix radius (distance from axis)
    float height = 3.0f;                // total height along the axis
    float turns = 5.0f;                 // number of full rotations
    uint32_t pathSteps = 128;           // segments along the helix
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

MeshData generate_helix_mesh(const HelixGeometry& geometry);

} // namespace exd::geometry

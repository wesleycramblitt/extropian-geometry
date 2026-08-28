#pragma once

#include <exd/geometry/types.hpp>
#include <exd/geometry/part.hpp>
#include <exd/math/vec3.hpp>

#include <vector>

namespace exd::geometry
{

// ── 3D primitive geometry descriptors ──

enum class SphereConstruction
{
    Uv,
    Icosphere
};

struct SphereGeometry
{
    float radius = 0.5f;
    uint32_t latitudeSegments = 32;
    uint32_t longitudeSegments = 64;
    SphereConstruction construction = SphereConstruction::Uv;
    bool generateNormals = true;
    bool generateTexcoords = true;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct BoxGeometry
{
    math::Vec3f size = {1.0f, 1.0f, 1.0f};
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct PlaneGeometry
{
    math::Vec3f size = {1.0f, 0.0f, 1.0f};
    uint32_t segmentsW = 1;
    uint32_t segmentsD = 1;
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct EllipsoidGeometry
{
    math::Vec3f radii = {0.5f, 1.0f, 0.5f};
    uint32_t latitudeSegments = 32;
    uint32_t longitudeSegments = 64;
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct CylinderGeometry
{
    float radius = 0.5f;
    float height = 1.0f;
    uint32_t slices = 64;
    bool capped = true;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct ConeGeometry
{
    float radius = 0.5f;
    float height = 1.0f;
    uint32_t slices = 64;
    bool capped = true;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct CapsuleGeometry
{
    float radius = 0.25f;
    float height = 1.0f;
    uint32_t slices = 64;
    uint32_t stacks = 16;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct TorusGeometry
{
    float majorRadius = 1.0f;
    float minorRadius = 0.3f;
    uint32_t majorSegments = 64;
    uint32_t minorSegments = 32;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct TubeGeometry
{
    std::vector<math::Vec3f> path;
    float radius = 0.1f;
    uint32_t radialSegments = 32;
    bool capped = true;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct DiskGeometry
{
    float outerRadius = 1.0f;
    float innerRadius = 0.0f;
    uint32_t segments = 64;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct Arrow3DGeometry
{
    math::Vec3f start = {0.0f, 0.0f, 0.0f};
    math::Vec3f end   = {0.0f, 1.0f, 0.0f};
    float headRadius  = 0.15f;
    float headLength  = 0.3f;
    float shaftRadius = 0.05f;
    uint32_t slices = 32;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct AxesGeometry
{
    float length = 1.0f;
    float shaftRadius = 0.02f;
    float headRadius  = 0.06f;
    float headLength  = 0.15f;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f}; // overridden per-axis
};

struct BillboardGeometry
{
    math::Vec3f size = {1.0f, 1.0f, 0.0f};
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};

// ── 3D mesh generators ──

MeshData generate_sphere_mesh(const SphereGeometry& geometry);
MeshData generate_box_mesh(const BoxGeometry& geometry);
MeshData generate_cylinder_mesh(const CylinderGeometry& geometry);
MeshData generate_plane_mesh(const PlaneGeometry& geometry);
MeshData generate_capsule_mesh(const CapsuleGeometry& geometry);
MeshData generate_icosahedron_mesh(float radius, int subdivisions, math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f});
MeshData generate_ellipsoid_mesh(const EllipsoidGeometry& geometry);
MeshData generate_torus_mesh(const TorusGeometry& geometry);
MeshData generate_cone_mesh(const ConeGeometry& geometry);
MeshData generate_tube_mesh(const TubeGeometry& geometry);
MeshData generate_disk_mesh(const DiskGeometry& geometry);
MeshData generate_arrow3d_mesh(const Arrow3DGeometry& geometry);
MeshData generate_axes_mesh(const AxesGeometry& geometry);
MeshData generate_billboard_mesh(const BillboardGeometry& geometry);

// ── Part generators (native BC patches) ──
Part generate_cylinder_part(const CylinderGeometry& geometry); // wall, cap_start (-Y), cap_end (+Y)
Part generate_cone_part(const ConeGeometry& geometry);         // wall, cap_start (base)
Part generate_box_part(const BoxGeometry& geometry);           // +x -x +y -y +z -z

} // namespace exd::geometry

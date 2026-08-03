#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

#include <numbers>
#include <vector>

namespace exd::geometry
{

// ── 2D primitive geometry descriptors ──

struct CornerRadii
{
    float topLeft     = 0.0f;
    float topRight    = 0.0f;
    float bottomRight = 0.0f;
    float bottomLeft  = 0.0f;
};

struct RectangleGeometry
{
    math::Vec3f size = {1.0f, 1.0f, 0.0f};
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f}; // RGBA: w=R, x=G, y=B, z=A
};

struct RoundedRectangleGeometry
{
    math::Vec3f size = {1.0f, 1.0f, 0.0f};
    CornerRadii radii;
    uint32_t cornerSegments = 16;
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct CircleGeometry
{
    float radius = 1.0f;
    uint32_t segments = 32;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct EllipseGeometry
{
    float radiusX = 1.0f;
    float radiusY = 0.5f;
    uint32_t segments = 32;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct ArcGeometry
{
    float radius = 1.0f;
    float startAngle = 0.0f;
    float endAngle = std::numbers::pi_v<float> * 1.5f;
    uint32_t segments = 32;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct RingGeometry
{
    float outerRadius = 1.0f;
    float innerRadius = 0.5f;
    uint32_t segments = 32;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct LineGeometry
{
    math::Vec3f start = {0.0f, 0.0f, 0.0f};
    math::Vec3f end   = {1.0f, 0.0f, 0.0f};
    float width = 1.0f;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct PolylineGeometry
{
    std::vector<math::Vec3f> points;
    float width = 1.0f;
    bool closed = false;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct ArrowGeometry
{
    math::Vec3f start = {0.0f, 0.0f, 0.0f};
    math::Vec3f end   = {1.0f, 0.0f, 0.0f};
    float headLength  = 0.2f;
    float headWidth   = 0.1f;
    float shaftWidth  = 0.02f;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct GridGeometry
{
    math::Vec3f size = {1.0f, 1.0f, 0.0f};
    uint32_t rows = 10;
    uint32_t columns = 10;
    float lineWidth = 0.01f;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

// ── Common shapes ──

struct StarGeometry
{
    float outerRadius = 1.0f;
    float innerRadius = 0.4f;
    uint32_t points = 5;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

struct RegularPolygonGeometry
{
    float radius = 1.0f;
    uint32_t sides = 6;
    math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f};
};

// ── 2D primitive mesh generators ──
// All meshes are generated on the XY plane (Z=0), centered at origin
// unless otherwise noted.

/// Plain (sharp-cornered) rectangle.
MeshData generate_rect_mesh(const RectangleGeometry& geom);

/// Rounded rectangle with per-corner radii.
MeshData generate_rounded_rect_mesh(const RoundedRectangleGeometry& geom);

/// Circle centered at origin.
MeshData generate_circle_mesh(const CircleGeometry& geom);

/// Ellipse centered at origin.
MeshData generate_ellipse_mesh(const EllipseGeometry& geom);

/// Circular arc (open).
MeshData generate_arc_mesh(const ArcGeometry& geom);

/// Ring / annulus (outer circle with inner hole).
MeshData generate_ring_mesh(const RingGeometry& geom);

/// Thick line segment between two points.
MeshData generate_line_mesh(const LineGeometry& geom);

/// Thick polyline (connected line segments).
MeshData generate_polyline_mesh(const PolylineGeometry& geom);

/// 2D arrow (shaft + head).
MeshData generate_arrow_mesh(const ArrowGeometry& geom);

/// Grid of horizontal and vertical lines.
MeshData generate_grid_mesh(const GridGeometry& geom);

/// Regular star polygon (alternating outer/inner vertices).
MeshData generate_star_mesh(const StarGeometry& geom);

/// Regular convex polygon (triangle, square, pentagon, hexagon, etc.).
MeshData generate_regular_polygon_mesh(const RegularPolygonGeometry& geom);

} // namespace exd::geometry

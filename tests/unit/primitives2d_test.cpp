#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>

#include <cmath>

using namespace exd::geometry;

// ── Rectangle ──────────────────────────────────────────────────────────────

TEST_CASE("rect: default produces valid filled rectangle")
{
    RectangleGeometry geom;
    auto mesh = generate_rect_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);

    // All vertices on XY plane (Z=0)
    for (const auto& v : mesh.vertices)
    {
        CHECK(v.position.z == doctest::Approx(0.0f));
    }

    // All normals point toward +Z
    for (const auto& v : mesh.vertices)
    {
        CHECK(v.normal.x == doctest::Approx(0.0f));
        CHECK(v.normal.y == doctest::Approx(0.0f));
        CHECK(v.normal.z == doctest::Approx(1.0f));
    }

    // Bounds cover expected area: centered rect of size (1,1) → [-0.5, 0.5]
    CHECK(mesh.bounds.min.x == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.x == doctest::Approx(0.5f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.5f));
    CHECK(mesh.bounds.min.z == doctest::Approx(0.0f));
    CHECK(mesh.bounds.max.z == doctest::Approx(0.0f));
}

TEST_CASE("rect: degenerate zero-size returns empty")
{
    RectangleGeometry geom;
    geom.size = {0.0f, 0.0f, 0.0f};
    auto mesh = generate_rect_mesh(geom);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("rect: custom size produces correct bounds")
{
    RectangleGeometry geom;
    geom.size = {4.0f, 2.0f, 0.0f};
    auto mesh = generate_rect_mesh(geom);
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.bounds.min.x == doctest::Approx(-2.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(2.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.y == doctest::Approx(1.0f));
}

// ── Circle ─────────────────────────────────────────────────────────────────

TEST_CASE("circle: default produces triangle fan")
{
    CircleGeometry geom; // radius=1.0, segments=64
    auto mesh = generate_circle_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // 1 center + 64 perimeter = 65 vertices
    CHECK(mesh.vertices.size() == 65);
    // 64 triangles × 3 = 192 indices
    CHECK(mesh.indices.size() == 192);

    // Center vertex at origin
    CHECK(mesh.vertices[0].position.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].position.y == doctest::Approx(0.0f));

    // Perimeter vertices all at radius distance from origin
    for (size_t i = 1; i < mesh.vertices.size(); ++i)
    {
        float dist = std::sqrt(mesh.vertices[i].position.x * mesh.vertices[i].position.x +
                               mesh.vertices[i].position.y * mesh.vertices[i].position.y);
        CHECK(dist == doctest::Approx(1.0f));
    }

    // All on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));

    // Bounds
    CHECK(mesh.bounds.min.x == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.y == doctest::Approx(1.0f));
}

TEST_CASE("circle: custom radius scales correctly")
{
    CircleGeometry geom;
    geom.radius = 2.5f;
    geom.segments = 16;
    auto mesh = generate_circle_mesh(geom);

    CHECK(mesh.vertices.size() == 17); // 1 + 16
    CHECK(mesh.indices.size() == 48);  // 16 * 3

    for (size_t i = 1; i < mesh.vertices.size(); ++i)
    {
        float dist = std::sqrt(mesh.vertices[i].position.x * mesh.vertices[i].position.x +
                               mesh.vertices[i].position.y * mesh.vertices[i].position.y);
        CHECK(dist == doctest::Approx(2.5f));
    }
}

TEST_CASE("circle: minimum segments clamped to 3")
{
    CircleGeometry geom;
    geom.segments = 0;
    auto mesh = generate_circle_mesh(geom);
    // Should be clamped to 3 segments
    CHECK(mesh.vertices.size() == 4);  // 1 center + 3
    CHECK(mesh.indices.size() == 9);   // 3 * 3
}

// ── Ellipse ────────────────────────────────────────────────────────────────

TEST_CASE("ellipse: default produces triangle fan with elliptical shape")
{
    EllipseGeometry geom; // rx=1.0, ry=0.5, segs=64
    auto mesh = generate_ellipse_mesh(geom);

    CHECK(mesh.vertices.size() == 65); // 1 center + 64 perimeter
    CHECK(mesh.indices.size() == 192);  // 64 * 3

    // Perimeter vertices form ellipse: (rx*cos(t), ry*sin(t))
    for (size_t i = 1; i < mesh.vertices.size(); ++i)
    {
        float ex = mesh.vertices[i].position.x / 1.0f;
        float ey = mesh.vertices[i].position.y / 0.5f;
        CHECK(ex * ex + ey * ey == doctest::Approx(1.0f).epsilon(0.01f));
    }

    // Bounds
    CHECK(mesh.bounds.min.x == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.5f));
}

TEST_CASE("ellipse: center vertex at origin")
{
    EllipseGeometry geom;
    auto mesh = generate_ellipse_mesh(geom);
    CHECK(mesh.vertices[0].position.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].position.y == doctest::Approx(0.0f));
}

TEST_CASE("ellipse: minimum segments clamped to 3")
{
    EllipseGeometry geom;
    geom.segments = 0;
    auto mesh = generate_ellipse_mesh(geom);

    CHECK(mesh.vertices.size() == 4);  // 1 center + 3 perimeter
    CHECK(mesh.indices.size() == 9);   // 3 * 3
}

TEST_CASE("ellipse: custom radii and segment count")
{
    EllipseGeometry geom;
    geom.radiusX = 3.0f;
    geom.radiusY = 0.25f;
    geom.segments = 64;
    auto mesh = generate_ellipse_mesh(geom);

    CHECK(mesh.vertices.size() == 65); // 1 center + 64 perimeter
    CHECK(mesh.indices.size() == 192); // 64 * 3
    CHECK(mesh.bounds.min.x == doctest::Approx(-3.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(3.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-0.25f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.25f));
}

// ── Arc ────────────────────────────────────────────────────────────────────

TEST_CASE("arc: default produces filled sector")
{
    ArcGeometry geom; // radius=1, start=0, end=1.5*pi (270°), segs=64
    auto mesh = generate_arc_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    // sweep = 1.5*pi, fraction = 0.75, segs = 64*0.75 = 48
    // 1 center + 49 perimeter = 50 vertices
    CHECK(mesh.vertices.size() == 50);
    // 48 triangles × 3 = 144 indices
    CHECK(mesh.indices.size() == 144);

    // Center vertex at origin
    CHECK(mesh.vertices[0].position.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].position.y == doctest::Approx(0.0f));

    // All perimeter vertices at radius distance
    for (size_t i = 1; i < mesh.vertices.size(); ++i)
    {
        float dist = std::sqrt(mesh.vertices[i].position.x * mesh.vertices[i].position.x +
                               mesh.vertices[i].position.y * mesh.vertices[i].position.y);
        CHECK(dist == doctest::Approx(1.0f));
    }
}

TEST_CASE("arc: zero sweep returns empty")
{
    ArcGeometry geom;
    geom.startAngle = 0.0f;
    geom.endAngle = 0.0f;
    auto mesh = generate_arc_mesh(geom);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("arc: full circle produces complete fan")
{
    ArcGeometry geom;
    geom.startAngle = 0.0f;
    geom.endAngle = 2.0f * 3.14159265358979323846f;
    geom.radius = 1.0f;
    geom.segments = 32;
    auto mesh = generate_arc_mesh(geom);

    // Full sweep → 32 segments → 33 perimeter + 1 center = 34
    CHECK(mesh.vertices.size() == 34);
    CHECK(mesh.indices.size() == 96); // 32 * 3
}

TEST_CASE("arc: negative sweep (clockwise) still produces mesh")
{
    ArcGeometry geom;
    geom.startAngle = 0.0f;
    geom.endAngle = -3.14159265358979323846f; // -π (clockwise half-circle)
    geom.segments = 32;
    auto mesh = generate_arc_mesh(geom);

    // Half sweep (abs = π): fraction = 0.5, segs = 16
    // 1 center + 17 perimeter = 18 vertices, 16 * 3 = 48 indices
    CHECK(mesh.vertices.size() == 18);
    CHECK(mesh.indices.size() == 48);
}

TEST_CASE("arc: sweep larger than full circle")
{
    ArcGeometry geom;
    geom.startAngle = 0.0f;
    geom.endAngle = 4.0f * 3.14159265358979323846f; // 4π (two full circles)
    geom.segments = 32;
    auto mesh = generate_arc_mesh(geom);

    // Full sweep: fraction = 2.0, segs = 64
    // 1 center + 65 perimeter = 66 vertices, 64 * 3 = 192 indices
    CHECK(mesh.vertices.size() == 66);
    CHECK(mesh.indices.size() == 192);
}

TEST_CASE("arc: small segment count clamped to at least 1")
{
    ArcGeometry geom;
    geom.segments = 1;
    geom.startAngle = 0.0f;
    geom.endAngle = 1.57079632679f; // 90°
    auto mesh = generate_arc_mesh(geom);

    // sweep = π/2, fraction = 0.25, segs = max(1, 1*0.25) = 1
    // 1 center + 2 perimeter = 3, 1 triangle = 3 indices
    CHECK(mesh.vertices.size() == 3);
    CHECK(mesh.indices.size() == 3);
}

// ── Ring ───────────────────────────────────────────────────────────────────

TEST_CASE("ring: default produces annulus")
{
    RingGeometry geom; // outer=1.0, inner=0.5, segs=64
    auto mesh = generate_ring_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // 2*segs vertices = 128
    CHECK(mesh.vertices.size() == 128);
    // segs quads × 6 indices = 384
    CHECK(mesh.indices.size() == 384);

    // All on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));

    // Outer vertex at radius 1.0, inner at 0.5
    size_t outerCount = 0, innerCount = 0;
    for (const auto& v : mesh.vertices)
    {
        float dist = std::sqrt(v.position.x * v.position.x + v.position.y * v.position.y);
        if (dist == doctest::Approx(1.0f))
            ++outerCount;
        else if (dist == doctest::Approx(0.5f))
            ++innerCount;
    }
    CHECK(outerCount == 64);
    CHECK(innerCount == 64);
}

TEST_CASE("ring: inner >= outer is handled by swapping")
{
    RingGeometry geom;
    geom.innerRadius = 1.5f;
    geom.outerRadius = 1.0f;
    auto mesh = generate_ring_mesh(geom);
    // Should swap so inner=1.0, outer=1.5 → still valid mesh
    CHECK(!mesh.vertices.empty());
    CHECK(mesh.vertices.size() == 128);
}

TEST_CASE("ring: minimum segments clamped to 3")
{
    RingGeometry geom;
    geom.segments = 0;
    auto mesh = generate_ring_mesh(geom);
    // Clamped to 3 segments → 6 vertices, 18 indices
    CHECK(mesh.vertices.size() == 6);
    CHECK(mesh.indices.size() == 18);
}

// ── Line ───────────────────────────────────────────────────────────────────

TEST_CASE("line: default produces quad")
{
    LineGeometry geom; // start=(0,0,0), end=(1,0,0), width=1.0
    auto mesh = generate_line_mesh(geom);

    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);
    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    // All on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));

    // Normals point +Z
    for (const auto& v : mesh.vertices)
    {
        CHECK(v.normal.x == doctest::Approx(0.0f));
        CHECK(v.normal.y == doctest::Approx(0.0f));
        CHECK(v.normal.z == doctest::Approx(1.0f));
    }
}

TEST_CASE("line: zero-length returns empty")
{
    LineGeometry geom;
    geom.end = geom.start;
    auto mesh = generate_line_mesh(geom);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("line: custom width affects bounds")
{
    LineGeometry geom;
    geom.start = {0.0f, 0.0f, 0.0f};
    geom.end = {2.0f, 0.0f, 0.0f};
    geom.width = 0.5f;
    auto mesh = generate_line_mesh(geom);

    CHECK(mesh.vertices.size() == 4);
    // Line along X from 0 to 2, width 0.5 → Y spans [-0.25, 0.25]
    CHECK(mesh.bounds.min.x == doctest::Approx(0.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(2.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-0.25f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.25f));
}

// ── Polyline ───────────────────────────────────────────────────────────────

TEST_CASE("polyline: simple polyline")
{
    PolylineGeometry geom;
    geom.points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
    geom.width = 0.1f;
    auto mesh = generate_polyline_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // 2 segments × 4 vertices = 8
    CHECK(mesh.vertices.size() == 8);
    // 2 segments × 6 indices = 12
    CHECK(mesh.indices.size() == 12);
}

TEST_CASE("polyline: closed polyline with extra segment")
{
    PolylineGeometry geom;
    geom.points = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
    geom.closed = true;
    auto mesh = generate_polyline_mesh(geom);

    // 3 segments (including closed segment) × 4 = 12 vertices
    CHECK(mesh.vertices.size() == 12);
    CHECK(mesh.indices.size() == 18);
}

TEST_CASE("polyline: fewer than 2 points returns empty")
{
    PolylineGeometry geom;
    geom.points = {{0, 0, 0}};
    auto mesh = generate_polyline_mesh(geom);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("polyline: empty points returns empty")
{
    PolylineGeometry geom;
    auto mesh = generate_polyline_mesh(geom);
    CHECK(mesh.vertices.empty());
}

// ── Arrow ──────────────────────────────────────────────────────────────────

TEST_CASE("arrow: default produces shaft + head")
{
    ArrowGeometry geom;
    auto mesh = generate_arrow_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());

    // Shaft: 4 vertices, Head: 3 vertices = 7 total
    CHECK(mesh.vertices.size() == 7);
    // Shaft: 6 indices, Head: 3 indices = 9 total
    CHECK(mesh.indices.size() == 9);

    // All on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));
}

TEST_CASE("arrow: zero-length returns empty")
{
    ArrowGeometry geom;
    geom.end = geom.start;
    auto mesh = generate_arrow_mesh(geom);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("arrow: head length clamped to total length")
{
    ArrowGeometry geom;
    geom.start = {0.0f, 0.0f, 0.0f};
    geom.end = {0.1f, 0.0f, 0.0f}; // very short
    geom.headLength = 0.5f;         // longer than total length
    auto mesh = generate_arrow_mesh(geom);
    // Should still produce a valid mesh (headLength clamped)
    CHECK(!mesh.vertices.empty());
}

TEST_CASE("arrow: diagonal direction")
{
    ArrowGeometry geom;
    geom.start = {0.0f, 0.0f, 0.0f};
    geom.end   = {1.0f, 1.0f, 0.0f}; // 45° diagonal
    auto mesh = generate_arrow_mesh(geom);

    CHECK(mesh.vertices.size() == 7);
    CHECK(mesh.indices.size() == 9);

    // All vertices on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));

    // Tip should be at (1, 1, 0)
    CHECK(mesh.vertices[4].position.x == doctest::Approx(1.0f));
    CHECK(mesh.vertices[4].position.y == doctest::Approx(1.0f));
}

TEST_CASE("arrow: custom shaft and head dimensions")
{
    ArrowGeometry geom;
    geom.shaftWidth = 0.2f;
    geom.headWidth  = 0.4f;
    geom.headLength = 0.3f;
    auto mesh = generate_arrow_mesh(geom);

    CHECK(!mesh.vertices.empty());
    // Should still produce 7 vertices (4 shaft + 3 head)
    CHECK(mesh.vertices.size() == 7);
}

// ── Grid ───────────────────────────────────────────────────────────────────

TEST_CASE("grid: default produces grid lines")
{
    GridGeometry geom; // size=(1,1,0), rows=10, cols=10, lineWidth=0.01
    auto mesh = generate_grid_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // (rows+1 + cols+1) lines × 4 vertices = (11+11)*4 = 88
    CHECK(mesh.vertices.size() == 88);
    // (11+11) lines × 6 indices = 132
    CHECK(mesh.indices.size() == 132);

    // All on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));
}

TEST_CASE("grid: zero rows and cols returns empty")
{
    GridGeometry geom;
    geom.rows = 0;
    geom.columns = 0;
    auto mesh = generate_grid_mesh(geom);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("grid: single row and column")
{
    GridGeometry geom;
    geom.rows = 1;
    geom.columns = 1;
    geom.size = {1.0f, 1.0f, 0.0f};
    auto mesh = generate_grid_mesh(geom);

    // 2 horizontal + 2 vertical = 4 lines × 4 vertices = 16
    CHECK(mesh.vertices.size() == 16);
    CHECK(mesh.indices.size() == 24);
}

// ── Rounded Rectangle ──────────────────────────────────────────────────────

TEST_CASE("rounded_rect: default produces valid mesh")
{
    RoundedRectangleGeometry geom; // size=(1,1,0), all corners=0, segs=32
    auto mesh = generate_rounded_rect_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());

    // All on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));

    // Center vertex at origin
    CHECK(mesh.vertices[0].position.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].position.y == doctest::Approx(0.0f));
}

TEST_CASE("rounded_rect: with corner radii")
{
    RoundedRectangleGeometry geom;
    geom.radii = {0.1f, 0.1f, 0.1f, 0.1f};
    auto mesh = generate_rounded_rect_mesh(geom);

    // With radii > 0, more vertices than sharp corners (4)
    CHECK(mesh.vertices.size() > 4);
}

TEST_CASE("rounded_rect: radii clamped to half-size")
{
    RoundedRectangleGeometry geom;
    geom.size = {1.0f, 1.0f, 0.0f};
    geom.radii = {10.0f, 10.0f, 10.0f, 10.0f}; // way too large
    auto mesh = generate_rounded_rect_mesh(geom);

    // Should still produce a valid mesh (radii clamped)
    CHECK(!mesh.vertices.empty());
    // Bounds should still be within [-0.5, 0.5]
    CHECK(mesh.bounds.min.x >= doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.x <= doctest::Approx(0.5f));
}

TEST_CASE("rounded_rect: asymmetric per-corner radii")
{
    RoundedRectangleGeometry geom;
    geom.size = {2.0f, 2.0f, 0.0f};
    geom.radii = {0.5f, 0.2f, 0.0f, 0.3f}; // different per corner
    auto mesh = generate_rounded_rect_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    // Bounds should match the input size (centered)
    CHECK(mesh.bounds.min.x == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.y == doctest::Approx(1.0f));

    // All on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));
}

TEST_CASE("rounded_rect: sharp corners produce simple rect")
{
    RoundedRectangleGeometry geom; // all radii = 0
    auto mesh = generate_rounded_rect_mesh(geom);

    // With zero radii, should still produce mesh with corner segments
    CHECK(!mesh.vertices.empty());

    // Center vertex at origin
    CHECK(mesh.vertices[0].position.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].position.y == doctest::Approx(0.0f));
}

// ── Line: diagonal ──────────────────────────────────────────────────────────

TEST_CASE("line: diagonal line has correct bounds")
{
    LineGeometry geom;
    geom.start = {0.0f, 0.0f, 0.0f};
    geom.end   = {3.0f, 4.0f, 0.0f};
    geom.width = 0.5f;
    auto mesh = generate_line_mesh(geom);

    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);

    // The line goes from (0,0) to (3,4), width=0.5
    // Bounds should roughly contain start and end points
    CHECK(mesh.bounds.min.x <= doctest::Approx(0.0f));
    CHECK(mesh.bounds.max.x >= doctest::Approx(3.0f));
    CHECK(mesh.bounds.min.y <= doctest::Approx(0.0f));
    CHECK(mesh.bounds.max.y >= doctest::Approx(4.0f));
}

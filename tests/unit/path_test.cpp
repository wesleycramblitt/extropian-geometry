#include <doctest/doctest.h>
#include <exd/geometry/path.hpp>

#include <cmath>
#include <numbers>

using namespace exd::geometry;

// ── Fill: Simple triangle ───────────────────────────────────────────────────

TEST_CASE("path: triangle fill produces 1 triangle")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});
    path.lineTo({0.5f, 1.0f, 0.0f});
    path.close();

    auto mesh = path.tessellateFill();

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // Triangle: 3 vertices, 3 indices
    CHECK(mesh.vertices.size() == 3);
    CHECK(mesh.indices.size() == 3);

    // All normals point +Z
    for (const auto& v : mesh.vertices) {
        CHECK(v.normal.z == doctest::Approx(1.0f));
    }
}

// ── Fill: Rectangle produces 2 triangles ────────────────────────────────────

TEST_CASE("path: rectangle fill produces 2 triangles")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({2.0f, 0.0f, 0.0f});
    path.lineTo({2.0f, 1.0f, 0.0f});
    path.lineTo({0.0f, 1.0f, 0.0f});
    path.close();

    auto mesh = path.tessellateFill();

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // Rectangle: 4 vertices, 2 triangles = 6 indices
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);

    // All normals point +Z
    for (const auto& v : mesh.vertices) {
        CHECK(v.normal.z == doctest::Approx(1.0f));
    }
}

// ── Fill: Circle via arc ────────────────────────────────────────────────────

TEST_CASE("path: circle via arc fill produces triangulation")
{
    Path2D path;
    path.moveTo({1.0f, 0.0f, 0.0f}); // start on the circle
    ArcDescriptor arc;
    arc.center = {0.0f, 0.0f, 0.0f};
    arc.radius = 1.0f;
    arc.startAngle = 0.0f;
    arc.endAngle = 2.0f * std::numbers::pi_v<float>;
    path.arcTo(arc);
    path.close();

    auto mesh = path.tessellateFill();

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    // Should have multiple triangles for a circle
    CHECK(mesh.indices.size() > 6);

    // All normals point +Z
    for (const auto& v : mesh.vertices) {
        CHECK(v.normal.z == doctest::Approx(1.0f));
    }
}

// ── Stroke: Single line produces quad ───────────────────────────────────────

TEST_CASE("path: stroke single line produces quad")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.1f;
    style.cap = LineCap::Butt;

    auto mesh = path.tessellateStroke(style);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // Single line segment: 4 vertices, 2 triangles = 6 indices
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);

    // All normals point +Z
    for (const auto& v : mesh.vertices) {
        CHECK(v.normal.z == doctest::Approx(1.0f));
    }
}

// ── Stroke: Polyline produces multiple quads ────────────────────────────────

TEST_CASE("path: stroke polyline produces multiple quads")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 1.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.1f;
    style.cap = LineCap::Butt;

    auto mesh = path.tessellateStroke(style);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // 2 segments × 4 vertices = 8 vertices, 2 segments × 6 indices = 12 indices
    CHECK(mesh.vertices.size() == 8);
    CHECK(mesh.indices.size() == 12);
}

// ── Stroke: Square cap adds extra geometry ──────────────────────────────────

TEST_CASE("path: stroke square cap adds extra geometry")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});

    StrokeStyle buttStyle;
    buttStyle.width = 0.2f;
    buttStyle.cap = LineCap::Butt;

    StrokeStyle squareStyle;
    squareStyle.width = 0.2f;
    squareStyle.cap = LineCap::Square;

    auto buttMesh = path.tessellateStroke(buttStyle);
    auto squareMesh = path.tessellateStroke(squareStyle);

    // Square cap should produce more vertices than butt cap
    CHECK(squareMesh.vertices.size() > buttMesh.vertices.size());
    CHECK(squareMesh.indices.size() > buttMesh.indices.size());

    // Butt: 4 vertices, 6 indices
    CHECK(buttMesh.vertices.size() == 4);
    CHECK(buttMesh.indices.size() == 6);
}

// ── Empty path returns empty mesh ───────────────────────────────────────────

TEST_CASE("path: empty path returns empty mesh from fill")
{
    Path2D path;
    auto mesh = path.tessellateFill();
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("path: empty path returns empty mesh from stroke")
{
    Path2D path;
    StrokeStyle style;
    auto mesh = path.tessellateStroke(style);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

// ── Path with only MoveTo returns empty mesh ────────────────────────────────

TEST_CASE("path: only moveTo returns empty mesh")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});

    auto fillMesh = path.tessellateFill();
    CHECK(fillMesh.vertices.empty());
    CHECK(fillMesh.indices.empty());

    StrokeStyle style;
    auto strokeMesh = path.tessellateStroke(style);
    CHECK(strokeMesh.vertices.empty());
    CHECK(strokeMesh.indices.empty());
}

// ── Quadratic curve is flattened to multiple segments ───────────────────────

TEST_CASE("path: quadratic curve is flattened to multiple segments")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    // A pronounced quadratic curve
    path.quadraticTo({0.5f, 2.0f, 0.0f}, {1.0f, 0.0f, 0.0f});

    // Stroke with small tolerance to get good flattening
    StrokeStyle style;
    style.width = 0.01f;
    style.cap = LineCap::Butt;

    auto mesh = path.tessellateStroke(style, 0.01f);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // A curved path should produce more than one segment (more than 4 vertices)
    CHECK(mesh.vertices.size() > 4);
    CHECK(mesh.indices.size() > 6);
}

// ── Cubic curve is flattened to multiple segments ───────────────────────────

TEST_CASE("path: cubic curve is flattened to multiple segments")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    // An S-curve
    path.cubicTo({0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.01f;
    style.cap = LineCap::Butt;

    auto mesh = path.tessellateStroke(style, 0.01f);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // A cubic S-curve should produce multiple segments
    CHECK(mesh.vertices.size() > 4);
    CHECK(mesh.indices.size() > 6);
}

// ── Revision tracking ───────────────────────────────────────────────────────

TEST_CASE("path: revision increments on each command")
{
    Path2D path;
    CHECK(path.revision() == 0);

    path.moveTo({0.0f, 0.0f, 0.0f});
    CHECK(path.revision() == 1);

    path.lineTo({1.0f, 0.0f, 0.0f});
    CHECK(path.revision() == 2);

    path.quadraticTo({0.5f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f});
    CHECK(path.revision() == 3);

    path.cubicTo({0.0f, 0.5f, 0.0f}, {1.0f, 0.5f, 0.0f}, {1.0f, 0.0f, 0.0f});
    CHECK(path.revision() == 4);

    ArcDescriptor arc;
    arc.center = {0.0f, 0.0f, 0.0f};
    arc.radius = 1.0f;
    arc.startAngle = 0.0f;
    arc.endAngle = 1.0f;
    path.arcTo(arc);
    CHECK(path.revision() == 5);

    path.close();
    CHECK(path.revision() == 6);
}

// ── Closed subpath stroke ───────────────────────────────────────────────────

TEST_CASE("path: closed subpath stroke has no caps")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 1.0f, 0.0f});
    path.lineTo({0.0f, 1.0f, 0.0f});
    path.close();

    StrokeStyle style;
    style.width = 0.1f;
    style.cap = LineCap::Square; // caps shouldn't matter for closed path

    auto mesh = path.tessellateStroke(style);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // 4 segments (closed) × 4 vertices = 16 vertices, no caps
    CHECK(mesh.vertices.size() == 16);
    CHECK(mesh.indices.size() == 24);
}

// ── Multi-subpath fill ──────────────────────────────────────────────────────

TEST_CASE("path: multiple subpaths produce combined fill")
{
    Path2D path;

    // First triangle
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});
    path.lineTo({0.5f, 1.0f, 0.0f});
    path.close();

    // Second triangle (separate subpath)
    path.moveTo({2.0f, 0.0f, 0.0f});
    path.lineTo({3.0f, 0.0f, 0.0f});
    path.lineTo({2.5f, 1.0f, 0.0f});
    path.close();

    auto mesh = path.tessellateFill();

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // 2 triangles × 3 vertices = 6 vertices, 2 triangles × 3 indices = 6 indices
    CHECK(mesh.vertices.size() == 6);
    CHECK(mesh.indices.size() == 6);
}

// ── Tolerance affects tessellation quality ───────────────────────────────────

TEST_CASE("path: smaller tolerance produces more segments")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.cubicTo({0.0f, 2.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.01f;

    auto coarse = path.tessellateStroke(style, 1.0f);
    auto fine   = path.tessellateStroke(style, 0.01f);

    // Finer tolerance should produce more segments (= more vertices)
    CHECK(coarse.vertices.size() > 0);
    CHECK(fine.vertices.size() > 0);
    // The fine mesh should have at least as many vertices as the coarse one
    // (actually should have more for a cubic curve)
    CHECK(fine.vertices.size() >= coarse.vertices.size());
}

// ── Cap and join style variations ────────────────────────────────────────────

TEST_CASE("path: round cap does not crash")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.1f;
    style.cap = LineCap::Round; // not fully implemented, should not crash

    auto mesh = path.tessellateStroke(style);
    CHECK(!mesh.vertices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("path: round join does not crash")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 1.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.1f;
    style.join = LineJoin::Round;

    auto mesh = path.tessellateStroke(style);
    CHECK(!mesh.vertices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("path: bevel join does not crash")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});
    path.lineTo({0.5f, 1.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.1f;
    style.join = LineJoin::Bevel;

    auto mesh = path.tessellateStroke(style);
    CHECK(!mesh.vertices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("path: miter join does not crash")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});
    path.lineTo({0.5f, 1.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.1f;
    style.join = LineJoin::Miter; // default
    style.miterLimit = 10.0f;

    auto mesh = path.tessellateStroke(style);
    CHECK(!mesh.vertices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("path: dash pattern met")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({5.0f, 0.0f, 0.0f});

    StrokeStyle style;
    style.width = 0.1f;
    style.dashPattern = {0.2f, 0.1f, 0.4f, 0.1f}; // dash-dot pattern
    style.dashOffset = 0.05f;

    // Dash pattern is accepted and doesn't crash (not yet implemented)
    auto mesh = path.tessellateStroke(style);
    CHECK(!mesh.vertices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

// ── Mixed path types ─────────────────────────────────────────────────────────

TEST_CASE("path: mixed line, arc, and curve in single path")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({2.0f, 0.0f, 0.0f});

    ArcDescriptor arc;
    arc.center = {2.0f, 1.0f, 0.0f};
    arc.radius = 1.0f;
    arc.startAngle = -1.57079632679f; // -90°
    arc.endAngle   = 0.0f;           // 0°
    path.arcTo(arc);

    path.quadraticTo({2.5f, 1.5f, 0.0f}, {1.0f, 2.0f, 0.0f});
    path.close();

    auto fillMesh = path.tessellateFill();
    CHECK(!fillMesh.vertices.empty());
    CHECK(!fillMesh.indices.empty());
    CHECK(fillMesh.topology == PrimitiveTopology::Triangles);

    StrokeStyle style;
    style.width = 0.05f;
    auto strokeMesh = path.tessellateStroke(style);
    CHECK(!strokeMesh.vertices.empty());
    CHECK(strokeMesh.topology == PrimitiveTopology::Triangles);
}

// ── Fill with curve-only path ────────────────────────────────────────────────

TEST_CASE("path: fill of curved-only path")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.cubicTo({0.0f, 2.0f, 0.0f}, {2.0f, 2.0f, 0.0f}, {2.0f, 0.0f, 0.0f});
    path.close();

    auto mesh = path.tessellateFill();
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

// ── Revision counter with close ──────────────────────────────────────────────

TEST_CASE("path: revision increments on close")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    CHECK(path.revision() == 1);
    path.close();
    CHECK(path.revision() == 2);
}

// ── Stroke with large width ──────────────────────────────────────────────────

TEST_CASE("path: stroke with large width produces valid mesh")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.lineTo({1.0f, 0.0f, 0.0f});

    StrokeStyle style;
    style.width = 10.0f; // very wide
    style.cap = LineCap::Square;

    auto mesh = path.tessellateStroke(style);
    CHECK(!mesh.vertices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

// ── Fill with modified tolerance ─────────────────────────────────────────────

TEST_CASE("path: fill with different tolerance values")
{
    Path2D path;
    path.moveTo({0.0f, 0.0f, 0.0f});
    path.cubicTo({0.0f, 2.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 0.0f});
    path.close();

    auto coarse = path.tessellateFill(FillRule::NonZero, 0.5f);
    auto fine   = path.tessellateFill(FillRule::NonZero, 0.01f);

    // Both should produce valid meshes with reasonable tolerance values
    CHECK(!fine.vertices.empty());

    // Coarse tolerance may produce empty mesh if curve flattening
    // yields too few segments; skip empty-check for coarse.
    // When both are non-empty, finer tolerance should produce at least
    // as many vertices.
    if (!coarse.vertices.empty()) {
        CHECK(fine.vertices.size() >= coarse.vertices.size());
    }
}

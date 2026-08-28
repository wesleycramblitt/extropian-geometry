#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>
#include <exd/math/mat4.hpp>

#include <span>

using namespace exd::geometry;

// ── compute_bounds ─────────────────────────────────────────────────────────

TEST_CASE("compute_bounds: single vertex")
{
    Vertex v;
    v.position = {1.0f, 2.0f, 3.0f};
    std::vector<Vertex> verts = {v};
    auto b = compute_bounds(verts);

    CHECK(b.min.x == doctest::Approx(1.0f));
    CHECK(b.min.y == doctest::Approx(2.0f));
    CHECK(b.min.z == doctest::Approx(3.0f));
    CHECK(b.max.x == doctest::Approx(1.0f));
    CHECK(b.max.y == doctest::Approx(2.0f));
    CHECK(b.max.z == doctest::Approx(3.0f));
}

TEST_CASE("compute_bounds: multiple vertices")
{
    std::vector<Vertex> verts(3);
    verts[0].position = {-1.0f, -2.0f, -3.0f};
    verts[1].position = {0.0f, 5.0f, 0.0f};
    verts[2].position = {2.0f, 1.0f, 4.0f};
    auto b = compute_bounds(verts);

    CHECK(b.min.x == doctest::Approx(-1.0f));
    CHECK(b.min.y == doctest::Approx(-2.0f));
    CHECK(b.min.z == doctest::Approx(-3.0f));
    CHECK(b.max.x == doctest::Approx(2.0f));
    CHECK(b.max.y == doctest::Approx(5.0f));
    CHECK(b.max.z == doctest::Approx(4.0f));
}

TEST_CASE("compute_bounds: empty returns zero")
{
    std::vector<Vertex> empty;
    auto b = compute_bounds(empty);
    CHECK(b.min.x == doctest::Approx(0.0f));
    CHECK(b.min.y == doctest::Approx(0.0f));
    CHECK(b.min.z == doctest::Approx(0.0f));
    CHECK(b.max.x == doctest::Approx(0.0f));
    CHECK(b.max.y == doctest::Approx(0.0f));
    CHECK(b.max.z == doctest::Approx(0.0f));
}

TEST_CASE("compute_bounds: works with std::span")
{
    std::vector<Vertex> verts(2);
    verts[0].position = {-5.0f, 0.0f, 0.0f};
    verts[1].position = {5.0f, 0.0f, 0.0f};
    std::span<const Vertex> span = verts;
    auto b = compute_bounds(span);

    CHECK(b.min.x == doctest::Approx(-5.0f));
    CHECK(b.max.x == doctest::Approx(5.0f));
}

// ── merge_meshes ───────────────────────────────────────────────────────────

TEST_CASE("merge_meshes: single mesh identity")
{
    auto rect = generate_rect_mesh({});
    std::vector<MeshData> meshes = {rect};
    auto merged = merge_meshes(meshes);

    CHECK(merged.vertices.size() == rect.vertices.size());
    CHECK(merged.indices.size() == rect.indices.size());
    CHECK(merged.topology == rect.topology);
}

TEST_CASE("merge_meshes: two meshes")
{
    auto r1 = generate_rect_mesh({});
    auto r2 = generate_rect_mesh({});
    std::vector<MeshData> meshes = {r1, r2};
    auto merged = merge_meshes(meshes);

    CHECK(merged.vertices.size() == r1.vertices.size() + r2.vertices.size());
    CHECK(merged.indices.size() == r1.indices.size() + r2.indices.size());

    // Second mesh's indices should be offset by first mesh's vertex count
    CHECK(merged.indices[r1.indices.size()] >= r1.vertices.size());
}

TEST_CASE("merge_meshes: empty input returns empty")
{
    std::vector<MeshData> empty;
    auto merged = merge_meshes(empty);
    CHECK(merged.vertices.empty());
    CHECK(merged.indices.empty());
}

TEST_CASE("merge_meshes: three meshes with correct index offsets")
{
    auto rect = generate_rect_mesh({});
    std::vector<MeshData> meshes = {rect, rect, rect};
    auto merged = merge_meshes(meshes);

    CHECK(merged.vertices.size() == rect.vertices.size() * 3);
    CHECK(merged.indices.size() == rect.indices.size() * 3);

    // Verify index offsets for each mesh
    // Mesh 0: indices 0..5, referencing vertices 0..3
    // Mesh 1: indices 6..11, referencing vertices 4..7
    // Mesh 2: indices 12..17, referencing vertices 8..11
    for (size_t i = 0; i < rect.indices.size(); ++i)
    {
        CHECK(merged.indices[i] == rect.indices[i]);
    }
    for (size_t i = 0; i < rect.indices.size(); ++i)
    {
        CHECK(merged.indices[rect.indices.size() + i] ==
              rect.indices[i] + static_cast<uint32_t>(rect.vertices.size()));
    }
    for (size_t i = 0; i < rect.indices.size(); ++i)
    {
        CHECK(merged.indices[2 * rect.indices.size() + i] ==
              rect.indices[i] + 2 * static_cast<uint32_t>(rect.vertices.size()));
    }
}

TEST_CASE("merge_meshes: works with std::span")
{
    auto rect = generate_rect_mesh({});
    std::vector<MeshData> meshes = {rect, rect};
    std::span<const MeshData> span = meshes;
    auto merged = merge_meshes(span);

    CHECK(merged.vertices.size() == rect.vertices.size() * 2);
}

// ── transform_mesh ─────────────────────────────────────────────────────────

TEST_CASE("transform_mesh: identity transform")
{
    auto rect = generate_rect_mesh({});
    auto identity = exd::math::Mat4::identity();
    auto transformed = transform_mesh(rect, identity);

    CHECK(transformed.vertices.size() == rect.vertices.size());
    // Positions should be unchanged
    for (size_t i = 0; i < rect.vertices.size(); ++i)
    {
        CHECK(transformed.vertices[i].position.x ==
              doctest::Approx(rect.vertices[i].position.x));
        CHECK(transformed.vertices[i].position.y ==
              doctest::Approx(rect.vertices[i].position.y));
        CHECK(transformed.vertices[i].position.z ==
              doctest::Approx(rect.vertices[i].position.z));
    }
}

TEST_CASE("transform_mesh: translation")
{
    auto rect = generate_rect_mesh({});
    auto b = rect.bounds;

    // Build translation matrix: translate by (10, 0, 0)
    exd::math::Mat4 translate = exd::math::Mat4::identity();
    translate.m[12] = 10.0f; // x translation (column-major)

    auto transformed = transform_mesh(rect, translate);
    auto tb = transformed.bounds;

    CHECK(tb.min.x == doctest::Approx(b.min.x + 10.0f));
    CHECK(tb.max.x == doctest::Approx(b.max.x + 10.0f));
    // Y and Z unchanged
    CHECK(tb.min.y == doctest::Approx(b.min.y));
    CHECK(tb.max.y == doctest::Approx(b.max.y));
    CHECK(tb.min.z == doctest::Approx(b.min.z));
    CHECK(tb.max.z == doctest::Approx(b.max.z));
}

TEST_CASE("transform_mesh: preserves indices")
{
    auto rect = generate_rect_mesh({});
    auto identity = exd::math::Mat4::identity();
    auto transformed = transform_mesh(rect, identity);

    CHECK(transformed.indices == rect.indices);
}

TEST_CASE("transform_mesh: normals not transformed when flag is false")
{
    auto rect = generate_rect_mesh({});

    // Rotation by +90° around Z (column-major)
    exd::math::Mat4 rotate90Z = exd::math::Mat4::identity();
    rotate90Z.m[0] = 0.0f;  rotate90Z.m[1] = 1.0f;
    rotate90Z.m[4] = -1.0f; rotate90Z.m[5] = 0.0f;

    auto transformed = transform_mesh(rect, rotate90Z, false);

    // Normals should be unchanged (original normals are all +Z)
    for (size_t i = 0; i < rect.vertices.size(); ++i)
    {
        CHECK(transformed.vertices[i].normal.x ==
              doctest::Approx(rect.vertices[i].normal.x));
        CHECK(transformed.vertices[i].normal.y ==
              doctest::Approx(rect.vertices[i].normal.y));
        CHECK(transformed.vertices[i].normal.z ==
              doctest::Approx(rect.vertices[i].normal.z));
    }
}

TEST_CASE("transform_mesh: normals transformed when flag is true")
{
    auto rect = generate_rect_mesh({});

    // Rotation by +90° around Z
    exd::math::Mat4 rotate90Z = exd::math::Mat4::identity();
    rotate90Z.m[0] = 0.0f;  rotate90Z.m[1] = 1.0f;
    rotate90Z.m[4] = -1.0f; rotate90Z.m[5] = 0.0f;

    auto transformed = transform_mesh(rect, rotate90Z, true);

    // Original normals are (0, 0, 1). After 90° Z rotation, still (0, 0, 1)
    // because rotation around Z doesn't affect Z component
    for (size_t i = 0; i < rect.vertices.size(); ++i)
    {
        CHECK(transformed.vertices[i].normal.z ==
              doctest::Approx(1.0f));
    }
}

TEST_CASE("transform_mesh: topology preserved")
{
    auto rect = generate_rect_mesh({});
    auto identity = exd::math::Mat4::identity();
    auto transformed = transform_mesh(rect, identity);

    CHECK(transformed.topology == rect.topology);
    CHECK(transformed.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("transform_mesh: scales positions correctly")
{
    auto rect = generate_rect_mesh({});

    // Scale by 2 in all axes
    exd::math::Mat4 scale = exd::math::Mat4::identity();
    scale.m[0] = 2.0f;
    scale.m[5] = 2.0f;
    scale.m[10] = 2.0f;

    auto transformed = transform_mesh(rect, scale);

    // Original bounds: [-0.5, 0.5] → scaled: [-1.0, 1.0]
    CHECK(transformed.bounds.min.x == doctest::Approx(-1.0f));
    CHECK(transformed.bounds.max.x == doctest::Approx(1.0f));
    CHECK(transformed.bounds.min.y == doctest::Approx(-1.0f));
    CHECK(transformed.bounds.max.y == doctest::Approx(1.0f));
}

TEST_CASE("transform_mesh: non-uniform scale affects normals when flag is true")
{
    auto rect = generate_rect_mesh({});

    // Non-uniform scale: stretch X by 3, Y by 0.5
    exd::math::Mat4 scale = exd::math::Mat4::identity();
    scale.m[0] = 3.0f;
    scale.m[5] = 0.5f;
    scale.m[10] = 1.0f;

    auto transformed = transform_mesh(rect, scale, true);

    // Original normals are (0, 0, 1). After non-uniform scale, Z is unchanged
    // but the transform_direction applies the upper 3x3 to normals.
    // Normal (0,0,1) × diag(3, 0.5, 1) = (0, 0, 1) — still +Z
    for (const auto& v : transformed.vertices)
    {
        CHECK(v.normal.z == doctest::Approx(1.0f));
    }
}

TEST_CASE("transform_mesh: non-uniform scale does not affect normals when flag is false")
{
    auto rect = generate_rect_mesh({});

    exd::math::Mat4 scale = exd::math::Mat4::identity();
    scale.m[0] = 3.0f;
    scale.m[5] = 0.5f;
    scale.m[10] = 1.0f;

    auto transformed = transform_mesh(rect, scale, false);

    // Positions should be scaled
    CHECK(transformed.bounds.min.x != doctest::Approx(rect.bounds.min.x));
    // Normals should be unchanged
    for (size_t i = 0; i < rect.vertices.size(); ++i)
    {
        CHECK(transformed.vertices[i].normal.x == doctest::Approx(rect.vertices[i].normal.x));
        CHECK(transformed.vertices[i].normal.y == doctest::Approx(rect.vertices[i].normal.y));
        CHECK(transformed.vertices[i].normal.z == doctest::Approx(rect.vertices[i].normal.z));
    }
}

// ── merge_meshes with mismatched topologies ─────────────────────────────────

TEST_CASE("merge_meshes: mismatched topologies still merge geometry")
{
    // Create a triangle mesh and a point mesh
    MeshBuilder builder;
    Vertex v;
    auto a = builder.add_vertex(v);
    auto b = builder.add_vertex(v);
    auto c = builder.add_vertex(v);
    builder.add_triangle(a, b, c);
    auto triMesh = builder.build(PrimitiveTopology::Triangles);

    MeshBuilder builder2;
    builder2.add_vertex(v);
    builder2.add_vertex(v);
    auto pointMesh = builder2.build(PrimitiveTopology::Points);

    std::vector<MeshData> meshes = {triMesh, pointMesh};
    auto merged = merge_meshes(meshes);

    // Should merge the geometry (falls back to first mesh's topology)
    CHECK(merged.vertices.size() == triMesh.vertices.size() + pointMesh.vertices.size());
    CHECK(merged.indices.size() == triMesh.indices.size() + pointMesh.indices.size());
    CHECK(merged.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("merge_meshes: single mesh with bounds preserved")
{
    auto rect = generate_rect_mesh({});
    std::vector<MeshData> meshes = {rect};
    auto merged = merge_meshes(meshes);

    CHECK(merged.bounds.min.x == doctest::Approx(rect.bounds.min.x));
    CHECK(merged.bounds.max.x == doctest::Approx(rect.bounds.max.x));
    CHECK(merged.bounds.min.y == doctest::Approx(rect.bounds.min.y));
    CHECK(merged.bounds.max.y == doctest::Approx(rect.bounds.max.y));
}

TEST_CASE("compute_bounds: all vertices share same position")
{
    std::vector<Vertex> verts(5);
    for (auto& v : verts)
        v.position = {3.0f, 4.0f, 5.0f};
    auto b = compute_bounds(verts);

    CHECK(b.min.x == doctest::Approx(3.0f));
    CHECK(b.max.x == doctest::Approx(3.0f));
    CHECK(b.min.y == doctest::Approx(4.0f));
    CHECK(b.max.y == doctest::Approx(4.0f));
    CHECK(b.min.z == doctest::Approx(5.0f));
    CHECK(b.max.z == doctest::Approx(5.0f));
}

// ── triangulate_polygon ────────────────────────────────────────────────────

namespace
{
float tri_area2(const exd::math::Vec3f& a, const exd::math::Vec3f& b, const exd::math::Vec3f& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
} // namespace

TEST_CASE("triangulate: convex quad")
{
    const std::vector<exd::math::Vec3f> outline = {{0,0,0},{1,0,0},{1,1,0},{0,1,0}};
    auto tris = triangulate_polygon(outline);
    REQUIRE(tris.size() == 6);
    for (uint32_t i : tris)
        CHECK(i < 4);

    float total = 0.0f;
    for (size_t t = 0; t < tris.size(); t += 3)
    {
        const float a2 = tri_area2(outline[tris[t]], outline[tris[t+1]], outline[tris[t+2]]);
        CHECK(a2 > 0.0f);   // CCW in the XY plane
        total += a2;
    }
    CHECK(total == doctest::Approx(2.0f));   // unit square: area 1 (2 * area)
}

TEST_CASE("triangulate: concave polygon")
{
    const std::vector<exd::math::Vec3f> outline = {{0,0,0},{2,0,0},{2,1,0},{1,1,0},{1,2,0},{0,2,0}};
    auto tris = triangulate_polygon(outline);
    REQUIRE(tris.size() == 12);   // 4 triangles for an L-shape
    for (uint32_t i : tris)
        CHECK(i < 6);

    float total = 0.0f;
    for (size_t t = 0; t < tris.size(); t += 3)
        total += tri_area2(outline[tris[t]], outline[tris[t+1]], outline[tris[t+2]]);
    CHECK(std::abs(total - 6.0f) < 1e-4f);   // L area 3 → 2 * area = 6
}

TEST_CASE("triangulate: polygon with hole")
{
    const std::vector<exd::math::Vec3f> outline = {{0,0,0},{1,0,0},{1,1,0},{0,1,0}};
    const std::vector<std::vector<exd::math::Vec3f>> holes = {
        {{0.25f, 0.25f, 0.0f}, {0.25f, 0.75f, 0.0f}, {0.75f, 0.75f, 0.0f}, {0.75f, 0.25f, 0.0f}}
    };
    auto tris = triangulate_polygon(outline, holes);
    REQUIRE(tris.size() == 24);   // 8 triangles
    for (uint32_t i : tris)
        CHECK(i < 8);

    std::vector<exd::math::Vec3f> combined = outline;
    combined.insert(combined.end(), holes[0].begin(), holes[0].end());

    float  total     = 0.0f;
    bool   usesOutline = false;
    bool   usesHole    = false;
    for (size_t t = 0; t < tris.size(); t += 3)
    {
        total += tri_area2(combined[tris[t]], combined[tris[t+1]], combined[tris[t+2]]);
        for (uint32_t k = 0; k < 3; ++k)
        {
            usesOutline = usesOutline || tris[t + k] < 4;
            usesHole    = usesHole    || tris[t + k] >= 4;
        }
    }
    CHECK(std::abs(total - 1.5f) < 1e-4f);   // outer (1) − inner (0.25) = 0.75 → 2 * area
    CHECK(usesOutline);
    CHECK(usesHole);
}

TEST_CASE("triangulate: reversed winding")
{
    // Same CCW square traversed clockwise — same triangle count and area.
    const std::vector<exd::math::Vec3f> outline = {{0,1,0},{1,1,0},{1,0,0},{0,0,0}};
    auto tris = triangulate_polygon(outline);
    REQUIRE(tris.size() == 6);
    for (uint32_t i : tris)
        CHECK(i < 4);

    float total = 0.0f;
    for (size_t t = 0; t < tris.size(); t += 3)
        total += tri_area2(outline[tris[t]], outline[tris[t+1]], outline[tris[t+2]]);
    CHECK(std::abs(total) == doctest::Approx(2.0f));   // same absolute area
}

TEST_CASE("triangulate: degenerate")
{
    CHECK(triangulate_polygon({}).empty());
    CHECK(triangulate_polygon({{0,0,0},{1,0,0}}).empty());
    // Collinear points: either empty or a non-degenerate result — but never garbage.
    const auto collinear = triangulate_polygon({{0,0,0},{1,0,0},{2,0,0},{3,0,0}});
    CHECK(collinear.size() % 3 == 0);
}

// ── weld_vertices ──────────────────────────────────────────────────────────

TEST_CASE("weld: duplicates merge")
{
    auto box = generate_box_mesh({});
    REQUIRE(box.vertices.size() == 24);   // 6 faces × 4 corners
    const auto before = compute_bounds(box.vertices);

    auto welded = weld_vertices(box, 1e-4f);
    CHECK(welded.vertices.size() == 8);   // 8 unique corner positions
    CHECK(welded.indices.size() == box.indices.size());

    for (size_t i = 0; i < welded.vertices.size(); ++i)
        for (size_t j = i + 1; j < welded.vertices.size(); ++j)
            CHECK((welded.vertices[i].position - welded.vertices[j].position).length() > 1e-3f);

    for (uint32_t i : welded.indices)
        CHECK(i < 8);

    CHECK(welded.bounds.min.x == doctest::Approx(before.min.x));
    CHECK(welded.bounds.max.x == doctest::Approx(before.max.x));
    CHECK(welded.bounds.min.y == doctest::Approx(before.min.y));
    CHECK(welded.bounds.max.y == doctest::Approx(before.max.y));
    CHECK(welded.bounds.min.z == doctest::Approx(before.min.z));
    CHECK(welded.bounds.max.z == doctest::Approx(before.max.z));
}

TEST_CASE("weld: epsilon zero returns copy")
{
    auto box = generate_box_mesh({});
    auto welded = weld_vertices(box, 0.0f);
    CHECK(welded.vertices.size() == box.vertices.size());
    CHECK(welded.indices == box.indices);
    CHECK(welded.topology == box.topology);
}

TEST_CASE("weld: no-op far apart")
{
    // Pre-welded unit box (8 unique verts); second copy translated far away.
    auto unit = weld_vertices(generate_box_mesh({}), 1e-4f);
    REQUIRE(unit.vertices.size() == 8);

    exd::math::Mat4 t10 = exd::math::Mat4::identity();
    t10.m[12] = 10.0f;
    const auto far = transform_mesh(unit, t10);

    const std::vector<MeshData> meshes = {unit, far};
    auto merged = merge_meshes(meshes);
    REQUIRE(merged.vertices.size() == 16);

    auto welded = weld_vertices(merged, 1e-6f);
    CHECK(welded.vertices.size() == merged.vertices.size());
    CHECK(welded.indices.size() == merged.indices.size());
}

// ── recompute_normals ──────────────────────────────────────────────────────

TEST_CASE("normals: flat")
{
    auto box = generate_box_mesh({});
    auto flat = recompute_normals(box, NormalMode::Flat);

    CHECK(flat.vertices.size() == 36);   // 3 verts × 12 faces
    CHECK(flat.indices.size() == 36);
    for (size_t i = 0; i < flat.indices.size(); ++i)
        CHECK(flat.indices[i] == i);

    for (size_t f = 0; f < 12; ++f)
    {
        const exd::math::Vec3f n = flat.vertices[f * 3].normal;
        CHECK(std::abs(n.length() - 1.0f) < 1e-4f);
        int axes = 0;
        if (std::abs(n.x) > 1e-5f) ++axes;
        if (std::abs(n.y) > 1e-5f) ++axes;
        if (std::abs(n.z) > 1e-5f) ++axes;
        CHECK(axes == 1);   // axis-aligned for a box
        CHECK(flat.vertices[f * 3 + 1].normal == n);
        CHECK(flat.vertices[f * 3 + 2].normal == n);
    }
}

TEST_CASE("normals: smooth")
{
    SphereGeometry sg;
    auto sphere = generate_sphere_mesh(sg);
    auto smooth = recompute_normals(sphere, NormalMode::Smooth);

    CHECK(smooth.vertices.size() == sphere.vertices.size());
    CHECK(smooth.indices == sphere.indices);

    // NOTE: the default UV sphere is wound INWARD — its geometric face normals
    // oppose the generator's analytic normals (dot ≈ −1). recompute_normals
    // follows the mesh topology, so each recomputed normal is parallel to the
    // analytic normal or its reflection.
    for (size_t i = 0; i < smooth.vertices.size(); ++i)
    {
        CHECK(std::abs(smooth.vertices[i].normal.length() - 1.0f) < 1e-4f);
        const auto analytic = sphere.vertices[i].normal;
        if (analytic.length() > 0.5f)
        {
            const float dPos = (smooth.vertices[i].normal - analytic).length();
            const float dNeg = (smooth.vertices[i].normal + analytic).length();
            CHECK(std::min(dPos, dNeg) < 0.05f);
        }
    }
}

TEST_CASE("normals: empty")
{
    MeshData empty;
    auto out = recompute_normals(empty);
    CHECK(out.vertices.empty());
    CHECK(out.indices.empty());
}

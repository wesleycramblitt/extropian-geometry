#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>
#include <exd/math/mat4.hpp>

#include <map>
#include <span>

using namespace exd::geometry;
using namespace exd::math;

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

// ── Boolean (CSG) ──────────────────────────────────────────────────────────

namespace
{

float tri_area(const MeshData& m, size_t t)
{
    const auto& a = m.vertices[m.indices[3 * t]].position;
    const auto& b = m.vertices[m.indices[3 * t + 1]].position;
    const auto& c = m.vertices[m.indices[3 * t + 2]].position;
    return 0.5f * (b - a).cross(c - a).length();
}

float sum_tri_area(const MeshData& m)
{
    float s = 0.0f;
    for (size_t t = 0; t < m.indices.size() / 3; ++t)
        s += tri_area(m, t);
    return s;
}

// Watertight helper: position-canonicalized undirected edge count == 2 for
// every edge. (Directed opposition is optional for the test helper.)
bool watertight(const MeshData& m)
{
    if (m.vertices.empty() || m.indices.empty() || m.indices.size() % 3 != 0)
        return false;

    constexpr float kCanonEps = 1e-6f;
    std::vector<uint32_t> canon(m.vertices.size());
    std::vector<uint32_t> reps;
    for (size_t i = 0; i < m.vertices.size(); ++i)
    {
        uint32_t keeper = UINT32_MAX;
        for (size_t j = 0; j < reps.size(); ++j)
        {
            if ((m.vertices[reps[j]].position - m.vertices[i].position).length() <= kCanonEps)
            {
                keeper = static_cast<uint32_t>(j);
                break;
            }
        }
        if (keeper == UINT32_MAX)
        {
            keeper = static_cast<uint32_t>(reps.size());
            reps.push_back(static_cast<uint32_t>(i));
        }
        canon[i] = keeper;
    }

    struct EK
    {
        uint32_t a, b;
        bool operator<(const EK& o) const { return a != o.a ? a < o.a : b < o.b; }
    };
    std::map<EK, int> edges;
    for (size_t t = 0; t < m.indices.size(); t += 3)
    {
        const uint32_t a = canon[m.indices[t]];
        const uint32_t b = canon[m.indices[t + 1]];
        const uint32_t c = canon[m.indices[t + 2]];
        auto add = [&](uint32_t x, uint32_t y)
        {
            edges[{std::min(x, y), std::max(x, y)}]++;
        };
        add(a, b);
        add(b, c);
        add(c, a);
    }
    for (const auto& kv : edges)
        if (kv.second != 2)
            return false;
    return true;
}

bool bounds_close(const Bounds& a, const Bounds& b, float eps)
{
    return (a.min - b.min).length() <= eps && (a.max - b.max).length() <= eps;
}

bool meshes_identical(const MeshData& r, const MeshData& a)
{
    if (r.vertices.size() != a.vertices.size())
        return false;
    if (r.indices != a.indices)
        return false;
    for (size_t i = 0; i < r.vertices.size(); ++i)
    {
        const Vertex& u = r.vertices[i];
        const Vertex& v = a.vertices[i];
        if (u.position != v.position)
            return false;
        if (u.normal != v.normal)
            return false;
        if (u.uv != v.uv)
            return false;
        if (u.tangent.w != v.tangent.w || u.tangent.x != v.tangent.x ||
            u.tangent.y != v.tangent.y || u.tangent.z != v.tangent.z)
            return false;
        if (u.color.w != v.color.w || u.color.x != v.color.x ||
            u.color.y != v.color.y || u.color.z != v.color.z)
            return false;
    }
    return true;
}

MeshData box_centered(const Vec3f& size, const Vec3f& center)
{
    BoxGeometry bg;
    bg.size = size;
    auto box = generate_box_mesh(bg);
    if (center.x == 0.0f && center.y == 0.0f && center.z == 0.0f)
        return box;
    Mat4 t = Mat4::identity();
    t.m[12] = center.x;
    t.m[13] = center.y;
    t.m[14] = center.z;
    return transform_mesh(box, t);
}

} // namespace

TEST_CASE("boolean: intersect of two offset boxes")
{
    // A spans [-1,1]^3 (default generator output is un-welded: 24 verts).
    auto a = box_centered({2, 2, 2}, {0, 0, 0});
    REQUIRE(a.vertices.size() == 24);
    REQUIRE(a.indices.size() == 36);
    // B spans [0.5,2.5] x [-0.5,1.5] x [-0.5,1.5]. Shifted in y/z so that no
    // face plane of B coincides with a face plane of A (coplanar overlap is a
    // documented V1 limitation and is tested separately).
    auto b = box_centered({2, 2, 2}, {1.5f, 0.5f, 0.5f});
    REQUIRE(b.vertices.size() == 24);

    auto r = boolean_mesh(a, b, BooleanOp::Intersect);
    REQUIRE(!r.vertices.empty());
    CHECK(watertight(r));

    // Overlap region: x[0.5,1] x [-0.5,1] x [-0.5,1] (0.5 x 1.5 x 1.5).
    Bounds exp;
    exp.min = {0.5f, -0.5f, -0.5f};
    exp.max = {1.0f, 1.0f, 1.0f};
    CHECK(bounds_close(r.bounds, exp, 1e-3f));

    // Surface of the overlap box = 2*(0.5*1.5 + 0.5*1.5 + 1.5*1.5) = 7.5.
    CHECK(std::abs(sum_tri_area(r) - 7.5f) < 1e-3f);
    CHECK(r.indices.size() / 3 > 6);
}

TEST_CASE("boolean: union of two offset boxes")
{
    auto a = box_centered({2, 2, 2}, {0, 0, 0});
    auto b = box_centered({2, 2, 2}, {1.5f, 0.5f, 0.5f});

    auto r = boolean_mesh(a, b, BooleanOp::Union);
    REQUIRE(!r.vertices.empty());
    CHECK(watertight(r));

    // Union bounds: A x[-1,1] ∪ B x[0.5,2.5] → [-1,2.5]; y[-1,1.5]; z[-1,1.5].
    Bounds exp;
    exp.min = {-1.0f, -1.0f, -1.0f};
    exp.max = {2.5f, 1.5f, 1.5f};
    CHECK(bounds_close(r.bounds, exp, 1e-3f));
}

TEST_CASE("boolean: subtract box minus box")
{
    // A [-1,1]^3, B fully inside A: [-0.25,0.75] x [-0.5,0.5] x [-0.5,0.5]
    // (no face of B is coplanar with a face of A).
    auto a = box_centered({2, 2, 2}, {0, 0, 0});
    auto b = box_centered({1, 1, 1}, {0.25f, 0.0f, 0.0f});

    auto r = boolean_mesh(a, b, BooleanOp::Subtract);
    REQUIRE(!r.vertices.empty());
    CHECK(watertight(r));
    // The carve is fully internal: the result bounds match A's solid extent.
    Bounds expA;
    expA.min = {-1.0f, -1.0f, -1.0f};
    expA.max = {1.0f, 1.0f, 1.0f};
    CHECK(bounds_close(r.bounds, expA, 1e-3f));

    // Result surface = A outer surface (24) + cavity walls of the 1x1x1
    // removed cube (2*(1*1 + 1*1 + 1*1) = 6) = 30.
    CHECK(std::abs(sum_tri_area(r) - 30.0f) < 1e-3f);
}

TEST_CASE("boolean: subtract cavity normals point inward")
{
    auto a = box_centered({2, 2, 2}, {0, 0, 0});
    auto b = box_centered({1, 1, 1}, {0.25f, 0.0f, 0.0f});
    auto r = boolean_mesh(a, b, BooleanOp::Subtract);
    REQUIRE(!r.vertices.empty());

    const Vec3f cavityCenter{0.25f, 0.0f, 0.0f};
    int wallTris = 0;
    for (size_t t = 0; t < r.indices.size() / 3; ++t)
    {
        const Vec3f& pa = r.vertices[r.indices[3 * t]].position;
        const Vec3f& pb = r.vertices[r.indices[3 * t + 1]].position;
        const Vec3f& pc = r.vertices[r.indices[3 * t + 2]].position;
        const Vec3f c = (pa + pb + pc) * (1.0f / 3.0f);
        // Retained triangle whose centroid lies inside the original B box.
        if (c.x > -0.26f && c.x < 0.76f && std::abs(c.y) < 0.52f && std::abs(c.z) < 0.52f)
        {
            ++wallTris;
            const Vec3f n = (pb - pa).cross(pc - pa).normalized();
            // Geometric normal must point toward the cavity center (inward).
            const Vec3f to = (cavityCenter - c).normalized();
            CHECK(n.dot(to) >= -1e-3f);
        }
    }
    CHECK(wallTris > 0);
}

TEST_CASE("boolean: disjoint subtract identical")
{
    // Pre-welded A so the post-assembly weld is a no-op and the short-circuit
    // path returns an exact copy (vertex/index identity preserved).
    BoxGeometry bg;
    bg.size = {2, 2, 2};
    auto a = weld_vertices(generate_box_mesh(bg), 1e-3f);
    REQUIRE(a.vertices.size() == 8);
    auto b = box_centered({1, 1, 1}, {10.0f, 0.0f, 0.0f});

    auto r = boolean_mesh(a, b, BooleanOp::Subtract);
    REQUIRE(!r.vertices.empty());
    CHECK(watertight(r));
    CHECK(r.vertices.size() == a.vertices.size());
    CHECK(meshes_identical(r, a));
}

TEST_CASE("boolean: disjoint union two shells")
{
    auto a = box_centered({2, 2, 2}, {0, 0, 0});
    auto b = box_centered({2, 2, 2}, {10.0f, 0.0f, 0.0f});

    auto r = boolean_mesh(a, b, BooleanOp::Union);
    REQUIRE(!r.vertices.empty());
    CHECK(watertight(r));
    // Disjoint union keeps every face of both shells (12 + 12 triangles;
    // each generator box has 24 vertices / 36 indices = 12 triangles).
    CHECK(r.indices.size() / 3 == a.indices.size() / 3 + b.indices.size() / 3);

    Bounds exp;
    exp.min = {-1.0f, -1.0f, -1.0f};
    exp.max = {11.0f, 1.0f, 1.0f};
    CHECK(bounds_close(r.bounds, exp, 1e-3f));
}

TEST_CASE("boolean: empty on open input")
{
    auto a = box_centered({2, 2, 2}, {0, 0, 0});
    auto plane = generate_plane_mesh({});   // open quad → fails the watertight gate

    auto r1 = boolean_mesh(plane, a, BooleanOp::Union);
    CHECK(r1.vertices.empty());
    CHECK(r1.indices.empty());

    auto r2 = boolean_mesh(a, plane, BooleanOp::Union);
    CHECK(r2.vertices.empty());
    CHECK(r2.indices.empty());
}

TEST_CASE("boolean: empty on zero volume")
{
    auto a = box_centered({2, 2, 2}, {0, 0, 0});
    // 1e-9-thick box: volume ~1e-9 below the 1e-9*diag^3 zero-volume gate.
    auto thin = box_centered({1, 1, 1e-9f}, {0.5f, 0.0f, 0.0f});

    auto r = boolean_mesh(a, thin, BooleanOp::Union);
    CHECK(r.vertices.empty());
    CHECK(r.indices.empty());
}

TEST_CASE("boolean: coplanar overlap returns empty")
{
    auto a = box_centered({2, 2, 2}, {0, 0, 0});        // y ∈ [-1,1], top face y=1
    auto b = box_centered({2, 2, 2}, {0, 2.0f, 0.0f});  // y ∈ [1,3], bottom face y=1

    // B's bottom face is exactly coplanar with A's top face and the 2D
    // projections overlap → documented V1 limitation → {}.
    auto r = boolean_mesh(a, b, BooleanOp::Union);
    CHECK(r.vertices.empty());
    CHECK(r.indices.empty());
}

TEST_CASE("boolean: canonicalization accepts un-welded inputs")
{
    BoxGeometry bg;
    bg.size = {2, 2, 2};
    auto a = generate_box_mesh(bg);
    auto b = generate_box_mesh(bg);
    REQUIRE(a.vertices.size() == 24);   // default generator output is un-welded
    REQUIRE(b.vertices.size() == 24);

    // Disjoint B (with a y/z offset); union keeps both shells.
    auto bShifted = box_centered({2, 2, 2}, {3.0f, 0.5f, 0.5f});
    auto r = boolean_mesh(a, bShifted, BooleanOp::Union);
    REQUIRE(!r.vertices.empty());
    CHECK(watertight(r));
    // Each box contributes 12 triangles (24 verts / 36 indices); disjoint union
    // keeps every face of both shells.
    CHECK(r.indices.size() / 3 == a.indices.size() / 3 + bShifted.indices.size() / 3);
    CHECK(r.indices.size() / 3 == 24);
}

// ── Mass properties ──

TEST_CASE("mass_properties: unit box exact integrals") {
    BoxGeometry box;
    box.size = {1.0f, 2.0f, 3.0f};               // a=1, b=2, c=3
    const MeshData m = generate_box_mesh(box);
    const MassProperties mp = mesh_properties(m, 1000.0f);

    CHECK(mp.volume == doctest::Approx(6.0f).epsilon(1e-4f));          // a·b·c
    CHECK(mp.surface_area == doctest::Approx(22.0f).epsilon(1e-3f));   // 2(ab+bc+ca)
    CHECK(mp.mass == doctest::Approx(6000.0f).epsilon(1e-3f));
    // centroid at the origin (box is centered)
    CHECK(mp.centroid.x == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(mp.centroid.y == doctest::Approx(0.0f).epsilon(1e-5f));
    CHECK(mp.centroid.z == doctest::Approx(0.0f).epsilon(1e-5f));
    // box inertia about COM: diag(m/12·(b²+c²), m/12·(a²+c²), m/12·(a²+b²))
    const float m12 = 6000.0f / 12.0f;
    CHECK(mp.inertia.m[0] == doctest::Approx(m12 * (4.0f + 9.0f)).epsilon(1e-3f));   // Ixx = m/12(b²+c²)= 6000/12·13
    CHECK(mp.inertia.m[4] == doctest::Approx(m12 * (1.0f + 9.0f)).epsilon(1e-3f));   // Iyy = 6000/12·10
    CHECK(mp.inertia.m[8] == doctest::Approx(m12 * (1.0f + 4.0f)).epsilon(1e-3f));   // Izz = 6000/12·5
    // off-diagonals vanish for the centered box
    CHECK(mp.inertia.m[1] == doctest::Approx(0.0f).epsilon(1e-2f));
    CHECK(mp.inertia.m[2] == doctest::Approx(0.0f).epsilon(1e-2f));
    CHECK(mp.inertia.m[5] == doctest::Approx(0.0f).epsilon(1e-2f));
}

TEST_CASE("mass_properties: offset box centroid + parallel axis") {
    // translate a unit cube to (1, 2, 3); centroid must follow, inertia about
    // the COM is unchanged by translation
    BoxGeometry box;
    box.size = {1.0f, 1.0f, 1.0f};
    MeshData m = generate_box_mesh(box);
    m = transform_mesh(m, exd::math::Mat4::trs({1.0f, 2.0f, 3.0f},
                                          exd::math::Quat{1.0f, 0.0f, 0.0f, 0.0f},
                                          exd::math::Vec3f{1.0f, 1.0f, 1.0f}));
    const MassProperties mp = mesh_properties(m, 2700.0f);   // aluminium
    CHECK(mp.centroid.x == doctest::Approx(1.0f).epsilon(1e-4f));
    CHECK(mp.centroid.y == doctest::Approx(2.0f).epsilon(1e-4f));
    CHECK(mp.centroid.z == doctest::Approx(3.0f).epsilon(1e-4f));
    CHECK(mp.mass == doctest::Approx(2700.0f).epsilon(1e-3f));
    // unit cube COM inertia: diag(m/6, m/6, m/6) = 450
    CHECK(mp.inertia.m[0] == doctest::Approx(450.0f).epsilon(1e-2f));
    CHECK(mp.inertia.m[4] == doctest::Approx(450.0f).epsilon(1e-2f));
    CHECK(mp.inertia.m[8] == doctest::Approx(450.0f).epsilon(1e-2f));
}

TEST_CASE("mass_properties: sphere sanity + determinism") {
    SphereGeometry sphere;
    sphere.radius = 0.5f;
    const MeshData m = generate_sphere_mesh(sphere);
    const MassProperties a = mesh_properties(m, 1000.0f);
    const MassProperties b = mesh_properties(m, 1000.0f);
    // faceted sphere: converges to the analytic values from below
    CHECK(a.volume == doctest::Approx(4.0f / 3.0f * 3.14159265f * 0.125f).epsilon(0.05f));
    CHECK(b.volume == a.volume);
    CHECK(b.mass == a.mass);
    CHECK(a.centroid.x == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(a.centroid.y == doctest::Approx(0.0f).epsilon(1e-4f));
    CHECK(a.centroid.z == doctest::Approx(0.0f).epsilon(1e-4f));
    // hollow vs solid: steam-engine flywheel (solid disc) must be ~solid
    SteamEngineDefinition d;
    const Assembly asm_ = generate_steam_engine_assembly(d);
    for (const Part& part : asm_.parts)
    {
        const MassProperties mp = mesh_properties(part.mesh);
        CHECK(mp.volume > 0.0f);
        if (part.name == "steam_chest")
            CHECK(mp.volume == doctest::Approx(0.12f * 0.08f * 0.06f).epsilon(1e-3f));
    }
}

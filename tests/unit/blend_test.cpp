#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>

using namespace exd::geometry;

// ── Single primitive ────────────────────────────────────────────────────────

TEST_CASE("blend: single sphere produces mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .radius = 0.5f
    });
    geom.cellSize = 0.1f;
    geom.blendRadius = 0.0f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(mesh.indices.size() % 3 == 0); // must be triangles

    // Bounds should roughly contain a sphere of radius 0.5
    CHECK(mesh.bounds.min.x <= doctest::Approx(-0.4f).epsilon(0.15f));
    CHECK(mesh.bounds.max.x >= doctest::Approx(0.4f).epsilon(0.15f));
    CHECK(mesh.bounds.min.y <= doctest::Approx(-0.4f).epsilon(0.15f));
    CHECK(mesh.bounds.max.y >= doctest::Approx(0.4f).epsilon(0.15f));
}

TEST_CASE("blend: single box produces mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Box,
        .halfExtent = 0.5f
    });
    geom.cellSize = 0.1f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("blend: single capsule produces mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Capsule,
        .radius = 0.3f,
        .height = 1.0f
    });
    geom.cellSize = 0.1f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("blend: single cylinder produces mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Cylinder,
        .radius = 0.5f,
        .height = 1.0f
    });
    geom.cellSize = 0.1f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("blend: single cone produces mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Cone,
        .radius = 0.5f,
        .height = 1.0f
    });
    geom.cellSize = 0.1f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("blend: single torus produces mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Torus,
        .radius = 0.3f,     // minor
        .radius2 = 1.0f     // major
    });
    geom.cellSize = 0.15f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

// ── Two-primitive blending ──────────────────────────────────────────────────

TEST_CASE("blend: two spheres with blending")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {-0.3f, 0.0f, 0.0f},
        .radius = 0.5f,
        .id = 1
    });
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {0.3f, 0.0f, 0.0f},
        .radius = 0.5f,
        .id = 2
    });
    geom.blendRadius = 0.15f;
    geom.cellSize = 0.08f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    // The blended shape should span roughly [-0.8, 0.8] in X
    CHECK(mesh.bounds.min.x <= doctest::Approx(-0.7f).epsilon(0.2f));
    CHECK(mesh.bounds.max.x >= doctest::Approx(0.7f).epsilon(0.2f));
}

TEST_CASE("blend: sphere and box with blending")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {0.0f, 0.5f, 0.0f},
        .radius = 0.4f
    });
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Box,
        .position = {0.0f, -0.3f, 0.0f},
        .halfExtent = 0.4f
    });
    geom.blendRadius = 0.1f;
    geom.cellSize = 0.08f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

// ── Normals ─────────────────────────────────────────────────────────────────

TEST_CASE("blend: generateNormals flag produces non-zero normals")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .radius = 0.5f
    });
    geom.generateNormals = true;
    geom.cellSize = 0.15f;

    auto mesh = generate_blend_mesh(geom);

    REQUIRE(!mesh.vertices.empty());

    // At least some normals should be non-zero
    bool hasNonZeroNormal = false;
    for (const auto& v : mesh.vertices)
    {
        if (v.normal.length() > 0.5f) { hasNonZeroNormal = true; break; }
    }
    CHECK(hasNonZeroNormal);
}

TEST_CASE("blend: generateNormals=false produces default normals")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .radius = 0.5f
    });
    geom.generateNormals = false;
    geom.cellSize = 0.15f;

    auto mesh = generate_blend_mesh(geom);

    REQUIRE(!mesh.vertices.empty());

    // With normals disabled, all normals should be (0, 1, 0)
    for (const auto& v : mesh.vertices)
    {
        CHECK(v.normal.x == doctest::Approx(0.0f));
        CHECK(v.normal.y == doctest::Approx(1.0f));
        CHECK(v.normal.z == doctest::Approx(0.0f));
    }
}

// ── Edge cases ──────────────────────────────────────────────────────────────

TEST_CASE("blend: empty primitives returns empty mesh")
{
    BlendGeometry geom;
    auto mesh = generate_blend_mesh(geom);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("blend: zero radius sphere returns empty mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .radius = 0.0f
    });
    auto mesh = generate_blend_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("blend: zero height capsule returns empty mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Capsule,
        .radius = 0.0f,
        .height = 0.0f
    });
    auto mesh = generate_blend_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("blend: three-primitive blend (sphere + capsule + box)")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {0.0f, 0.6f, 0.0f},
        .radius = 0.4f,
        .id = 1
    });
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Capsule,
        .position = {0.0f, 0.0f, 0.0f},
        .radius = 0.3f,
        .height = 0.8f,
        .id = 2
    });
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Box,
        .position = {0.0f, -0.6f, 0.0f},
        .halfExtent = 0.35f,
        .id = 3
    });
    geom.blendRadius = 0.12f;
    geom.cellSize = 0.08f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("blend: transform (position + scale) applied correctly")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {2.0f, 0.0f, 0.0f},
        .scale = {0.5f, 1.5f, 0.5f},
        .radius = 0.5f
    });
    geom.cellSize = 0.1f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());

    // The sphere center should be around x=2.0
    // With scale.y=1.5 and radius=0.5, the Y extent should be ~0.75
    float avgX = 0, avgY = 0, avgZ = 0;
    for (const auto& v : mesh.vertices)
    {
        avgX += v.position.x;
        avgY += v.position.y;
        avgZ += v.position.z;
    }
    avgX /= mesh.vertices.size();
    avgY /= mesh.vertices.size();
    avgZ /= mesh.vertices.size();

    CHECK(avgX == doctest::Approx(2.0f).epsilon(0.15f));
    CHECK(std::abs(avgY) < 0.2f);
    CHECK(std::abs(avgZ) < 0.2f);

    // Y spread should be larger than X/Z spread
    float ySpread = mesh.bounds.max.y - mesh.bounds.min.y;
    float xSpread = mesh.bounds.max.x - mesh.bounds.min.x;
    CHECK(ySpread > xSpread * 1.2f);
}

TEST_CASE("blend: zero blend radius produces sharp union")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {-0.35f, 0.0f, 0.0f},
        .radius = 0.5f
    });
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {0.35f, 0.0f, 0.0f},
        .radius = 0.5f
    });
    geom.blendRadius = 0.0f; // sharp union
    geom.cellSize = 0.08f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("blend: larger blend radius produces valid mesh")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {-0.4f, 0.0f, 0.0f},
        .radius = 0.5f
    });
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .position = {0.4f, 0.0f, 0.0f},
        .radius = 0.5f
    });
    geom.blendRadius = 0.3f; // very soft blend
    geom.cellSize = 0.08f;

    auto mesh = generate_blend_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());

    // With soft blending, the gap between spheres should be filled
    // so the X span should be continuous (no gap around x=0)
    float gapMin = 1e30f, gapMax = -1e30f;
    for (const auto& v : mesh.vertices)
    {
        if (std::abs(v.position.y) < 0.15f && std::abs(v.position.z) < 0.15f)
        {
            gapMin = std::min(gapMin, v.position.x);
            gapMax = std::max(gapMax, v.position.x);
        }
    }
    // Rough check: the blended shape near y=0,z=0 should span from ~-0.9 to ~0.9
    CHECK(gapMin <= doctest::Approx(-0.7f).epsilon(0.2f));
    CHECK(gapMax >= doctest::Approx(0.7f).epsilon(0.2f));
}

TEST_CASE("blend: mesh indices are valid (no out-of-bounds)")
{
    BlendGeometry geom;
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Sphere,
        .radius = 0.5f
    });
    geom.primitives.push_back({
        .kind = BlendPrimitiveKind::Box,
        .position = {0.8f, 0.0f, 0.0f},
        .scale = {0.5f, 1.0f, 1.0f},
        .halfExtent = 0.3f
    });
    geom.blendRadius = 0.1f;
    geom.cellSize = 0.1f;

    auto mesh = generate_blend_mesh(geom);

    REQUIRE(!mesh.vertices.empty());
    uint32_t nv = static_cast<uint32_t>(mesh.vertices.size());
    for (auto idx : mesh.indices)
        CHECK(idx < nv);
}

#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>

#include <cmath>

using namespace exd::geometry;

// ── Extrusion ───────────────────────────────────────────────────────────────

TEST_CASE("extrusion: simple triangle extrude")
{
    ExtrusionGeometry geom;
    geom.profile = {{-0.5f, -0.5f, 0}, {0.5f, -0.5f, 0}, {0, 0.5f, 0}};
    geom.depth = 1.0f;

    auto mesh = generate_extrusion_mesh(geom);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("extrusion: uncapped")
{
    ExtrusionGeometry geom;
    geom.profile = {{0, 0, 0}, {1, 0, 0}, {0.5f, 1, 0}};
    geom.depth = 0.5f;
    geom.capped = false;

    auto mesh = generate_extrusion_mesh(geom);
    CHECK(!mesh.vertices.empty());
}

TEST_CASE("extrusion: empty profile returns empty")
{
    ExtrusionGeometry geom;
    auto mesh = generate_extrusion_mesh(geom);
    CHECK(mesh.vertices.empty());
}

// ── Lathe ───────────────────────────────────────────────────────────────────

TEST_CASE("lathe: simple line profile produces cylinder-like shape")
{
    LatheGeometry geom;
    geom.profile = {{0.5f, -0.5f, 0}, {0.5f, 0.5f, 0}};
    geom.segments = 32;

    auto mesh = generate_lathe_mesh(geom);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    size_t expectedVerts = 2 * 33; // 2 profile points × (32+1) rings
    CHECK(mesh.vertices.size() == expectedVerts);
}

TEST_CASE("lathe: partial revolve (half)")
{
    LatheGeometry geom;
    geom.profile = {{0.5f, -0.3f, 0}, {0.5f, 0.3f, 0}};
    geom.segments = 16;
    geom.startAngle = 0;
    geom.endAngle = 3.14159265358979323846f; // 180°

    auto mesh = generate_lathe_mesh(geom);
    CHECK(!mesh.vertices.empty());
}

TEST_CASE("lathe: vase-like profile")
{
    LatheGeometry geom;
    geom.profile = {
        {0.1f, -1.0f, 0}, // base
        {0.3f, -0.8f, 0},
        {0.2f, -0.3f, 0}, // narrow neck
        {0.4f,  0.0f, 0}, // bulge
        {0.15f, 0.5f, 0}, // neck
        {0.25f, 0.8f, 0}, // lip
        {0.2f,  1.0f, 0}, // top
    };
    geom.segments = 48;

    auto mesh = generate_lathe_mesh(geom);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("lathe: empty profile returns empty")
{
    LatheGeometry geom;
    auto mesh = generate_lathe_mesh(geom);
    CHECK(mesh.vertices.empty());
}

// ── Helix ───────────────────────────────────────────────────────────────────

TEST_CASE("helix: triangular profile produces spring")
{
    HelixGeometry geom;
    geom.profile = {{-0.1f, -0.1f, 0}, {0.1f, -0.1f, 0}, {0, 0.1f, 0}};
    geom.radius = 1.0f;
    geom.height = 3.0f;
    geom.turns = 5.0f;
    geom.pathSteps = 64;

    auto mesh = generate_helix_mesh(geom);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    bool hasLow = false, hasHigh = false;
    for (const auto& v : mesh.vertices) {
        if (v.position.y < 0.1f) hasLow = true;
        if (v.position.y > 2.5f) hasHigh = true;
    }
    CHECK(hasLow);
    CHECK(hasHigh);
}

TEST_CASE("helix: empty profile returns empty")
{
    HelixGeometry geom;
    auto mesh = generate_helix_mesh(geom);
    CHECK(mesh.vertices.empty());
}

// ── Heightmap ───────────────────────────────────────────────────────────────

TEST_CASE("heightmap: simple terrain")
{
    Heightmap hm;
    hm.width = 4;
    hm.height = 3;
    hm.heightData = {
        0, 0.2f, 0.4f, 0.6f,
        0.1f, 0.3f, 0.5f, 0.7f,
        0.2f, 0.4f, 0.6f, 0.8f
    };
    hm.size = {2.0f, 1.0f, 1.5f};

    auto mesh = generate_heightmap_mesh(hm);
    CHECK(!mesh.vertices.empty());
    CHECK(mesh.vertices.size() == 12);
    CHECK(mesh.indices.size() == 36);
}

TEST_CASE("heightmap: too small returns empty")
{
    Heightmap hm;
    hm.width = 1;
    hm.height = 1;
    hm.heightData = {0.5f};
    auto mesh = generate_heightmap_mesh(hm);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("heightmap: data size mismatch returns empty")
{
    Heightmap hm;
    hm.width = 10;
    hm.height = 10;
    hm.heightData = {0};
    auto mesh = generate_heightmap_mesh(hm);
    CHECK(mesh.vertices.empty());
}

// ── Deformation ─────────────────────────────────────────────────────────────

TEST_CASE("deform: bend produces curved mesh")
{
    auto box = generate_box_mesh({.size = {0.2f, 2.0f, 0.2f}});
    DeformDescriptor d;
    d.bend = true;
    d.bendAngle = 1.57079632679f;
    d.bendRadius = 1.0f;
    d.bendAxis = {0, 0, 1};

    auto bent = deform_mesh(box, d);
    CHECK(!bent.vertices.empty());
    CHECK(bent.vertices.size() == box.vertices.size());

    bool displaced = false;
    for (size_t i = 0; i < box.vertices.size(); ++i) {
        if ((bent.vertices[i].position - box.vertices[i].position).length() > 0.01f)
            displaced = true;
    }
    CHECK(displaced);
}

TEST_CASE("deform: twist rotates vertices")
{
    auto box = generate_box_mesh({.size = {1.0f, 2.0f, 1.0f}});
    DeformDescriptor d;
    d.twist = true;
    d.twistAngle = 1.57079632679f;
    d.twistAxis = {0, 1, 0};

    auto twisted = deform_mesh(box, d);
    CHECK(!twisted.vertices.empty());

    float topMinX = 1e10f, topMaxZ = -1e10f;
    for (const auto& v : twisted.vertices) {
        if (v.position.y > 0.5f) {
            topMinX = std::min(topMinX, v.position.x);
            topMaxZ = std::max(topMaxZ, v.position.z);
        }
    }
    CHECK(topMinX < 0.0f);
    CHECK(topMaxZ > 0.0f);
}

TEST_CASE("deform: taper scales vertices")
{
    auto box = generate_box_mesh({.size = {1.0f, 2.0f, 1.0f}});
    DeformDescriptor d;
    d.taper = true;
    d.taperStart = 1.0f;
    d.taperEnd = 0.2f;

    auto tapered = deform_mesh(box, d);
    CHECK(!tapered.vertices.empty());

    float topSpread = 0, bottomSpread = 0;
    for (const auto& v : tapered.vertices) {
        float r = std::sqrt(v.position.x * v.position.x + v.position.z * v.position.z);
        if (v.position.y > 0.5f) topSpread = std::max(topSpread, r);
        else bottomSpread = std::max(bottomSpread, r);
    }
    CHECK(topSpread < bottomSpread);
}

TEST_CASE("deform: noise displaces vertices")
{
    auto plane = generate_plane_mesh({.size = {2, 0, 2}, .segmentsW = 4, .segmentsD = 4});
    DeformDescriptor d;
    d.noise = true;
    d.noiseAmplitude = 0.2f;
    d.noiseFrequency = 3.0f;
    d.noiseSeed = 42;

    auto noisy = deform_mesh(plane, d);
    CHECK(!noisy.vertices.empty());

    bool displaced = false;
    for (const auto& v : noisy.vertices) {
        if (std::abs(v.position.y) > 0.01f) displaced = true;
    }
    CHECK(displaced);
}

TEST_CASE("deform: chained deformations (bend + twist)")
{
    auto box = generate_box_mesh({.size = {0.3f, 2.0f, 0.3f}});
    DeformDescriptor d1;
    d1.bend = true;
    d1.bendAngle = 0.5f;
    auto bent = deform_mesh(box, d1);

    DeformDescriptor d2;
    d2.twist = true;
    d2.twistAngle = 0.3f;
    auto chained = deform_mesh(bent, d2);

    CHECK(!chained.vertices.empty());
    CHECK(chained.vertices.size() == box.vertices.size());
}

TEST_CASE("deform: identity returns identical mesh")
{
    auto box = generate_box_mesh({});
    DeformDescriptor d;
    auto result = deform_mesh(box, d);

    CHECK(result.vertices.size() == box.vertices.size());
    for (size_t i = 0; i < box.vertices.size(); ++i)
        CHECK(result.vertices[i].position.x == doctest::Approx(box.vertices[i].position.x));
}

// ── BlendOp: Subtract / Intersect ──────────────────────────────────────────

TEST_CASE("blend: subtract (box minus sphere) produces hollow")
{
    BlendGeometry geom;
    geom.primitives.push_back({.kind = BlendPrimitiveKind::Box, .halfExtent = 0.6f});
    geom.primitives.push_back({.kind = BlendPrimitiveKind::Sphere, .radius = 0.4f});
    geom.op = BlendOp::Subtract;
    geom.cellSize = 0.1f;

    auto mesh = generate_blend_mesh(geom);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("blend: intersect (box and sphere) produces lens shape")
{
    BlendGeometry geom;
    geom.primitives.push_back({.kind = BlendPrimitiveKind::Box, .halfExtent = 0.5f});
    geom.primitives.push_back({.kind = BlendPrimitiveKind::Sphere, .radius = 0.6f});
    geom.op = BlendOp::Intersect;
    geom.cellSize = 0.1f;

    auto mesh = generate_blend_mesh(geom);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

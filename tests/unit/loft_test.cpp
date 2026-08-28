#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>

#include <cmath>
#include <vector>

using namespace exd::geometry;
using exd::math::Vec3f;

TEST_CASE("loft: square to square")
{
    LoftGeometry g;
    g.sections = {
        {{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}},
        {{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}},
    };
    auto mesh = generate_loft_mesh(g);
    REQUIRE(mesh.vertices.size() == 16);   // 8 skin + 4 + 4 cap verts
    REQUIRE(mesh.indices.size() == 36);    // 8 + 2 + 2 triangles

    CHECK(mesh.bounds.min.x == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.y == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.z == doctest::Approx(0.0f));
    CHECK(mesh.bounds.max.z == doctest::Approx(1.0f));

    for (uint32_t i : mesh.indices)
        CHECK(i < 16);

    // Wall faces are the first 8 triangles: non-degenerate and pointing OUTWARD
    // from the spine (dot with the radial direction in XY is positive).
    for (size_t f = 0; f < 8; ++f)
    {
        const Vec3f a = mesh.vertices[mesh.indices[f * 3 + 0]].position;
        const Vec3f b = mesh.vertices[mesh.indices[f * 3 + 1]].position;
        const Vec3f c = mesh.vertices[mesh.indices[f * 3 + 2]].position;
        const Vec3f n = (b - a).cross(c - a);
        CHECK(n.length() > 0.5f);
        const Vec3f centroid = (a + b + c) / 3.0f;
        CHECK(n.dot(Vec3f{centroid.x, centroid.y, 0.0f}) > 0.0f);
    }

    // Cap normals: cap_start opposes the spine, cap_end follows it.
    for (uint32_t i = 8; i < 12; ++i)
        CHECK(mesh.vertices[i].normal.z < -0.99f);
    for (uint32_t i = 12; i < 16; ++i)
        CHECK(mesh.vertices[i].normal.z > 0.99f);
}

TEST_CASE("loft: circle to square")
{
    std::vector<Vec3f> circle;
    circle.reserve(16);
    for (uint32_t i = 0; i < 16; ++i)
    {
        const float a = 2.0f * std::acos(-1.0f) * static_cast<float>(i) / 16.0f;
        circle.push_back({0.5f * std::cos(a), 0.5f * std::sin(a), 0.0f});
    }
    const auto square = resample_ring({{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}}, 16);
    REQUIRE(square.size() == 16);

    LoftGeometry g;
    g.sections = {circle, square};
    auto mesh = generate_loft_mesh(g);
    REQUIRE(!mesh.vertices.empty());
    REQUIRE(!mesh.indices.empty());

    // Skin faces occupy the first 2 * (ns-1) * n = 32 triangles; all must be
    // non-degenerate. (Observed minimum |face normal| is ~0.28 for this
    // circle→square blend; 0.05 is a firm non-degeneracy bound.)
    for (size_t f = 0; f < 32; ++f)
    {
        const Vec3f a = mesh.vertices[mesh.indices[f * 3 + 0]].position;
        const Vec3f b = mesh.vertices[mesh.indices[f * 3 + 1]].position;
        const Vec3f c = mesh.vertices[mesh.indices[f * 3 + 2]].position;
        CHECK((b - a).cross(c - a).length() > 0.05f);
    }
}

TEST_CASE("loft: caps concave")
{
    const std::vector<Vec3f> L = {{0,0,0},{2,0,0},{2,1,0},{1,1,0},{1,2,0},{0,2,0}};
    std::vector<Vec3f> L2 = L;
    for (auto& p : L2)
        p.z = 3.0f;

    LoftGeometry g;
    g.sections = {L, L2};
    auto mesh = generate_loft_mesh(g);
    REQUIRE(!mesh.vertices.empty());

    // Wall = 12 faces; cap_start = 4 faces at z == 0; cap_end = 4 faces at z == 3.
    REQUIRE(mesh.indices.size() / 3 == 20);
    for (size_t f = 12; f < 16; ++f)
    {
        for (uint32_t k = 0; k < 3; ++k)
            CHECK(std::abs(mesh.vertices[mesh.indices[f * 3 + k]].position.z) < 1e-4f);
    }
}

TEST_CASE("loft: mismatched counts empty")
{
    LoftGeometry g;
    g.sections = {
        {{0,0,0},{1,0,0},{0,1,0}},
        {{0,0,0},{1,0,0},{0,1,0},{1,1,0}},
    };
    auto mesh = generate_loft_mesh(g);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("loft: too few sections empty")
{
    LoftGeometry g;
    g.sections = {{{0,0,0},{1,0,0},{0,1,0}}};
    auto mesh = generate_loft_mesh(g);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("loft: coincident centroids empty")
{
    LoftGeometry g;
    g.sections = {
        {{0,0,0},{1,0,0},{0,1,0}},
        {{0,0,0},{1,0,0},{0,1,0}},
    };
    auto mesh = generate_loft_mesh(g);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("loft part: patches")
{
    LoftGeometry g;
    g.sections = {
        {{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}},
        {{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}},
    };
    auto part = generate_loft_part(g);
    REQUIRE(part.mesh.vertices.size() == 16);
    REQUIRE(part.mesh.indices.size() == 36);
    REQUIRE(part.patches.size() == 3);

    CHECK(part.patches[0].name == "wall");
    CHECK(part.patches[0].faces.size() == 8);
    CHECK(part.patches[1].name == "cap_start");
    CHECK(part.patches[1].faces.size() == 2);
    CHECK(part.patches[2].name == "cap_end");
    CHECK(part.patches[2].faces.size() == 2);

    // cap_start faces reference only the z == 0 cap ring vertices.
    for (uint32_t f : part.patches[1].faces)
    {
        for (uint32_t k = 0; k < 3; ++k)
            CHECK(std::abs(part.mesh.vertices[part.mesh.indices[f * 3 + k]].position.z) < 1e-4f);
    }

    // Patches cover every face.
    uint32_t total = 0;
    for (const auto& p : part.patches)
        total += static_cast<uint32_t>(p.faces.size());
    CHECK(total == part.mesh.indices.size() / 3);
}

TEST_CASE("resample: equal point counts")
{
    const std::vector<Vec3f> sq = {{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}};
    auto r = resample_ring(sq, 8);
    REQUIRE(r.size() == 8);
    CHECK(r[0] == sq[0]);

    float perim = 0.0f;
    for (size_t i = 0; i < r.size(); ++i)
        perim += (r[(i + 1) % r.size()] - r[i]).length();
    CHECK(std::abs(perim - 8.0f) < 1e-4f);
}

TEST_CASE("resample: degenerate")
{
    CHECK(resample_ring({{0,0,0},{1,0,0}}, 4).empty());              // < 3 source points
    CHECK(resample_ring({{0,0,0},{1,0,0},{1,1,0}}, 2).empty());      // count < 3
    CHECK(resample_ring({{1,1,0},{1,1,0},{1,1,0}}, 6).empty());      // all coincident
}
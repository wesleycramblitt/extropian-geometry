#include <doctest/doctest.h>

#include <exd/geometry/terrain.hpp>

#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>

using namespace exd::geometry;

namespace {

float max_height(const Heightmap& hm)
{
    float m = 0.0f;
    for (const float h : hm.heightData) m = std::max(m, h);
    return m;
}

float avg_height(const Heightmap& hm)
{
    float s = 0.0f;
    for (const float h : hm.heightData) s += h;
    return s / static_cast<float>(hm.heightData.size());
}

} // namespace

TEST_CASE("terrain: every kind produces a valid, deterministic field") {
    for (int k = 0; k < static_cast<int>(TerrainKind::Terraced) + 1; ++k) {
        const auto kind = static_cast<TerrainKind>(k);
        CAPTURE(k);

        TerrainConfig cfg;
        cfg.kind = kind;
        cfg.width = 64;
        cfg.height = 48;

        const Heightmap a = generate_terrain_heightmap(cfg);
        REQUIRE(a.heightData.size() == 64u * 48u);
        for (const float h : a.heightData) {
            CHECK(h >= 0.0f);
            CHECK(h <= 1.0f);
        }

        // Determinism: same seed -> bit-identical field.
        const Heightmap b = generate_terrain_heightmap(cfg);
        for (size_t i = 0; i < a.heightData.size(); ++i)
            CHECK(a.heightData[i] == doctest::Approx(b.heightData[i]).epsilon(1e-7f));

        // Different seed -> different field.
        cfg.seed = 99u;
        const Heightmap c = generate_terrain_heightmap(cfg);
        float diff = 0.0f;
        for (size_t i = 0; i < a.heightData.size(); ++i)
            diff = std::max(diff, std::abs(a.heightData[i] - c.heightData[i]));
        CHECK(diff > 0.01f);
    }
}

TEST_CASE("terrain: mountains are taller than plains") {
    TerrainConfig cfg;
    cfg.width = 96;
    cfg.height = 96;

    cfg.kind = TerrainKind::Plains;
    const float plains_max = max_height(generate_terrain_heightmap(cfg));

    cfg.kind = TerrainKind::Mountains;
    const float mtn_max = max_height(generate_terrain_heightmap(cfg));

    CHECK(mtn_max > plains_max + 0.15f);
    CHECK(plains_max < 0.15f);          // plains stay low
    CHECK(mtn_max > 0.45f);             // mountains actually rise
}

TEST_CASE("terrain: terraced heights are quantized") {
    TerrainConfig cfg;
    cfg.kind = TerrainKind::Terraced;
    cfg.terraces = 8;
    cfg.amplitude = 1.0f;   // quantize the full range: k/8 is exact in float
    cfg.width = 64;
    cfg.height = 64;

    const Heightmap hm = generate_terrain_heightmap(cfg);
    for (const float h : hm.heightData) {
        const float q = std::round(h * 8.0f) / 8.0f;
        CHECK(std::abs(h - q) < 1e-4f);
    }
}

TEST_CASE("terrain: islands sink below water at the corners") {
    TerrainConfig cfg;
    cfg.kind = TerrainKind::Islands;
    cfg.width = 64;
    cfg.height = 64;

    const Heightmap hm = generate_terrain_heightmap(cfg);
    auto at = [&](uint32_t x, uint32_t z) { return hm.heightData[z * cfg.width + x]; };
    CHECK(at(0, 0) < 0.01f);
    CHECK(at(63, 0) < 0.01f);
    CHECK(at(0, 63) < 0.01f);
    CHECK(at(63, 63) < 0.01f);
    // And there is land somewhere in the middle.
    float mid_max = 0.0f;
    for (uint32_t z = 20; z < 44; ++z)
        for (uint32_t x = 20; x < 44; ++x)
            mid_max = std::max(mid_max, hm.heightData[z * cfg.width + x]);
    CHECK(mid_max > 0.3f);
}

TEST_CASE("terrain: canyon has both mesas and deep valleys") {
    TerrainConfig cfg;
    cfg.kind = TerrainKind::Canyon;
    cfg.width = 96;
    cfg.height = 96;
    cfg.seed = 5u;

    const Heightmap hm = generate_terrain_heightmap(cfg);
    int high = 0, low = 0;
    for (const float h : hm.heightData) {
        if (h > 0.55f) ++high;   // mesa tops
        if (h < 0.15f) ++low;    // canyon floors
    }
    const float n = static_cast<float>(hm.heightData.size());
    // Measured across seeds: ~19-24% mesa plateaus, ~30-43% valley floors.
    CHECK(static_cast<float>(high) > 0.15f * n);
    CHECK(static_cast<float>(low)  > 0.25f * n);
}

TEST_CASE("terrain: sea level floods low ground") {
    TerrainConfig cfg;
    cfg.kind = TerrainKind::RollingHills;
    cfg.width = 64;
    cfg.height = 64;
    cfg.sea_level = 0.4f;

    const Heightmap hm = generate_terrain_heightmap(cfg);
    for (const float h : hm.heightData) {
        CHECK(h >= 0.0f);
        CHECK(h <= 1.0f);
    }
    // Flooding must have flattened some cells that were above zero before.
    TerrainConfig dry = cfg;
    dry.sea_level = 0.0f;
    const Heightmap dm = generate_terrain_heightmap(dry);
    bool flattened = false;
    for (size_t i = 0; i < hm.heightData.size(); ++i)
        if (dm.heightData[i] > 0.0f && hm.heightData[i] == 0.0f) flattened = true;
    CHECK(flattened);
}

TEST_CASE("terrain: recipe overrides change the output") {
    TerrainConfig a, b;
    a.kind = b.kind = TerrainKind::RollingHills;
    a.octaves = 2;
    a.frequency = 1.5f;
    a.gain = 0.4f;
    a.amplitude = 0.5f;
    b.octaves = 6;
    b.frequency = 7.0f;
    b.gain = 0.6f;
    b.amplitude = 0.25f;
    a.width = b.width = 48;
    a.height = b.height = 48;

    const Heightmap ha = generate_terrain_heightmap(a);
    const Heightmap hb = generate_terrain_heightmap(b);
    float diff = 0.0f;
    for (size_t i = 0; i < ha.heightData.size(); ++i)
        diff = std::max(diff, std::abs(ha.heightData[i] - hb.heightData[i]));
    CHECK(diff > 0.1f);
}

TEST_CASE("terrain: mesh has world-size bounds, real normals, and colors") {
    TerrainConfig cfg;
    cfg.kind = TerrainKind::Mountains;
    cfg.width = 48;
    cfg.height = 48;
    cfg.seed = 3u;
    cfg.size = {50.0f, 8.0f, 50.0f};

    const MeshData mesh = generate_terrain_mesh(cfg);
    REQUIRE_FALSE(mesh.vertices.empty());
    CHECK(mesh.indices.size() % 3 == 0);
    CHECK(mesh.vertices.size() == 48u * 48u);

    const Bounds b = compute_bounds(mesh.vertices);
    CHECK(b.min.x == doctest::Approx(-25.0f).epsilon(1e-3f));
    CHECK(b.max.x == doctest::Approx(25.0f).epsilon(1e-3f));
    CHECK(b.min.z == doctest::Approx(-25.0f).epsilon(1e-3f));
    CHECK(b.max.z == doctest::Approx(25.0f).epsilon(1e-3f));
    CHECK(b.max.y <= 8.0f + 1e-3f);

    // Normals are computed from the surface, not flat up.
    size_t sloped = 0;
    for (const auto& v : mesh.vertices)
        if (std::abs(v.normal.x) > 0.05f || std::abs(v.normal.z) > 0.05f) ++sloped;
    CHECK(sloped > mesh.vertices.size() / 2u);

    // Vertex colors vary with height when colorize is on.
    float cmin = 1e9f, cmax = -1e9f;
    for (const auto& v : mesh.vertices) {
        cmin = std::min(cmin, v.color.x);
        cmax = std::max(cmax, v.color.x);
    }
    CHECK(cmax - cmin > 0.1f);

    // Colorize off -> uniform default color.
    cfg.colorize = false;
    const MeshData plain = generate_terrain_mesh(cfg);
    const float first = plain.vertices.front().color.x;
    bool uniform = true;
    for (const auto& v : plain.vertices)
        if (v.color.x != first) uniform = false;
    CHECK(uniform);
}

TEST_CASE("terrain: degenerate config produces empty output") {
    TerrainConfig cfg;
    cfg.width = 1;
    cfg.height = 1;
    CHECK(generate_terrain_heightmap(cfg).heightData.empty());
    CHECK(generate_terrain_mesh(cfg).vertices.empty());
}

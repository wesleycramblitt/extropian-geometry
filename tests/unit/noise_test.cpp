#include <doctest/doctest.h>

#include <exd/geometry/noise.hpp>

#include <algorithm>
#include <cmath>

using namespace exd::geometry;

TEST_CASE("noise: same seed reproduces identical samples") {
    const Noise2D a(42u), b(42u);
    for (float y = -1.5f; y < 2.0f; y += 0.37f) {
        for (float x = -1.5f; x < 2.0f; x += 0.29f) {
            CHECK(a.value(x, y) == doctest::Approx(b.value(x, y)).epsilon(1e-6f));
            CHECK(a.perlin(x, y) == doctest::Approx(b.perlin(x, y)).epsilon(1e-6f));
            CHECK(a.fbm(x, y, 4) == doctest::Approx(b.fbm(x, y, 4)).epsilon(1e-6f));
            CHECK(a.ridged(x, y, 4) == doctest::Approx(b.ridged(x, y, 4)).epsilon(1e-6f));
            CHECK(a.voronoi_f1(x, y) == doctest::Approx(b.voronoi_f1(x, y)).epsilon(1e-6f));
        }
    }
}

TEST_CASE("noise: different seeds diverge") {
    const Noise2D a(1u), b(2u);
    float max_diff = 0.0f;
    for (int i = 0; i < 200; ++i) {
        const float x = static_cast<float>(i) * 0.13f;
        max_diff = std::max(max_diff, std::abs(a.perlin(x, 0.5f) - b.perlin(x, 0.5f)));
    }
    CHECK(max_diff > 0.1f);
}

TEST_CASE("noise: value and perlin stay in [-1, 1]") {
    const Noise2D n(7u);
    float vmax = 0.0f, pmax = 0.0f;
    for (int i = 0; i < 4096; ++i) {
        const float x = static_cast<float>((i * 37) % 977) * 0.037f;
        const float y = static_cast<float>((i * 91) % 911) * 0.043f;
        vmax = std::max(vmax, std::abs(n.value(x, y)));
        pmax = std::max(pmax, std::abs(n.perlin(x, y)));
    }
    CHECK(vmax <= 1.0f + 1e-6f);
    CHECK(pmax <= 1.05f);
}

TEST_CASE("noise: lattices are not constant or degenerate") {
    const Noise2D n(7u);
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < 256; ++i) {
        const float x = static_cast<float>(i) * 0.123f;
        const float y = static_cast<float>(i % 17) * 0.071f;
        lo = std::min(lo, n.perlin(x, y));
        hi = std::max(hi, n.perlin(x, y));
    }
    CHECK(hi - lo > 0.5f);   // real structure, not a flat field
}

TEST_CASE("noise: fbm with one octave equals the base lattice") {
    const Noise2D n(11u);
    for (float y = -0.7f; y < 0.8f; y += 0.23f)
        for (float x = -0.7f; x < 0.8f; x += 0.19f)
            CHECK(n.fbm(x, y, 1) == doctest::Approx(n.perlin(x, y)).epsilon(1e-5f));
}

TEST_CASE("noise: fbm and ridged stay in documented ranges") {
    const Noise2D n(13u);
    for (int i = 0; i < 512; ++i) {
        const float x = static_cast<float>(i) * 0.517f;
        const float y = static_cast<float>((i * 13) % 29) * 0.3f;
        CHECK(std::abs(n.fbm(x, y, 5)) <= 1.0f + 1e-6f);
        CHECK(n.ridged(x, y, 5) >= 0.0f);
        CHECK(n.ridged(x, y, 5) <= 1.0f + 1e-6f);
        CHECK(n.warped(x, y, 4) == doctest::Approx(n.warped(x, y, 4)).epsilon(1e-6f));
        CHECK(std::isfinite(n.warped(x, y, 4)));
    }
}

TEST_CASE("noise: voronoi distances are ordered and non-negative") {
    const Noise2D n(17u);
    for (int i = 0; i < 512; ++i) {
        const float x = static_cast<float>(i) * 0.371f;
        const float y = static_cast<float>(i % 13) * 0.213f;
        const float f1 = n.voronoi_f1(x, y);
        const float f2 = n.voronoi_f2(x, y);
        const float crack = n.voronoi_f2_minus_f1(x, y);
        CHECK(f1 >= 0.0f);
        CHECK(f2 >= f1 - 1e-6f);
        CHECK(crack >= -1e-6f);
        CHECK(crack <= 1.5f);
    }
}

TEST_CASE("noise: value and perlin are smooth (C2 lattice interpolation)") {
    const Noise2D n(23u);
    for (int i = 0; i < 256; ++i) {
        const float x = static_cast<float>(i) * 0.37f;
        const float y = static_cast<float>(i % 11) * 0.23f;
        const float eps = 0.002f;
        CHECK(std::abs(n.value(x + eps, y) - n.value(x, y)) < 0.01f);
        CHECK(std::abs(n.perlin(x, y + eps) - n.perlin(x, y)) < 0.02f);
    }
}

#include <doctest/doctest.h>

#include <exd/geometry/spline.hpp>

#include <cmath>

using namespace exd::geometry;

TEST_CASE("monotone spline interpolates its control points") {
    const MonotoneCubicSpline s({0.0f, 1.0f, 2.0f, 3.0f}, {0.0f, 0.5f, 0.5f, 1.0f});
    REQUIRE(s.valid());
    CHECK(s.evaluate(0.0f) == doctest::Approx(0.0f));
    CHECK(s.evaluate(1.0f) == doctest::Approx(0.5f));
    CHECK(s.evaluate(2.0f) == doctest::Approx(0.5f));
    CHECK(s.evaluate(3.0f) == doctest::Approx(1.0f));
}

TEST_CASE("monotone spline never overshoots monotone data") {
    // Strictly increasing data must yield a strictly increasing curve.
    const MonotoneCubicSpline s({0.0f, 1.0f, 2.0f, 3.0f, 4.0f},
                                {0.0f, 1.0f, 1.1f, 5.0f, 6.0f});
    float prev = -1e9f;
    for (int i = 0; i <= 40; ++i) {
        const float x = 4.0f * static_cast<float>(i) / 40.0f;
        const float y = s.evaluate(x);
        CHECK(y >= prev);
        prev = y;
    }
}

TEST_CASE("monotone spline is clamped at the ends") {
    const MonotoneCubicSpline s({0.0f, 1.0f}, {0.0f, 1.0f});
    CHECK(s.evaluate(-5.0f) == doctest::Approx(0.0f));
    CHECK(s.evaluate(9.0f) == doctest::Approx(1.0f));
}

TEST_CASE("monotone spline derivative is finite across the domain") {
    const MonotoneCubicSpline s({0.0f, 1.0f, 2.0f}, {0.0f, 0.5f, 1.5f});
    for (int i = 0; i <= 20; ++i) {
        const float x = 2.0f * static_cast<float>(i) / 20.0f;
        const float d = s.derivative(x);
        CHECK(std::isfinite(d));
    }
}

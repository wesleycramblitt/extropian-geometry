#include <exd/geometry/noise.hpp>

#include <exd/core/random.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace exd::geometry
{

namespace
{

constexpr float kPi = 3.14159265358979323846f;

/// Quintic fade: 6t^5 - 15t^4 + 10t^3 (C2-continuous interpolation weight).
float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float lerp(float a, float b, float t) { return a + (b - a) * t; }

/// 16 unit gradient directions for Perlin (8 axes + 8 diagonals).
constexpr std::array<std::array<float, 2>, 16> kGradients{{
    { 1.0f,  0.0f}, {-1.0f,  0.0f}, { 0.0f,  1.0f}, { 0.0f, -1.0f},
    { 0.70710678f,  0.70710678f}, {-0.70710678f,  0.70710678f},
    { 0.70710678f, -0.70710678f}, {-0.70710678f, -0.70710678f},
    { 1.0f,  0.0f}, {-1.0f,  0.0f}, { 0.0f,  1.0f}, { 0.0f, -1.0f},
    { 0.70710678f,  0.70710678f}, {-0.70710678f,  0.70710678f},
    { 0.70710678f, -0.70710678f}, {-0.70710678f, -0.70710678f},
}};

} // namespace

Noise2D::Noise2D(uint32_t seed)
{
    std::array<uint8_t, 256> table;
    std::iota(table.begin(), table.end(), 0);

    // Fisher-Yates shuffle driven by the seeded PCG so the whole lattice is
    // a deterministic function of the seed.
    exd::core::RNG rng(seed);
    for (int i = 255; i > 0; --i) {
        const int j = rng.next_int(0, i);
        std::swap(table[static_cast<size_t>(i)], table[static_cast<size_t>(j)]);
    }
    for (size_t i = 0; i < 256; ++i) {
        perm_[i]         = table[i];
        perm_[i + 256u]  = table[i];
    }
}

uint8_t Noise2D::hash2(int32_t x, int32_t y) const
{
    return perm_[(perm_[static_cast<uint32_t>(x) & 255u] + y) & 255u];
}

float Noise2D::value(float x, float y) const
{
    const int32_t x0 = static_cast<int32_t>(std::floor(x));
    const int32_t y0 = static_cast<int32_t>(std::floor(y));
    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);

    const float a = hash2(x0,     y0)     / 255.0f;
    const float b = hash2(x0 + 1, y0)     / 255.0f;
    const float c = hash2(x0,     y0 + 1) / 255.0f;
    const float d = hash2(x0 + 1, y0 + 1) / 255.0f;

    const float u = fade(fx);
    const float v = fade(fy);
    return lerp(lerp(a, b, u), lerp(c, d, u), v) * 2.0f - 1.0f;
}

float Noise2D::perlin(float x, float y) const
{
    const int32_t x0 = static_cast<int32_t>(std::floor(x));
    const int32_t y0 = static_cast<int32_t>(std::floor(y));
    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);

    const auto& g00 = kGradients[hash2(x0,     y0)     & 15u];
    const auto& g10 = kGradients[hash2(x0 + 1, y0)     & 15u];
    const auto& g01 = kGradients[hash2(x0,     y0 + 1) & 15u];
    const auto& g11 = kGradients[hash2(x0 + 1, y0 + 1) & 15u];

    // Gradient contributions at the offsets within the cell. With unit
    // gradients and fade weights summing to one the blend stays within
    // approximately [-1, 1] (worst case ~1.02 near cell corners).
    const float n00 = g00[0] * fx        + g00[1] * fy;
    const float n10 = g10[0] * (fx - 1)  + g10[1] * fy;
    const float n01 = g01[0] * fx        + g01[1] * (fy - 1);
    const float n11 = g11[0] * (fx - 1)  + g11[1] * (fy - 1);

    const float u = fade(fx);
    const float v = fade(fy);
    return lerp(lerp(n00, n10, u), lerp(n01, n11, u), v);
}

float Noise2D::voronoi_f1(float x, float y) const
{
    const int32_t xi = static_cast<int32_t>(std::floor(x));
    const int32_t yi = static_cast<int32_t>(std::floor(y));
    const float fx = x - static_cast<float>(xi);
    const float fy = y - static_cast<float>(yi);

    float best = 1e9f;
    for (int32_t dy = -1; dy <= 1; ++dy) {
        for (int32_t dx = -1; dx <= 1; ++dx) {
            const float ox = hash2(xi + dx,     yi + dy)         / 255.0f;
            const float oy = hash2(xi + dx + 31, yi + dy + 197)  / 255.0f;
            const float ddx = fx - (static_cast<float>(dx) + ox);
            const float ddy = fy - (static_cast<float>(dy) + oy);
            best = std::min(best, ddx * ddx + ddy * ddy);
        }
    }
    return std::sqrt(best);
}

float Noise2D::voronoi_f2(float x, float y) const
{
    const int32_t xi = static_cast<int32_t>(std::floor(x));
    const int32_t yi = static_cast<int32_t>(std::floor(y));
    const float fx = x - static_cast<float>(xi);
    const float fy = y - static_cast<float>(yi);

    float f1 = 1e9f, f2 = 1e9f;
    for (int32_t dy = -1; dy <= 1; ++dy) {
        for (int32_t dx = -1; dx <= 1; ++dx) {
            const float ox = hash2(xi + dx,     yi + dy)        / 255.0f;
            const float oy = hash2(xi + dx + 31, yi + dy + 197) / 255.0f;
            const float ddx = fx - (static_cast<float>(dx) + ox);
            const float ddy = fy - (static_cast<float>(dy) + oy);
            const float d = ddx * ddx + ddy * ddy;
            if (d < f1) { f2 = f1; f1 = d; }
            else if (d < f2) { f2 = d; }
        }
    }
    return std::sqrt(f2);
}

float Noise2D::voronoi_f2_minus_f1(float x, float y) const
{
    const int32_t xi = static_cast<int32_t>(std::floor(x));
    const int32_t yi = static_cast<int32_t>(std::floor(y));
    const float fx = x - static_cast<float>(xi);
    const float fy = y - static_cast<float>(yi);

    float f1 = 1e9f, f2 = 1e9f;
    for (int32_t dy = -1; dy <= 1; ++dy) {
        for (int32_t dx = -1; dx <= 1; ++dx) {
            const float ox = hash2(xi + dx,     yi + dy)        / 255.0f;
            const float oy = hash2(xi + dx + 31, yi + dy + 197) / 255.0f;
            const float ddx = fx - (static_cast<float>(dx) + ox);
            const float ddy = fy - (static_cast<float>(dy) + oy);
            const float d = ddx * ddx + ddy * ddy;
            if (d < f1) { f2 = f1; f1 = d; }
            else if (d < f2) { f2 = d; }
        }
    }
    return std::sqrt(f2) - std::sqrt(f1);
}

float Noise2D::fbm(float x, float y, int octaves,
                   float lacunarity, float gain, Basis basis) const
{
    const int n = std::max(1, octaves);
    float sum = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float s = (basis == Basis::Value) ? value(x * freq, y * freq)
                                                : perlin(x * freq, y * freq);
        sum += amp * s;
        norm += amp;
        amp  *= gain;
        freq *= lacunarity;
    }
    return norm > 0.0f ? sum / norm : 0.0f;
}

float Noise2D::ridged(float x, float y, int octaves,
                      float lacunarity, float gain) const
{
    const int n = std::max(1, octaves);
    float sum = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float n_ = 1.0f - std::abs(perlin(x * freq, y * freq));
        const float ridge = n_ * n_;
        sum += amp * ridge;
        norm += amp;
        amp  *= gain;
        freq *= lacunarity;
    }
    return norm > 0.0f ? sum / norm : 0.0f;
}

float Noise2D::warped(float x, float y, int octaves,
                      float strength, float frequency,
                      float lacunarity, float gain) const
{
    // Warp field sampled at the requested frequency (two decorrelated
    // channels), then fBm sampled at the warped coordinates.
    const float qx = fbm(x * frequency, y * frequency, octaves, lacunarity, gain);
    const float qy = fbm(x * frequency + 31.4159f, y * frequency + 17.0123f,
                         octaves, lacunarity, gain);
    return fbm(x + strength * qx, y + strength * qy, octaves, lacunarity, gain);
}

} // namespace exd::geometry

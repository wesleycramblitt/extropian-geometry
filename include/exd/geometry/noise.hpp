#pragma once

#include <array>
#include <cstdint>

namespace exd::geometry
{

/// Seeded, deterministic 2D noise field.
///
/// One shuffled permutation table (seeded via exd::core::RNG / PCG32) backs
/// every lattice below: plain value noise, Perlin gradient noise, Voronoi
/// (Worley) cell distances, and the fractal sums built on top (fBm, ridged
/// multifractal, domain-warped fBm). The same seed produces identical
/// samples everywhere and on every platform; nothing here is stateful after
/// construction, so instances can be shared across threads.
///
/// Coordinate convention: noise lattices tile the real plane with unit
/// cells — callers scale coordinates to control feature size (see
/// TerrainConfig::frequency).
class Noise2D
{
public:
    explicit Noise2D(uint32_t seed = 1337u);

    /// Plain value noise (interpolated lattice hash), range [-1, 1].
    float value(float x, float y) const;

    /// Perlin gradient noise (unit gradients + quintic fade).
    /// Range approx [-1, 1] (worst-case blend bound ~1.02).
    float perlin(float x, float y) const;

    /// Worley: distance to the nearest feature point, range [0, ~1.5].
    /// (the two corner-distance bounds: f1 near 0 at features, up to ~1.4 worst case)
    float voronoi_f1(float x, float y) const;

    /// Worley: distance to the second-nearest feature point, range [0, ~1.5].
    float voronoi_f2(float x, float y) const;

    /// "Crackle" = f2 - f1: 0 at feature points, peaks on cell borders,
    /// range [0, ~1.4]. Classic canyon / badlands source.
    float voronoi_f2_minus_f1(float x, float y) const;

    enum class Basis { Value, Perlin };

    /// Fractal Brownian motion: octave sum of a base lattice, range [-1, 1].
    /// `lacunarity` multiplies frequency and `gain` multiplies amplitude per
    /// octave; results are normalized by the amplitude series.
    float fbm(float x, float y, int octaves,
              float lacunarity = 2.0f, float gain = 0.5f,
              Basis basis = Basis::Perlin) const;

    /// Ridged multifractal: |noise| -> invert -> square per octave, range
    /// [0, 1]. Produces sharp mountain spines.
    float ridged(float x, float y, int octaves,
                 float lacunarity = 2.0f, float gain = 0.5f) const;

    /// fBm sampled at noise-warped coordinates: eroded, organic shapes and
    /// realistic coastlines. `strength` is the warp amplitude in noise-space
    /// units, `frequency` the warp field's base frequency. Approx range [-1, 1].
    float warped(float x, float y, int octaves,
                 float strength = 1.0f, float frequency = 2.0f,
                 float lacunarity = 2.0f, float gain = 0.5f) const;

private:
    /// Table lookup hash of a lattice cell -> uint8_t in [0, 255].
    uint8_t hash2(int32_t x, int32_t y) const;

    std::array<uint8_t, 512> perm_;   // 256-entry seed-shuffled table, doubled
};

} // namespace exd::geometry

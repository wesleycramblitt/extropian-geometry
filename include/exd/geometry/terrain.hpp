#pragma once

#include <exd/geometry/heightmap.hpp>
#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

#include <cstdint>

namespace exd::geometry
{

/// Parametric terrain families. Each is a seeded recipe over the Noise2D
/// primitives (fBm, ridged multifractal, Worley crackle, domain warp) with
/// sensible defaults; every knob in TerrainConfig can override the preset.
enum class TerrainKind {
    Plains,        // gentle low-frequency undulation (value-noise fBm)
    RollingHills,  // classic Perlin fBm hills
    Mountains,     // ridged multifractal spines, sharp peaks
    Dunes,         // wind-aligned sand ridges with ripple detail
    Canyon,        // mesa plateaus split by deep crackle valleys
    Volcanic,      // crater fields on ridged ground
    Islands,       // radial-falloff fBm dropping below sea level
    Terraced,      // quantized hill benches (rice-terrace look)
};

/// Descriptor for procedural terrain generation. Zero / default field values
/// fall back to the recipe's preset defaults; world-space conventions match
/// Heightmap (size.y scales heights, terrain is centered on the origin).
struct TerrainConfig
{
    TerrainKind kind = TerrainKind::RollingHills;
    uint32_t seed = 1337u;              // reproducibility
    uint32_t width  = 128;              // grid columns
    uint32_t height = 128;              // grid rows
    math::Vec3f size = {100.0f, 10.0f, 100.0f};   // world extent (y = height scale)

    // Recipe overrides; 0 values keep the preset defaults.
    int   octaves    = 0;               // fractal octaves
    float frequency  = 0.0f;            // base noise cells across the map
    float lacunarity = 0.0f;            // per-octave frequency multiplier
    float gain       = 0.0f;            // per-octave amplitude multiplier
    float amplitude  = 0.0f;            // max height as a fraction of size.y
    float sea_level  = 0.0f;            // heights below this (rel. 0..1) clamp to 0
    int   terraces   = 0;               // Terraced quantization bins (0 = 8)
    bool  colorize   = true;            // bake sand/grass/rock/snow gradient
};

/// Generate the height field only: heightData normalized to [0, 1] (heights
/// world-space when meshed through generate_heightmap_mesh, which scales by
/// size.y). Deterministic for a given config.
Heightmap generate_terrain_heightmap(const TerrainConfig& cfg);

/// Generate the full terrain mesh: height field, smooth per-vertex normals,
/// optional height-based vertex colors (sea-water tint below sea_level).
MeshData generate_terrain_mesh(const TerrainConfig& cfg);

} // namespace exd::geometry

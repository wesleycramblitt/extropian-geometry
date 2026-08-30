#include <exd/geometry/terrain.hpp>

#include <exd/geometry/noise.hpp>
#include <exd/math/quat.hpp>

#include <algorithm>
#include <cmath>

namespace exd::geometry
{

namespace
{

constexpr float kTwoPi = 2.0f * 3.14159265358979323846f;

// ── Preset recipes ─────────────────────────────────────────────────────────

struct Recipe
{
    int   octaves    = 5;
    float frequency  = 4.0f;
    float lacunarity = 2.0f;
    float gain       = 0.5f;
    float amplitude  = 0.35f;          // fraction of size.y
    Noise2D::Basis basis = Noise2D::Basis::Perlin;
};

Recipe preset_recipe(TerrainKind kind)
{
    switch (kind) {
    case TerrainKind::Plains:   return {3, 2.0f, 2.0f, 0.5f, 0.08f, Noise2D::Basis::Value};
    case TerrainKind::RollingHills: return {5, 4.0f, 2.0f, 0.5f, 0.35f, Noise2D::Basis::Perlin};
    case TerrainKind::Mountains: return {6, 5.0f, 2.3f, 0.55f, 0.60f, Noise2D::Basis::Perlin};
    case TerrainKind::Dunes:    return {4, 3.0f, 2.2f, 0.55f, 0.25f, Noise2D::Basis::Value};
    case TerrainKind::Canyon:   return {1, 2.8f, 2.0f, 0.5f, 0.75f, Noise2D::Basis::Perlin};
    case TerrainKind::Volcanic: return {4, 4.0f, 2.2f, 0.5f, 0.65f, Noise2D::Basis::Perlin};
    case TerrainKind::Islands:  return {5, 3.0f, 2.1f, 0.55f, 0.70f, Noise2D::Basis::Perlin};
    case TerrainKind::Terraced: return {5, 4.0f, 2.0f, 0.5f, 0.45f, Noise2D::Basis::Perlin};
    }
    return {};
}

Recipe resolve(const TerrainConfig& cfg)
{
    Recipe r = preset_recipe(cfg.kind);
    if (cfg.octaves    > 0) r.octaves    = cfg.octaves;
    if (cfg.frequency  > 0) r.frequency  = cfg.frequency;
    if (cfg.lacunarity > 0) r.lacunarity = cfg.lacunarity;
    if (cfg.gain       > 0) r.gain       = cfg.gain;
    if (cfg.amplitude  > 0) r.amplitude  = cfg.amplitude;
    return r;
}

float smoothstep(float e0, float e1, float x)
{
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float sample_terrain(const TerrainConfig& cfg, const Recipe& r, const Noise2D& noise,
                     float u, float v, float nx, float nz)
{
    float h = 0.0f;
    switch (cfg.kind) {
    case TerrainKind::Plains:
    case TerrainKind::RollingHills:
        h = 0.5f + 0.5f * noise.fbm(u, v, r.octaves, r.lacunarity, r.gain, r.basis);
        break;

    case TerrainKind::Mountains:
        h = noise.ridged(u, v, r.octaves, r.lacunarity, r.gain);
        break;

    case TerrainKind::Dunes: {
        // Wind-aligned sand: lattice stretched along v, then the height is
        // modulated by low-frequency rows perpendicular to the wind plus a
        // fine ripple layer.
        const float base   = 0.5f + 0.5f * noise.fbm(u, v * 1.8f, r.octaves, r.lacunarity, r.gain, r.basis);
        const float rows   = 0.62f + 0.38f * std::sin(kTwoPi * u * 0.35f * r.frequency);
        const float ripple = 1.0f + 0.16f * noise.value(u * 24.0f, v * 24.0f);
        h = base * rows * ripple;
        break;
    }

    case TerrainKind::Canyon: {
        // Crackle (f2 - f1) peaks on Voronoi cell borders -> invert so the
        // cell interiors stay high as mesa plateaus while the borders carve
        // deep valleys; a touch of ridged detail textures the walls.
        const float cr    = std::clamp(noise.voronoi_f2_minus_f1(u, v) * 1.9f, 0.0f, 1.0f);
        const float mesa  = (1.0f - cr) * (1.0f - cr);
        const float walls = 0.06f * noise.ridged(u * 9.0f, v * 9.0f, 2, 2.0f, 0.6f);
        h = std::clamp(mesa + walls, 0.0f, 1.0f);
        break;
    }

    case TerrainKind::Volcanic: {
        // Ridged ground plus a Worley crater field: each cell's feature
        // point becomes a pit with an upraised rim around it.
        const float base = 0.5f + 0.5f * noise.ridged(u, v, r.octaves, r.lacunarity, r.gain);
        const float d    = noise.voronoi_f1(u, v);
        const float pit  = std::exp(-((d - 0.06f) * (d - 0.06f)) / 0.028f);
        const float rim  = std::exp(-((d - 0.30f) * (d - 0.30f)) / 0.012f);
        h = std::clamp(base * 0.72f - 0.55f * pit + 0.62f * rim, 0.0f, 1.0f);
        break;
    }

    case TerrainKind::Islands: {
        // Domain-warped fBm shaped by a radial falloff (with a wobble so the
        // coast is not a circle), then dropped below sea level at the edges.
        const float e = 0.5f + 0.5f * noise.warped(u, v, r.octaves, 0.8f, r.frequency * 0.5f,
                                                   r.lacunarity, r.gain);
        const float radial = std::sqrt(nx * nx + nz * nz);
        const float wobble = 0.30f * noise.value(u * 2.3f, v * 2.3f);
        const float falloff = smoothstep(0.62f, 0.12f, radial + wobble);
        h = std::clamp(e * falloff - 0.22f, 0.0f, 1.0f) * 1.28f;   // flood + renormalize
        break;
    }

    case TerrainKind::Terraced: {
        const float e = 0.5f + 0.5f * noise.fbm(u, v, r.octaves, r.lacunarity, r.gain, r.basis);
        const int bins = std::max(2, cfg.terraces > 0 ? cfg.terraces : 8);
        h = std::round(e * static_cast<float>(bins)) / static_cast<float>(bins);
        break;
    }
    }

    h *= r.amplitude;
    if (cfg.sea_level > 0.0f && h < cfg.sea_level) h = 0.0f;   // flood plane
    return std::clamp(h, 0.0f, 1.0f);
}

// ── Height-based vertex colors (w=R, x=G, y=B, z=A) ────────────────────────

struct ColorStop { float t; float r, g, b; };

constexpr ColorStop kTerrainRamp[] = {
    {0.00f, 0.76f, 0.70f, 0.50f},   // sand
    {0.35f, 0.32f, 0.55f, 0.30f},   // grass
    {0.55f, 0.42f, 0.40f, 0.38f},   // rock
    {0.80f, 0.92f, 0.93f, 0.96f},   // snow
};

math::Quat height_color(float h, float sea)
{
    if (h <= sea) return {0.16f, 0.40f, 0.58f, 1.0f};   // water
    for (size_t i = 1; i < std::size(kTerrainRamp); ++i) {
        if (h <= kTerrainRamp[i].t) {
            const auto& a = kTerrainRamp[i - 1];
            const auto& b = kTerrainRamp[i];
            const float t = std::clamp((h - a.t) / (b.t - a.t), 0.0f, 1.0f);
            return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t,
                    a.b + (b.b - a.b) * t, 1.0f};
        }
    }
    const auto& top = kTerrainRamp[std::size(kTerrainRamp) - 1];
    return {top.r, top.g, top.b, 1.0f};
}

} // namespace

Heightmap generate_terrain_heightmap(const TerrainConfig& cfg)
{
    if (cfg.width < 2 || cfg.height < 2) return {};

    const Recipe r = resolve(cfg);
    const Noise2D noise(cfg.seed);

    Heightmap hm;
    hm.width  = cfg.width;
    hm.height = cfg.height;
    hm.size   = cfg.size;
    hm.heightData.resize(static_cast<size_t>(cfg.width) * cfg.height);

    const float inv_w = 1.0f / static_cast<float>(cfg.width - 1);
    const float inv_h = 1.0f / static_cast<float>(cfg.height - 1);
    for (uint32_t z = 0; z < cfg.height; ++z) {
        for (uint32_t x = 0; x < cfg.width; ++x) {
            const float fx = static_cast<float>(x) * inv_w;
            const float fz = static_cast<float>(z) * inv_h;
            hm.heightData[static_cast<size_t>(z) * cfg.width + x] =
                sample_terrain(cfg, r, noise,
                               r.frequency * fx, r.frequency * fz,
                               fx * 2.0f - 1.0f, fz * 2.0f - 1.0f);
        }
    }
    return hm;
}

MeshData generate_terrain_mesh(const TerrainConfig& cfg)
{
    const Heightmap hm = generate_terrain_heightmap(cfg);
    if (hm.heightData.empty()) return {};

    MeshData mesh = generate_heightmap_mesh(hm);
    if (!cfg.colorize || mesh.vertices.empty()) return mesh;

    for (Vertex& v : mesh.vertices) {
        const float h = v.position.y / std::max(cfg.size.y, 1e-6f);
        v.color = height_color(h, cfg.sea_level);
    }
    return mesh;
}

} // namespace exd::geometry

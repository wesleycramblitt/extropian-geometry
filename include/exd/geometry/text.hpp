#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace exd::geometry
{

// ── Forward declarations ──
class FontAtlas;

// ── Font / glyph identity ──

using FontId  = uint64_t;
using GlyphId = uint32_t;

// ── Enums ──

enum class FontWeight
{
    Thin    = 100,
    Light   = 300,
    Regular = 400,
    Medium  = 500,
    Bold    = 700,
    Black   = 900
};

enum class TextAlignment
{
    Left,
    Center,
    Right
};

// ── Text style ──

struct TextStyle
{
    FontId        font = 0;
    float         size = 16.0f;
    FontWeight    weight = FontWeight::Regular;
    TextAlignment alignment = TextAlignment::Left;
    float         lineHeight = 1.2f;
    float         letterSpacing = 0.0f;
    math::Quat    color = {1.0f, 1.0f, 1.0f, 1.0f}; // RGBA: w=R, x=G, y=B, z=A
};

// ── Glyph placement ──

struct GlyphPlacement
{
    GlyphId     glyph = 0;
    std::size_t sourceByteBegin = 0;
    std::size_t sourceByteEnd = 0;
    math::Vec3f position = {0.0f, 0.0f, 0.0f};
    math::Vec3f size     = {0.0f, 0.0f, 0.0f};
    Bounds      atlasRect; // UV rectangle in glyph atlas
    math::Quat  color = {1.0f, 1.0f, 1.0f, 1.0f};
    bool        hasColorOverride = false;
};

// ── Shaped text ──

struct ShapedText
{
    std::vector<GlyphPlacement> glyphs;
    math::Vec3f size = {0.0f, 0.0f, 0.0f};
    Bounds      bounds;
};

// ── Text visual descriptor ──

struct TextVisualDescriptor
{
    std::string text;
    TextStyle   style;
};

// ── Text Shaper ──

/// Abstract text shaper. Implementations use HarfBuzz or other shaping engines.
class TextShaper {
public:
    virtual ~TextShaper() = default;

    /// Shape text with the given style. Returns glyph placements with positions
    /// and sizes. The atlasRect fields in GlyphPlacement will be empty (zero)
    /// until rasterized via FontAtlas::rasterize_glyph().
    virtual ShapedText shape(std::string_view text,
                             const TextStyle& style,
                             float maxWidth = 0.0f) const = 0;
};

// ── Shaper factory ──

/// Create a HarfBuzz-backed text shaper.
/// The shaper holds a reference to the atlas and must not outlive it.
std::unique_ptr<TextShaper> create_harfbuzz_shaper(const FontAtlas& atlas);

// ── Glyph mesh generation ──

/// Generate a textured quad mesh for a single glyph.
/// The glyph's atlasRect must be filled in (via FontAtlas::rasterize_glyph) beforehand.
/// The quad is placed at glyph.position with dimensions glyph.size, UV-mapped to atlasRect.
/// An optional vertex color can be applied (defaults to white).
MeshData generate_glyph_mesh(const GlyphPlacement& glyph,
                              math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f},
                              float sdfMarginLayout = 0.0f);

/// Generate a combined mesh for all glyphs in a shaped text run.
/// Calls rasterize_glyph on atlas for any glyphs that haven't been rasterized yet.
/// The output mesh contains one quad per glyph, positioned and UV-mapped.
/// An optional vertex color can be applied (defaults to white).
MeshData generate_text_mesh(const ShapedText& shaped, FontAtlas& atlas,
                             math::Quat color = {1.0f, 1.0f, 1.0f, 1.0f});

} // namespace exd::geometry

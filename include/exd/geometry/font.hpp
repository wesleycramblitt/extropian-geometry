#pragma once

#include <exd/geometry/text.hpp>
#include <exd/math/vec3.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace exd::geometry {

/// Manages font loading and glyph rasterization into a shared texture atlas.
/// Uses FreeType internally. Callers can query atlas data for GPU upload.
class FontAtlas {
public:
    /// Create an atlas with the given texture dimensions (power-of-2 recommended).
    explicit FontAtlas(int atlasWidth = 512, int atlasHeight = 512);
    ~FontAtlas();

    FontAtlas(const FontAtlas&) = delete;
    FontAtlas& operator=(const FontAtlas&) = delete;
    FontAtlas(FontAtlas&&) noexcept;
    FontAtlas& operator=(FontAtlas&&) noexcept;

    /// Load a font file. Returns a FontId for use in TextStyle.
    /// faceIndex selects a face within a font collection (.ttc); default is 0.
    /// Returns 0 on failure.
    FontId load_font(const std::string& path, int faceIndex = 0);

    /// Rasterize a glyph at the given font size into the atlas.
    /// Returns the UV rectangle (normalized 0-1 atlas coordinates) for the glyph.
    /// If the glyph is already rasterized at this size, returns the cached rect.
    Bounds rasterize_glyph(FontId font, GlyphId glyph, float fontSize);

    /// Look up a previously rasterized glyph's atlas rectangle.
    /// Returns false if the glyph hasn't been rasterized yet.
    bool get_glyph_rect(FontId font, GlyphId glyph, float fontSize, Bounds& outRect) const;

    /// Raw RGBA8 pixel data for the atlas texture. One byte per channel, 4 bytes per pixel.
    std::span<const uint8_t> atlas_data() const;

    /// Atlas texture dimensions in pixels.
    int atlas_width() const;
    int atlas_height() const;

    /// Get glyph metrics for a font at a given size.
    /// outAdvance: horizontal advance for this glyph (pixels)
    /// outSize: width/height of the glyph bitmap (pixels)
    bool get_glyph_metrics(FontId font, GlyphId glyph, float fontSize,
                           float& outAdvance, math::Vec3f& outSize) const;

    /// Create a HarfBuzz font object for use with text shaping.
    /// The caller is responsible for destroying the returned hb_font_t with hb_font_destroy().
    /// Returns nullptr if the font hasn't been loaded.
    void* create_hb_font(FontId font) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace exd::geometry

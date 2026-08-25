#pragma once

#include <exd/geometry/text.hpp>
#include <exd/math/vec3.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace exd::geometry {

// ── Default font selection ──

enum class DefaultFont
{
    Sans,       // sans-serif (e.g. DejaVu Sans, Liberation Sans, Arial)
    Serif,      // serif (e.g. DejaVu Serif, Liberation Serif, Times New Roman)
    Mono,       // monospace (e.g. DejaVu Sans Mono, Liberation Mono, Courier New)
};

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

    /// Load a font from an in-memory buffer (for embedded/bundled fonts).
    /// The buffer must remain valid for the lifetime of the FontAtlas.
    /// Returns 0 on failure.
    FontId load_font_memory(const uint8_t* data, size_t size, int faceIndex = 0);

    /// Load a system default font. Searches registered search paths
    /// (see add_font_search_path) for matching font files.
    /// Returns 0 if no matching default font is found.
    FontId load_default(DefaultFont which);

    /// Register a directory to search for default fonts.
    /// Paths are searched in the order they are added.
    /// No paths are registered by default — call this to populate the search list.
    void add_font_search_path(const std::string& directory);

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
    void* create_hb_font(FontId font, float fontSize = 16.0f) const;

    /// UV coordinate of a small opaque white texel reserved in this atlas.
    /// Use this for solid-fill geometry (e.g. a math fraction bar) merged
    /// into a mesh that otherwise samples this atlas as glyph quads — such
    /// a quad's own UVs would otherwise land on whatever happens to be
    /// packed there (usually mostly-transparent gutter) and render as an
    /// invisible hairline instead of a solid fill.
    math::Vec3f solid_uv() const;

    /// Initialize default font search paths including bundled fonts.
    void initialize_default_paths();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace exd::geometry

#include <exd/geometry/text.hpp>
#include <exd/geometry/font.hpp>

#include <hb.h>
#include <hb-ft.h>

#include <cmath>
#include <memory>
#include <string>

namespace exd::geometry {

// ── HarfBuzz Shaper Implementation ──

class HarfBuzzShaper : public TextShaper {
public:
    explicit HarfBuzzShaper(const FontAtlas& atlas) : atlas_(atlas) {}

    ShapedText shape(std::string_view text, const TextStyle& style,
                     float /*maxWidth*/) const override {
        ShapedText result;
        if (text.empty()) return result;

        // Create hb_font from atlas
        hb_font_t* hbFont = static_cast<hb_font_t*>(atlas_.create_hb_font(style.font));
        if (!hbFont) return result;

        // Create buffer
        hb_buffer_t* buf = hb_buffer_create();
        hb_buffer_add_utf8(buf, text.data(), static_cast<int>(text.size()), 0, static_cast<int>(text.size()));
        hb_buffer_set_direction(buf, HB_DIRECTION_LTR);
        hb_buffer_set_script(buf, HB_SCRIPT_LATIN);
        hb_buffer_set_language(buf, hb_language_from_string("en", -1));

        // Shape
        hb_shape(hbFont, buf, nullptr, 0);

        // Get results
        unsigned int glyphCount = 0;
        hb_glyph_info_t* glyphInfo = hb_buffer_get_glyph_infos(buf, &glyphCount);
        hb_glyph_position_t* glyphPos = hb_buffer_get_glyph_positions(buf, &glyphCount);

        // Scale factor: HarfBuzz positions are in font units (26.6 fixed point)
        // Convert to pixels by dividing by 64
        const float scale = 1.0f / 64.0f;

        float cursorX = 0.0f;
        float cursorY = 0.0f;

        for (unsigned int i = 0; i < glyphCount; ++i) {
            GlyphPlacement gp;
            gp.glyph = glyphInfo[i].codepoint;
            gp.position = {
                cursorX + static_cast<float>(glyphPos[i].x_offset) * scale,
                cursorY + static_cast<float>(glyphPos[i].y_offset) * scale,
                0.0f
            };

            // Get glyph metrics from atlas
            float advance = 0.0f;
            math::Vec3f gsize;
            if (atlas_.get_glyph_metrics(style.font, gp.glyph, style.size, advance, gsize)) {
                gp.size = gsize;
            } else {
                gp.size = {style.size, style.size, 0.0f}; // fallback
            }

            result.glyphs.push_back(gp);

            cursorX += static_cast<float>(glyphPos[i].x_advance) * scale;
            cursorY += static_cast<float>(glyphPos[i].y_advance) * scale;
        }

        // Compute bounds
        float maxX = 0.0f, maxY = 0.0f;
        for (const auto& g : result.glyphs) {
            float right = g.position.x + g.size.x;
            float top   = g.position.y + g.size.y;
            if (right > maxX) maxX = right;
            if (top   > maxY) maxY = top;
        }
        result.size = {maxX, maxY, 0.0f};
        result.bounds = {{0, 0, 0}, {maxX, maxY, 0}};

        // Cleanup
        hb_buffer_destroy(buf);
        hb_font_destroy(hbFont);

        return result;
    }

private:
    const FontAtlas& atlas_;
};

// ── Factory ──

std::unique_ptr<TextShaper> create_harfbuzz_shaper(const FontAtlas& atlas) {
    return std::make_unique<HarfBuzzShaper>(atlas);
}

} // namespace exd::geometry

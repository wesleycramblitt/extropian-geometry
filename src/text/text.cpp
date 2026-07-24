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
                     float maxWidth) const override {
        ShapedText result;
        if (text.empty()) return result;

        // Create hb_font from atlas
        hb_font_t* hbFont = static_cast<hb_font_t*>(atlas_.create_hb_font(style.font));
        if (!hbFont) return result;

        const float scale = 1.0f / 64.0f;
        const float letterSpacing = style.letterSpacing;

        // Shape the full text via HarfBuzz
        auto shape_run = [&](std::string_view run) -> std::vector<GlyphPlacement> {
            std::vector<GlyphPlacement> glyphs;
            if (run.empty()) return glyphs;

            hb_buffer_t* buf = hb_buffer_create();
            hb_buffer_add_utf8(buf, run.data(), static_cast<int>(run.size()), 0, static_cast<int>(run.size()));
            hb_buffer_set_direction(buf, HB_DIRECTION_LTR);
            hb_buffer_set_script(buf, HB_SCRIPT_LATIN);
            hb_buffer_set_language(buf, hb_language_from_string("en", -1));
            hb_shape(hbFont, buf, nullptr, 0);

            unsigned int glyphCount = 0;
            hb_glyph_info_t* glyphInfo = hb_buffer_get_glyph_infos(buf, &glyphCount);
            hb_glyph_position_t* glyphPos = hb_buffer_get_glyph_positions(buf, &glyphCount);

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

                float advance = 0.0f;
                math::Vec3f gsize;
                if (atlas_.get_glyph_metrics(style.font, gp.glyph, style.size, advance, gsize)) {
                    gp.size = gsize;
                } else {
                    gp.size = {style.size, style.size, 0.0f};
                }

                glyphs.push_back(gp);

                cursorX += static_cast<float>(glyphPos[i].x_advance) * scale + letterSpacing;
                cursorY += static_cast<float>(glyphPos[i].y_advance) * scale;
            }

            hb_buffer_destroy(buf);
            return glyphs;
        };

        // ── Line breaking (basic word-wrap) ──────────────────────────────
        float lineHeight = style.size * style.lineHeight;
        float currentY = 0.0f;
        std::vector<std::vector<GlyphPlacement>> lines;

        if (maxWidth <= 0.0f) {
            // No wrapping: single line
            lines.push_back(shape_run(text));
        } else {
            // Word-wrap: split text at spaces, accumulate until maxWidth exceeded
            std::string currentLine;
            float currentLineWidth = 0.0f;
            std::string remaining(text);
            size_t wordStart = 0;

            while (wordStart < remaining.size()) {
                // Find next word boundary
                size_t wordEnd = remaining.find(' ', wordStart);
                if (wordEnd == std::string::npos) wordEnd = remaining.size();

                std::string_view word(remaining.data() + wordStart, wordEnd - wordStart);
                auto wordGlyphs = shape_run(word);
                float wordWidth = 0.0f;
                for (const auto& g : wordGlyphs)
                    wordWidth += g.size.x + letterSpacing;

                if (!currentLine.empty() && currentLineWidth + wordWidth > maxWidth) {
                    // Wrap: emit current line
                    lines.push_back(shape_run(currentLine));
                    currentLine.clear();
                    currentLineWidth = 0.0f;
                }

                if (!currentLine.empty()) {
                    currentLine += ' ';
                    currentLineWidth += wordWidth; // rough: space width approximated
                }
                currentLine += std::string(word);
                auto newGlyphs = shape_run(currentLine);
                currentLineWidth = 0.0f;
                for (const auto& g : newGlyphs) currentLineWidth += g.size.x + letterSpacing;

                wordStart = wordEnd;
                if (wordStart < remaining.size() && remaining[wordStart] == ' ')
                    ++wordStart; // skip the space
            }
            if (!currentLine.empty())
                lines.push_back(shape_run(currentLine));
        }

        // ── Apply alignment & position each line ─────────────────────────
        for (auto& lineGlyphs : lines) {
            // Compute total line width (for alignment)
            float lineWidth = 0.0f;
            float maxGlyphHeight = 0.0f;
            for (const auto& g : lineGlyphs) {
                float right = g.position.x + g.size.x;
                if (right > lineWidth) lineWidth = right;
                if (g.size.y > maxGlyphHeight) maxGlyphHeight = g.size.y;
            }

            // Alignment offset
            float alignOffset = 0.0f;
            switch (style.alignment) {
            case TextAlignment::Center:
                alignOffset = (maxWidth > 0.0f ? maxWidth - lineWidth : 0.0f) * 0.5f;
                break;
            case TextAlignment::Right:
                alignOffset = (maxWidth > 0.0f ? maxWidth - lineWidth : 0.0f);
                break;
            default:
                break;
            }

            for (auto& g : lineGlyphs) {
                g.position.x += alignOffset;
                g.position.y += currentY;
                result.glyphs.push_back(g);
            }
            currentY += maxGlyphHeight * lineHeight;
        }

        // ── Compute bounds ────────────────────────────────────────────────
        float maxX = 0.0f, maxY = 0.0f;
        for (const auto& g : result.glyphs) {
            float right = g.position.x + g.size.x;
            float top   = g.position.y + g.size.y;
            if (right > maxX) maxX = right;
            if (top   > maxY) maxY = top;
        }
        result.size = {maxX, maxY, 0.0f};
        result.bounds = {{0, 0, 0}, {maxX, maxY, 0}};

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

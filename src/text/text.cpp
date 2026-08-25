#include <exd/geometry/text.hpp>
#include <exd/geometry/font.hpp>

#include <hb.h>
#include <hb-ft.h>

#include <algorithm>
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
        hb_font_t* hbFont = static_cast<hb_font_t*>(
            atlas_.create_hb_font(style.font, style.size));
        if (!hbFont) return result;

        const float scale = 1.0f / 64.0f;
        const float letterSpacing = style.letterSpacing;

        // Shape the full text via HarfBuzz
        auto shape_run = [&](std::string_view run, std::size_t sourceBase = 0)
            -> std::vector<GlyphPlacement> {
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
                gp.sourceByteBegin = sourceBase + glyphInfo[i].cluster;
                auto clusterEnd = run.size();
                for (unsigned int j = i + 1; j < glyphCount; ++j) {
                    if (glyphInfo[j].cluster > glyphInfo[i].cluster) {
                        clusterEnd = glyphInfo[j].cluster;
                        break;
                    }
                }
                gp.sourceByteEnd = sourceBase + clusterEnd;
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

        auto wrap_paragraph = [&](std::string_view paragraph,
                                  std::size_t paragraphSourceStart) {
            if (maxWidth <= 0.0f) {
                lines.push_back(shape_run(paragraph, paragraphSourceStart));
                return;
            }
            // Word-wrap: split at spaces, accumulating complete words.
            std::string currentLine;
            std::size_t currentLineSourceStart = paragraphSourceStart;
            std::size_t wordStart = 0;

            // HarfBuzz positions glyphs using advances, kerning, and bearings.
            // Summing bitmap widths is not a valid line measurement: it omits
            // spaces and can disagree substantially with the rendered extent.
            // Wrapping must use the same shaped positions that create the mesh.
            auto shaped_width = [](const std::vector<GlyphPlacement>& glyphs) {
                float minX = 0.0f;
                float maxX = 0.0f;
                bool initialized = false;
                for (const auto& glyph : glyphs) {
                    const float left = glyph.position.x;
                    const float right = glyph.position.x + glyph.size.x;
                    if (!initialized) {
                        minX = left;
                        maxX = right;
                        initialized = true;
                    } else {
                        minX = std::min(minX, left);
                        maxX = std::max(maxX, right);
                    }
                }
                return initialized ? maxX - std::min(0.0f, minX) : 0.0f;
            };

            while (wordStart < paragraph.size()) {
                std::size_t wordEnd = paragraph.find(' ', wordStart);
                if (wordEnd == std::string::npos) wordEnd = paragraph.size();

                std::string_view word(paragraph.data() + wordStart, wordEnd - wordStart);
                std::string candidate = currentLine;
                if (!candidate.empty()) candidate += ' ';
                candidate += word;
                const auto candidateGlyphs = shape_run(candidate);

                if (!currentLine.empty() && shaped_width(candidateGlyphs) > maxWidth) {
                    // Wrap: emit current line
                    lines.push_back(shape_run(currentLine, currentLineSourceStart));
                    currentLine.clear();
                }

                if (currentLine.empty())
                    currentLineSourceStart = paragraphSourceStart + wordStart;
                if (!currentLine.empty()) {
                    currentLine += ' ';
                }
                currentLine += std::string(word);

                wordStart = wordEnd;
                if (wordStart < paragraph.size() && paragraph[wordStart] == ' ')
                    ++wordStart; // skip the space
            }
            if (!currentLine.empty())
                lines.push_back(shape_run(currentLine, currentLineSourceStart));
        };

        // Explicit newlines create paragraph/blank-line boundaries even when
        // wrapping is disabled. Source byte offsets remain relative to the
        // original UTF-8 string for semantic spans and interaction.
        std::size_t paragraphStart = 0;
        while (paragraphStart <= text.size()) {
            const auto paragraphEnd = text.find('\n', paragraphStart);
            const auto end = paragraphEnd == std::string_view::npos
                ? text.size() : paragraphEnd;
            wrap_paragraph(text.substr(paragraphStart, end - paragraphStart),
                           paragraphStart);
            if (end == text.size()) break;
            if (end == paragraphStart) lines.emplace_back();
            paragraphStart = end + 1;
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
            // Screen-space documents use Y-up coordinates, while prose reads
            // top-to-bottom. Subsequent baselines therefore advance downward.
            currentY -= lineHeight;
        }

        // ── Compute exact bounds ──────────────────────────────────────────
        // Glyph bearings put descenders below the baseline. Starting bounds at
        // zero loses that part of the line and makes measurement disagree with
        // the generated glyph mesh, which in turn causes callers to place text
        // against a clipped bottom edge.
        if (!result.glyphs.empty()) {
            float minX = result.glyphs.front().position.x;
            float minY = result.glyphs.front().position.y + result.glyphs.front().size.z;
            float maxX = minX + result.glyphs.front().size.x;
            float maxY = minY + result.glyphs.front().size.y;
            for (const auto& g : result.glyphs) {
                const float left = g.position.x;
                const float bottom = g.position.y + g.size.z;
                minX = std::min(minX, left);
                minY = std::min(minY, bottom);
                maxX = std::max(maxX, left + g.size.x);
                maxY = std::max(maxY, bottom + g.size.y);
            }
            result.size = {maxX - minX, maxY - minY, 0.0f};
            result.bounds = {{minX, minY, 0}, {maxX, maxY, 0}};
        }

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

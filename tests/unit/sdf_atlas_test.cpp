#include <doctest/doctest.h>
#include <exd/geometry/font.hpp>
#include <exd/math/vec3.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

using namespace exd::geometry;
using namespace exd::math;

namespace {

std::string resolve_font_dir()
{
    if (const char* env = std::getenv("EXD_TEST_FONT_DIR"))
        return std::string(env) + "/";
    for (const char* sys : {"/usr/share/fonts/TTF/",
                             "/usr/share/fonts/liberation/",
                             "/usr/share/fonts/noto/",
                             "/usr/share/fonts/truetype/dejavu/",
                             "/usr/share/fonts/truetype/liberation/",
                             "/usr/share/fonts/truetype/noto/"}) {
        if (std::filesystem::exists(std::string(sys) + "DejaVuSans.ttf"))
            return sys;
    }
    return "/usr/share/fonts/TTF/";
}

std::string TEST_FONT() { return resolve_font_dir() + "DejaVuSans.ttf"; }

// DejaVu's cmap does not map codepoints to identity glyph indices; tests
// must pass the REAL glyph index (the library always receives indices from
// HarfBuzz). Resolve once via FreeType over the test font file.
GlyphId char_index(char c)
{
    FT_Library ft = nullptr;
    FT_Init_FreeType(&ft);
    FT_Face face = nullptr;
    FT_New_Face(ft, TEST_FONT().c_str(), 0, &face);
    GlyphId id = (face && ft)
        ? FT_Get_Char_Index(face, static_cast<unsigned char>(c)) : 0;
    if (face) FT_Done_Face(face);
    if (ft) FT_Done_FreeType(ft);
    return id;
}

// Access helpers on the raw atlas data
int alpha_at(const FontAtlas& a, int x, int y)
{
    const auto data = a.atlas_data();
    return data[static_cast<size_t>(y) * a.atlas_width() * 4
                + static_cast<size_t>(x) * 4 + 3];
}

} // namespace

// ── SDF parameters ──────────────────────────────────────────────────────────

TEST_CASE("sdf: parameters exposed and sane")
{
    FontAtlas atlas(256, 256, 1.0f, 4);
    CHECK(atlas.sdf_scale() == doctest::Approx(1.0f));
    CHECK(atlas.sdf_margin() == 4);
    CHECK(atlas.sdf_margin_layout() == doctest::Approx(4.0f));

    FontAtlas scaled(512, 512, 2.0f, 8);
    CHECK(scaled.sdf_margin() == 8);
    CHECK(scaled.sdf_margin_layout() == doctest::Approx(4.0f)); // 8 / 2

    // Defaults: SDF on, 1:1 raster scale, 4px margin
    FontAtlas dflt(128, 128);
    CHECK(dflt.sdf_scale() == doctest::Approx(1.0f));
    CHECK(dflt.sdf_margin() == 4);
}

TEST_CASE("sdf: ctor clamps degenerate parameters")
{
    FontAtlas bogus(64, 64, 0.0f, 0);
    // (char_index needs a font load source only; reuse TEST_FONT path)
    CHECK(bogus.sdf_scale() == doctest::Approx(1.0f));   // clamped up
    CHECK(bogus.sdf_margin() == 1);                      // clamped up
    // Rasterizing with clamped params must still produce a sane field
    FontId font = bogus.load_font(TEST_FONT());
    REQUIRE(font != 0);
    const GlyphId gI = char_index('I');
    REQUIRE(gI != 0);
    const Bounds rect = bogus.rasterize_glyph(font, gI, 16.0f);
    REQUIRE(rect.max.x > rect.min.x); // no NaN/invisible degenerate rect
}

TEST_CASE("sdf: solid patch stays opaque at (0,0)")
{
    FontAtlas atlas(256, 256);
    CHECK(alpha_at(atlas, 0, 0) == 255);
    CHECK(alpha_at(atlas, 1, 1) == 255);
    // Unpacked gutter is far-outside
    CHECK(alpha_at(atlas, 100, 100) == 0);
}

// ── SDF glyph rasterization (needs a real font) ─────────────────────────────

TEST_CASE("sdf: 'I' rasterizes as a signed field with outline at 0.5")
{
    FontAtlas atlas(256, 256, 1.0f, 4);
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    const GlyphId gI = char_index('I');
    REQUIRE(gI != 0);
    const Bounds rect = atlas.rasterize_glyph(font, gI, 16.0f);
    REQUIRE(rect.max.x > rect.min.x);
    REQUIRE(rect.max.y > rect.min.y);

    const int aw = atlas.atlas_width();
    const int ah = atlas.atlas_height();
    const int x0 = static_cast<int>(rect.min.x * aw + 0.5f);
    const int y0 = static_cast<int>(rect.min.y * ah + 0.5f);
    const int x1 = static_cast<int>(rect.max.x * aw + 0.5f);
    const int y1 = static_cast<int>(rect.max.y * ah + 0.5f);
    const int w = x1 - x0 + 1;
    const int h = y1 - y0 + 1;

    // Padded rect is larger than the raw ink bbox + margin (tolerant:
    // bbox vs bitmap rounding can differ by a pixel or two)
    float inkW = 0.0f, inkH = 0.0f, advance = 0.0f, bearingY = 0.0f;
    Vec3f ink;
    REQUIRE(atlas.get_glyph_metrics(font, gI, 16.0f, advance, ink));
    REQUIRE(ink.x > 0.0f);
    CHECK(w >= static_cast<int>(ink.x) + 2 * 4 - 2);
    CHECK(h >= static_cast<int>(ink.y) + 2 * 4 - 2);

    // Rect edge midpoints are far-outside (field clamped to 0)
    CHECK(alpha_at(atlas, x0 + w / 2, y0) <= 8);
    CHECK(alpha_at(atlas, x0 + w / 2, y1) <= 8);
    CHECK(alpha_at(atlas, x0, y0 + h / 2) <= 8);
    CHECK(alpha_at(atlas, x1, y0 + h / 2) <= 8);

    // Ink interior: the SDF encodes 0.5 - d/(2M); a 16px 'I' stem/serif is
    // 1-4px thick so the peak alpha sits around 0.6-0.9 (the shader's
    // smoothstep turns anything > 0.5 + fwidth into SOLID — saturation to
    // 255 would require ink deeper than the margin). Assert the classic
    // SDF signature instead: a peak well above the 0.5 outline in the
    // middle of the rect, not at the border.
    const int cx = x0 + w / 2;
    const int cy = y0 + h / 2;
    int peak = 0, peakX = -1;
    for (int x = x0; x <= x1; ++x) {
        const int a = alpha_at(atlas, x, cy);
        if (a > peak) { peak = a; peakX = x; }
    }
    REQUIRE(peak >= 150);
    REQUIRE(peakX >= x0 + w / 4);
    REQUIRE(peakX <= x0 + 3 * w / 4);

    // Outline crossing: along the midline, alpha passes through ~0.5
    bool sawCross = false;
    for (int x = x0; x <= x1; ++x) {
        const int a = alpha_at(atlas, x, cy);
        if (a >= 90 && a <= 170) { sawCross = true; break; }
    }
    CHECK(sawCross);

    // Falloff toward the edge: from x0 to the peak the profile must not
    // DECREASE significantly (the outline can legitimately sit between two
    // sample points, allowing a rise of up to ~2px of distance per step, but
    // a decrease of >8 on the way to the peak would mean a field hole).
    int prev = -1;
    bool monotone = true;
    for (int x = x0; x <= peakX; ++x) {
        const int a = alpha_at(atlas, x, cy);
        if (a < prev - 8) { monotone = false; break; }
        prev = a;
    }
    CHECK(monotone);

    // Cache: second call returns the identical rect
    const Bounds again = atlas.rasterize_glyph(font, gI, 16.0f);
    CHECK(again.min.x == doctest::Approx(rect.min.x));
    CHECK(again.max.y == doctest::Approx(rect.max.y));

    // Deterministic: rasterizing at a different size packs a NEW rect
    const Bounds other = atlas.rasterize_glyph(font, gI, 24.0f);
    REQUIRE(other.max.x > other.min.x);
    CHECK(other.min.x != doctest::Approx(rect.min.x));
}

// ── Production config: 2x raster scale, 8px raster margin ─────────────────

TEST_CASE("sdf: production 2x/8px config keeps shape and metrics scale")
{
    FontAtlas atlas(256, 256, 2.0f, 8);
    CHECK(atlas.sdf_scale() == doctest::Approx(2.0f));
    CHECK(atlas.sdf_margin() == 8);
    CHECK(atlas.sdf_margin_layout() == doctest::Approx(4.0f)); // 8 / 2

    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    // ORDER MATTERS regression: rasterizing FIRST must NOT poison the
    // shared metrics cache with 2x-scale values (the get_glyph_metrics key
    // is the same {font, glyph, fontSize}).
    const GlyphId gY = char_index('y');
    REQUIRE(gY != 0);
    const Bounds rect = atlas.rasterize_glyph(font, gY, 24.0f);
    REQUIRE(rect.max.x > rect.min.x);

    float advance = 0.0f;
    Vec3f ink;
    REQUIRE(atlas.get_glyph_metrics(font, gY, 24.0f, advance, ink));
    // Layout-scale values: DejaVu 'y' at 24px advances ~14px and inks
    // ~13px wide; a poisoned 2x raster slot would report ~28/26 — the point
    // of the ordering regression.
    CHECK(advance > 8.0f);
    CHECK(advance < 20.0f);
    CHECK(ink.x > 4.0f);
    CHECK(ink.x < 20.0f);

    // Padding math: rect covers ink + 2*margin (raster px) at 2x scale
    const int aw = atlas.atlas_width();
    const int wpx = static_cast<int>((rect.max.x - rect.min.x) * aw + 0.5f);
    CHECK(wpx >= static_cast<int>(ink.x * 2.0f) + 2 * 8 - 4); // tolerant
}

// ── Counter glyphs: the two-transform signed field must punch the hole ────

TEST_CASE("sdf: counter glyph 'o' has an outside hole at its center")
{
    FontAtlas atlas(256, 256, 1.0f, 4);
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    const GlyphId gO = char_index('o');
    REQUIRE(gO != 0);
    const Bounds rect = atlas.rasterize_glyph(font, gO, 24.0f);
    REQUIRE(rect.max.x > rect.min.x);

    const int aw = atlas.atlas_width();
    const int x0 = static_cast<int>(rect.min.x * aw + 0.5f);
    const int y0 = static_cast<int>(rect.min.y * aw + 0.5f);
    const int x1 = static_cast<int>(rect.max.x * aw + 0.5f);
    const int y1 = static_cast<int>(rect.max.y * aw + 0.5f);
    const int w = x1 - x0 + 1;
    const int h = y1 - y0 + 1;
    const int cx = x0 + w / 2;
    const int cy = y0 + h / 2;

    // The counter (the hole) is OUTSIDE: the center sits ~3px inside a 4px
    // margin, so it must read well below the 0.5 outline (a filled counter
    // would read >= 150 here).
    CHECK(alpha_at(atlas, cx, cy) <= 96);
    // But a point below center (the bottom stroke) must be inside-ish
    // (>= the outline value)
    int bottomPeak = 0;
    for (int y = cy; y <= y1; ++y)
        bottomPeak = std::max(bottomPeak, alpha_at(atlas, cx, y));
    CHECK(bottomPeak >= 150);
}

// ── Single-pixel stems stay resolvable at the production raster ratio ──────

TEST_CASE("sdf: 2x raster resolves a hairline 'l' stem with an inside peak")
{
    FontAtlas atlas(256, 256, 2.0f, 8);
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    const GlyphId gL = char_index('l');
    REQUIRE(gL != 0);
    const Bounds rect = atlas.rasterize_glyph(font, gL, 12.0f); // 'l' at 12px
    REQUIRE(rect.max.x > rect.min.x);

    const int aw = atlas.atlas_width();
    const int x0 = static_cast<int>(rect.min.x * aw + 0.5f);
    const int y0 = static_cast<int>(rect.min.y * aw + 0.5f);
    const int x1 = static_cast<int>(rect.max.x * aw + 0.5f);
    const int y1 = static_cast<int>(rect.max.y * aw + 0.5f);
    int peak = 0;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            peak = std::max(peak, alpha_at(atlas, x, y));
        }
    }
    // At 2x the stem is 2 texels: the field plateau must exceed the 0.5
    // outline by a margin-wide step (a ~1 device px stem encodes s=-1
    // raster px -> a = 0.5 + 1/16 = 0.5625 -> >= 143); verify >= 140.
    CHECK(peak >= 140);
}

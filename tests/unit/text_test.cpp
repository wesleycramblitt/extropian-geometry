#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>
#include <exd/math/vec3.hpp>

#include <hb.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace exd::geometry;
using namespace exd::math;

// ── Font path resolution ────────────────────────────────────────────────────
// Resolves test font directory: EXD_TEST_FONT_DIR env var > system fallback

static std::string resolve_font_dir()
{
    // 1. Environment variable override
    if (const char* env = std::getenv("EXD_TEST_FONT_DIR"))
        return std::string(env) + "/";

    // 2. System fallback (Linux)
    for (const char* sys : {"/usr/share/fonts/TTF/",
                             "/usr/share/fonts/liberation/",
                             "/usr/share/fonts/noto/"}) {
        if (std::filesystem::exists(std::string(sys) + "DejaVuSans.ttf"))
            return sys;
    }

    return "/usr/share/fonts/TTF/"; // last resort
}

static std::string font_path(const char* name) {
    return resolve_font_dir() + name;
}

// ── Font inventory ──────────────────────────────────────────────────────────
// Test font paths (only those present on the system are used)

struct FontEntry
{
    std::string path;
    const char* category;
};

static std::vector<FontEntry> build_inventory()
{
    std::string dir = resolve_font_dir();
    std::vector<FontEntry> inv;
    auto add = [&](const char* name, const char* cat) {
        std::string p = dir + name;
        if (std::filesystem::exists(p))
            inv.push_back({p, cat});
    };
    add("DejaVuSans.ttf",                 "sans");
    add("LiberationSans-Regular.ttf",     "sans");
    add("NotoSans-Regular.ttf",           "sans");
    add("DejaVuSerif.ttf",                "serif");
    add("LiberationSerif-Regular.ttf",    "serif");
    add("NotoSerif-Regular.ttf",          "serif");
    add("DejaVuSansMono.ttf",             "mono");
    add("LiberationMono-Regular.ttf",     "mono");
    add("NotoSansMono-Regular.ttf",       "mono");
    add("DejaVuSans-Bold.ttf",            "bold");
    return inv;
}

// Primary test font
static std::string TEST_FONT() { return font_path("DejaVuSans.ttf"); }

// Valid glyph indices in DejaVu Sans (indices 1-3 are empty/non-renderable)
// Glyph 4 = valid small glyph, Glyph 6 = valid medium glyph
static const GlyphId VALID_GLYPH_1 = 4;
static const GlyphId VALID_GLYPH_2 = 6;
static const GlyphId VALID_GLYPH_3 = 7;

// ── FontAtlas Tests ──

TEST_CASE("FontAtlas: construction and destruction") {
    FontAtlas atlas(256, 256);
    CHECK(atlas.atlas_width() == 256);
    CHECK(atlas.atlas_height() == 256);
    CHECK(atlas.atlas_data().size() == 256 * 256 * 4);
}

TEST_CASE("FontAtlas: load valid font file") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    CHECK(font != 0);
}

TEST_CASE("FontAtlas: load invalid font returns 0") {
    FontAtlas atlas;
    FontId font = atlas.load_font("/nonexistent/font.ttf");
    CHECK(font == 0);
}

TEST_CASE("FontAtlas: rasterize glyph produces non-zero rect") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    // Glyph index 4 is a valid glyph in DejaVu Sans
    Bounds rect = atlas.rasterize_glyph(font, VALID_GLYPH_1, 24.0f);

    // Rect should be non-zero if rasterization succeeded
    CHECK((rect.max.x - rect.min.x) > 0.0f);
    CHECK((rect.max.y - rect.min.y) > 0.0f);
}

TEST_CASE("FontAtlas: get_glyph_rect returns cached result") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    Bounds r1 = atlas.rasterize_glyph(font, VALID_GLYPH_2, 24.0f);

    Bounds r2;
    bool found = atlas.get_glyph_rect(font, VALID_GLYPH_2, 24.0f, r2);
    CHECK(found);
    CHECK(r2.min.x == doctest::Approx(r1.min.x));
    CHECK(r2.min.y == doctest::Approx(r1.min.y));
    CHECK(r2.max.x == doctest::Approx(r1.max.x));
    CHECK(r2.max.y == doctest::Approx(r1.max.y));
}

TEST_CASE("FontAtlas: get_glyph_rect returns false for uncached glyph") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    Bounds r;
    bool found = atlas.get_glyph_rect(font, 9999, 24.0f, r);
    CHECK(!found);
}

TEST_CASE("FontAtlas: get_glyph_metrics returns valid data") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    // First rasterize to ensure metrics are computed
    atlas.rasterize_glyph(font, VALID_GLYPH_3, 24.0f);

    float advance;
    Vec3f size;
    bool ok = atlas.get_glyph_metrics(font, VALID_GLYPH_3, 24.0f, advance, size);
    CHECK(ok);
    CHECK(advance > 0.0f);
    CHECK(size.x >= 0.0f);
}

TEST_CASE("FontAtlas: create_hb_font returns valid pointer") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    void* hbFont = atlas.create_hb_font(font);
    CHECK(hbFont != nullptr);
    // Clean up
    hb_font_destroy(static_cast<hb_font_t*>(hbFont));
}

TEST_CASE("FontAtlas: create_hb_font returns nullptr for invalid font") {
    FontAtlas atlas;
    void* hbFont = atlas.create_hb_font(9999);
    CHECK(hbFont == nullptr);
}

TEST_CASE("FontAtlas: glyph caching avoids re-rasterization") {
    FontAtlas atlas(512, 512);
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    Bounds r1 = atlas.rasterize_glyph(font, VALID_GLYPH_1, 24.0f);
    Bounds r2 = atlas.rasterize_glyph(font, VALID_GLYPH_1, 24.0f);

    // Same glyph at same size should return identical rect
    CHECK(r1.min.x == doctest::Approx(r2.min.x));
    CHECK(r1.min.y == doctest::Approx(r2.min.y));
    CHECK(r1.max.x == doctest::Approx(r2.max.x));
    CHECK(r1.max.y == doctest::Approx(r2.max.y));
}

TEST_CASE("FontAtlas: move semantics") {
    FontAtlas atlas1;
    FontId font = atlas1.load_font(TEST_FONT());
    REQUIRE(font != 0);

    // Rasterize a glyph
    Bounds r1 = atlas1.rasterize_glyph(font, VALID_GLYPH_1, 24.0f);

    FontAtlas atlas2(std::move(atlas1));
    CHECK(atlas2.atlas_width() == 512);
    CHECK(atlas2.atlas_height() == 512);

    // Cached glyph should still be accessible
    Bounds r2;
    bool found = atlas2.get_glyph_rect(font, VALID_GLYPH_1, 24.0f, r2);
    CHECK(found);
    CHECK(r2.min.x == doctest::Approx(r1.min.x));
}

TEST_CASE("FontAtlas: different font sizes produce different rects") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    Bounds r24 = atlas.rasterize_glyph(font, VALID_GLYPH_1, 24.0f);
    Bounds r48 = atlas.rasterize_glyph(font, VALID_GLYPH_1, 48.0f);

    // Larger font size should produce larger glyph
    float w24 = r24.max.x - r24.min.x;
    float w48 = r48.max.x - r48.min.x;
    CHECK(w48 > w24);
}

// ── TextShaper Tests ──

TEST_CASE("TextShaper: shape simple text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    REQUIRE(shaper != nullptr);

    TextStyle style;
    style.font = font;
    style.size = 24.0f;

    ShapedText shaped = shaper->shape("Hello", style);
    CHECK(shaped.glyphs.size() > 0);
    CHECK(shaped.size.x > 0.0f);
    CHECK(shaped.size.y > 0.0f);
}

TEST_CASE("TextShaper: empty text returns empty") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    ShapedText shaped = shaper->shape("", TextStyle{font, 24.0f});
    CHECK(shaped.glyphs.empty());
}

TEST_CASE("TextShaper: shaped text has correct bounds") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    ShapedText shaped = shaper->shape("Test", TextStyle{font, 24.0f});

    CHECK(shaped.bounds.min.x == doctest::Approx(0.0f));
    CHECK(shaped.bounds.min.y == doctest::Approx(0.0f));
    CHECK(shaped.bounds.max.x > 0.0f); // should have some width
}

TEST_CASE("TextShaper: longer text produces more glyphs") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style{font, 24.0f};

    ShapedText short_text = shaper->shape("Hi", style);
    ShapedText long_text  = shaper->shape("Hello World", style);

    CHECK(long_text.glyphs.size() > short_text.glyphs.size());
}

TEST_CASE("TextShaper: glyph positions are monotonically increasing in X") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    ShapedText shaped = shaper->shape("ABCDE", TextStyle{font, 24.0f});

    for (size_t i = 1; i < shaped.glyphs.size(); ++i) {
        CHECK(shaped.glyphs[i].position.x >= shaped.glyphs[i - 1].position.x);
    }
}

TEST_CASE("TextShaper: invalid font returns empty shaped text") {
    FontAtlas atlas;
    auto shaper = create_harfbuzz_shaper(atlas);

    TextStyle style;
    style.font = 0; // invalid
    style.size = 24.0f;

    ShapedText shaped = shaper->shape("Hello", style);
    CHECK(shaped.glyphs.empty());
}

// ── Glyph Mesh Tests ──

TEST_CASE("generate_glyph_mesh: produces valid quad") {
    GlyphPlacement gp;
    gp.glyph = 1;
    gp.position = {10.0f, 20.0f, 0.0f};
    gp.size = {16.0f, 16.0f, 0.0f};
    gp.atlasRect = {{0.0f, 0.0f, 0.0f}, {0.5f, 0.5f, 0.0f}};

    auto mesh = generate_glyph_mesh(gp);
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);
    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    // Check UVs
    CHECK(mesh.vertices[0].uv.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[1].uv.x == doctest::Approx(0.5f));
}

TEST_CASE("generate_glyph_mesh: vertex positions are correct") {
    GlyphPlacement gp;
    gp.position = {100.0f, 200.0f, 0.0f};
    gp.size = {50.0f, 30.0f, 0.0f};
    gp.atlasRect = {{0.1f, 0.2f, 0.0f}, {0.3f, 0.4f, 0.0f}};

    auto mesh = generate_glyph_mesh(gp);

    // Bottom-left
    CHECK(mesh.vertices[0].position.x == doctest::Approx(100.0f));
    CHECK(mesh.vertices[0].position.y == doctest::Approx(200.0f));
    // Top-right
    CHECK(mesh.vertices[2].position.x == doctest::Approx(150.0f));
    CHECK(mesh.vertices[2].position.y == doctest::Approx(230.0f));
}

TEST_CASE("generate_text_mesh: empty shaped text returns empty mesh") {
    ShapedText empty;
    FontAtlas atlas;
    auto mesh = generate_text_mesh(empty, atlas);
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("generate_text_mesh: produces combined mesh") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    ShapedText shaped = shaper->shape("AB", TextStyle{font, 24.0f});
    REQUIRE(shaped.glyphs.size() >= 2);

    // Rasterize each glyph first
    for (auto& gp : shaped.glyphs) {
        gp.atlasRect = atlas.rasterize_glyph(font, gp.glyph, 24.0f);
    }

    auto mesh = generate_text_mesh(shaped, atlas);
    // Should have 2 quads = 8 vertices, 12 indices
    CHECK(mesh.vertices.size() >= 8);
    CHECK(mesh.indices.size() >= 12);
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("generate_text_mesh: skips glyphs with zero atlasRect") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    ShapedText shaped = shaper->shape("AB", TextStyle{font, 24.0f});
    REQUIRE(shaped.glyphs.size() >= 2);

    // Only rasterize first glyph
    shaped.glyphs[0].atlasRect = atlas.rasterize_glyph(font, shaped.glyphs[0].glyph, 24.0f);
    // Leave second glyph with zero rect

    auto mesh = generate_text_mesh(shaped, atlas);
    // Should have 1 quad = 4 vertices, 6 indices
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);
}

// ── End-to-End Tests ──

TEST_CASE("end-to-end: load font, shape text, rasterize, generate mesh") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    ShapedText shaped = shaper->shape("Hello", TextStyle{font, 32.0f});

    REQUIRE(shaped.glyphs.size() > 0);

    // Rasterize all glyphs
    for (auto& gp : shaped.glyphs) {
        gp.atlasRect = atlas.rasterize_glyph(font, gp.glyph, 32.0f);
    }

    // Generate mesh
    auto mesh = generate_text_mesh(shaped, atlas);
    CHECK(mesh.vertices.size() > 0);
    CHECK(mesh.indices.size() > 0);

    // Atlas has data
    CHECK(atlas.atlas_data().size() > 0);
}

TEST_CASE("end-to-end: multiple font sizes in same atlas") {
    FontAtlas atlas(1024, 1024);
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);

    // Shape and rasterize at multiple sizes
    for (float size : {12.0f, 24.0f, 48.0f}) {
        ShapedText shaped = shaper->shape("Test", TextStyle{font, size});
        for (auto& gp : shaped.glyphs) {
            Bounds rect = atlas.rasterize_glyph(font, gp.glyph, size);
            CHECK((rect.max.x - rect.min.x) > 0.0f);
        }
    }
}

TEST_CASE("end-to-end: atlas data is valid RGBA") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    // Rasterize a glyph to populate atlas
    atlas.rasterize_glyph(font, 68, 32.0f); // 'D'

    auto data = atlas.atlas_data();
    CHECK(data.size() == 512 * 512 * 4);

    // Check that some pixels have non-zero alpha (the glyph)
    bool hasAlpha = false;
    for (size_t i = 3; i < data.size(); i += 4) {
        if (data[i] > 0) {
            hasAlpha = true;
            break;
        }
    }
    CHECK(hasAlpha);
}

// ── FontAtlas: faceIndex parameter ──────────────────────────────────────────

TEST_CASE("FontAtlas: load_font with explicit faceIndex parameter") {
    FontAtlas atlas;
    // faceIndex=0 is the default; this tests it doesn't crash
    FontId font = atlas.load_font(TEST_FONT(), 0);
    CHECK(font != 0);
}

TEST_CASE("FontAtlas: load_font with faceIndex on non-collection font") {
    FontAtlas atlas;
    // A single .ttf file only has face 0, but faceIndex is accepted
    FontId font = atlas.load_font(TEST_FONT(), 0);
    CHECK(font != 0);

    // Verify the loaded font works for rasterization
    Bounds rect = atlas.rasterize_glyph(font, VALID_GLYPH_1, 24.0f);
    CHECK((rect.max.x - rect.min.x) > 0.0f);
}

// ── FontAtlas: multiple fonts ───────────────────────────────────────────────

TEST_CASE("FontAtlas: loading same font twice returns different IDs") {
    FontAtlas atlas;
    FontId f1 = atlas.load_font(TEST_FONT());
    FontId f2 = atlas.load_font(TEST_FONT());
    REQUIRE(f1 != 0);
    REQUIRE(f2 != 0);
    // Each load creates a new font entry
    CHECK(f1 != f2);
}

// ── TextShaper: alignment variations ────────────────────────────────────────

TEST_CASE("TextShaper: center alignment produces valid shaped text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 24.0f;
    style.alignment = TextAlignment::Center;

    ShapedText shaped = shaper->shape("Hello", style);
    CHECK(shaped.glyphs.size() > 0);
}

TEST_CASE("TextShaper: right alignment produces valid shaped text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 24.0f;
    style.alignment = TextAlignment::Right;

    ShapedText shaped = shaper->shape("Hello", style);
    CHECK(shaped.glyphs.size() > 0);
}

// ── TextShaper: FontWeight variations ───────────────────────────────────────

TEST_CASE("TextShaper: bold weight produces valid shaped text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 24.0f;
    style.weight = FontWeight::Bold;

    ShapedText shaped = shaper->shape("Hello", style);
    CHECK(shaped.glyphs.size() > 0);
}

TEST_CASE("TextShaper: light weight produces valid shaped text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 24.0f;
    style.weight = FontWeight::Light;

    ShapedText shaped = shaper->shape("Hello", style);
    CHECK(shaped.glyphs.size() > 0);
}

// ── TextShaper: lineHeight and letterSpacing ────────────────────────────────

TEST_CASE("TextShaper: custom lineHeight produces valid shaped text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 24.0f;
    style.lineHeight = 1.8f;

    ShapedText shaped = shaper->shape("Hello", style);
    CHECK(shaped.glyphs.size() > 0);
}

TEST_CASE("TextShaper: custom letterSpacing produces valid shaped text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 24.0f;
    style.letterSpacing = 2.0f;

    ShapedText shaped = shaper->shape("Hello", style);
    CHECK(shaped.glyphs.size() > 0);
}

// ── TextShaper: maxWidth ────────────────────────────────────────────────────

TEST_CASE("TextShaper: maxWidth parameter produces valid shaped text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 24.0f;

    ShapedText shaped = shaper->shape("Hello World", style, 100.0f);
    CHECK(shaped.glyphs.size() > 0);
}

TEST_CASE("TextShaper: zero maxWidth produces valid shaped text") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style{font, 24.0f};

    ShapedText shaped = shaper->shape("Hi", style, 0.0f);
    CHECK(shaped.glyphs.size() > 0);
}

// ── TextVisualDescriptor usage ──────────────────────────────────────────────

TEST_CASE("TextVisualDescriptor: can be constructed and used") {
    TextVisualDescriptor desc;
    desc.text = "Hello";
    desc.style.font = 0;
    desc.style.size = 16.0f;
    desc.style.weight = FontWeight::Regular;

    CHECK(desc.text == "Hello");
    CHECK(desc.style.size == doctest::Approx(16.0f));
    CHECK(desc.style.weight == FontWeight::Regular);
}

// ── Glyph mesh: zero-size atlasRect handled ─────────────────────────────────

TEST_CASE("generate_glyph_mesh: zero atlas rect produces valid quad") {
    GlyphPlacement gp;
    gp.glyph = 1;
    gp.position = {0.0f, 0.0f, 0.0f};
    gp.size = {10.0f, 10.0f, 0.0f};
    gp.atlasRect = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}; // zero

    auto mesh = generate_glyph_mesh(gp);
    // Should still produce a quad regardless of atlasRect
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);
}

// ── End-to-end: different font weight ──────────────────────────────────────

TEST_CASE("end-to-end: bold weight full pipeline") {
    FontAtlas atlas;
    FontId font = atlas.load_font(TEST_FONT());
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 32.0f;
    style.weight = FontWeight::Bold;

    ShapedText shaped = shaper->shape("Bold", style);
    REQUIRE(shaped.glyphs.size() > 0);

    for (auto& gp : shaped.glyphs) {
        gp.atlasRect = atlas.rasterize_glyph(font, gp.glyph, 32.0f);
    }

    auto mesh = generate_text_mesh(shaped, atlas);
    CHECK(mesh.vertices.size() > 0);
    CHECK(mesh.indices.size() > 0);
}

// ── Multi-font coverage ────────────────────────────────────────────────────

/// Helper: check if a font file exists on disk.
static bool font_exists(const std::string& path)
{
    return std::filesystem::exists(path);
}

/// Helper: collect fonts that are actually present on this system.
static std::vector<FontEntry> available_fonts()
{
    return build_inventory();
}

TEST_CASE("FontAtlas: multi-font — load and rasterize all available")
{
    auto fonts = available_fonts();
    INFO("Found ", fonts.size(), " fonts for coverage testing");
    REQUIRE(fonts.size() >= 3); // at least DejaVu Sans, Serif, Mono

    FontAtlas atlas(1024, 1024);

    for (const auto& fe : fonts) {
        CAPTURE(fe.path);
        CAPTURE(fe.category);

        FontId font = atlas.load_font(fe.path);
        REQUIRE_MESSAGE(font != 0, "Failed to load ", fe.path);

        // Shape a short Latin string
        auto shaper = create_harfbuzz_shaper(atlas);
        ShapedText shaped = shaper->shape("AbQ", TextStyle{font, 18.0f});
        REQUIRE_MESSAGE(shaped.glyphs.size() > 0, "No glyphs for ", fe.path);

        // Rasterize at least one glyph
        Bounds rect = atlas.rasterize_glyph(font, shaped.glyphs[0].glyph, 18.0f);
        CHECK_MESSAGE((rect.max.x - rect.min.x) > 0.0f,
                      "Zero-width glyph raster for ", fe.path);
    }
}

TEST_CASE("FontAtlas: mono fonts produce equal-width glyphs")
{
    std::string monoFonts[] = {
        font_path("DejaVuSansMono.ttf"),
        font_path("NotoSansMono-Regular.ttf"),
    };

    FontAtlas atlas;
    for (const auto& mf : monoFonts) {
        if (!font_exists(mf.c_str())) continue;

        CAPTURE(mf);
        FontId font = atlas.load_font(mf);
        REQUIRE(font != 0);

        auto shaper = create_harfbuzz_shaper(atlas);
        // Shape 'W' (wide) and 'i' (narrow) — should have same advance in mono
        ShapedText shaped = shaper->shape("Wi", TextStyle{font, 20.0f});
        REQUIRE(shaped.glyphs.size() >= 2);

        // Rasterize both
        atlas.rasterize_glyph(font, shaped.glyphs[0].glyph, 20.0f);
        atlas.rasterize_glyph(font, shaped.glyphs[1].glyph, 20.0f);

        float advanceW = 0, advanceI = 0;
        Vec3f sizeW, sizeI;
        atlas.get_glyph_metrics(font, shaped.glyphs[0].glyph, 20.0f, advanceW, sizeW);
        atlas.get_glyph_metrics(font, shaped.glyphs[1].glyph, 20.0f, advanceI, sizeI);

        // Advances should be nearly identical for a monospace font
        CHECK_MESSAGE(advanceW == doctest::Approx(advanceI).epsilon(0.02f),
                      "Non-mono advance for ", mf, ": W=", advanceW, " i=", advanceI);
    }
}

TEST_CASE("FontAtlas: bold font file yields different metrics than regular")
{
    std::string regularFont = font_path("DejaVuSans.ttf");
    std::string boldFont    = font_path("DejaVuSans-Bold.ttf");

    if (!font_exists(regularFont) || !font_exists(boldFont))
        return; // skip if not available

    FontAtlas atlas;
    FontId regId = atlas.load_font(regularFont);
    FontId boldId = atlas.load_font(boldFont);
    REQUIRE(regId != 0);
    REQUIRE(boldId != 0);

    auto shaper = create_harfbuzz_shaper(atlas);

    // Shape "M" with both fonts at same size
    ShapedText regShaped  = shaper->shape("M", TextStyle{regId, 24.0f});
    ShapedText boldShaped = shaper->shape("M", TextStyle{boldId, 24.0f});
    REQUIRE(regShaped.glyphs.size() > 0);
    REQUIRE(boldShaped.glyphs.size() > 0);

    // Rasterize
    atlas.rasterize_glyph(regId,  regShaped.glyphs[0].glyph, 24.0f);
    atlas.rasterize_glyph(boldId, boldShaped.glyphs[0].glyph, 24.0f);

    float regAdvance = 0, boldAdvance = 0;
    Vec3f regSize, boldSize;
    atlas.get_glyph_metrics(regId,  regShaped.glyphs[0].glyph,  24.0f, regAdvance,  regSize);
    atlas.get_glyph_metrics(boldId, boldShaped.glyphs[0].glyph, 24.0f, boldAdvance, boldSize);

    // Bold should have at least as wide an advance (and typically wider)
    CHECK(boldAdvance >= regAdvance);
}

TEST_CASE("FontAtlas: non-Latin font shapes Latin fallback text")
{
    // Noto Sans has broad Unicode coverage; it should still shape Latin text
    std::string notoSans = font_path("NotoSans-Regular.ttf");
    if (!font_exists(notoSans)) return;

    FontAtlas atlas;
    FontId font = atlas.load_font(notoSans);
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    ShapedText shaped = shaper->shape("Hello World", TextStyle{font, 20.0f});
    CHECK(shaped.glyphs.size() == 11); // 11 characters

    // Rasterize first glyph
    Bounds rect = atlas.rasterize_glyph(font, shaped.glyphs[0].glyph, 20.0f);
    CHECK((rect.max.x - rect.min.x) > 0.0f);
}

TEST_CASE("FontAtlas: all fonts survive multiple size rasterizations")
{
    auto fonts = available_fonts();
    REQUIRE(fonts.size() >= 2);

    FontAtlas atlas(2048, 2048);

    for (const auto& fe : fonts) {
        CAPTURE(fe.path);
        FontId font = atlas.load_font(fe.path);
        REQUIRE(font != 0);

        // Rasterize 'A' at several sizes
        for (float sz : {8.0f, 16.0f, 32.0f, 48.0f}) {
            // Use shaping to find the glyph index for 'A'
            auto shaper = create_harfbuzz_shaper(atlas);
            ShapedText shaped = shaper->shape("A", TextStyle{font, sz});
            if (shaped.glyphs.empty()) continue;
            Bounds rect = atlas.rasterize_glyph(font, shaped.glyphs[0].glyph, sz);
            CHECK_MESSAGE((rect.max.x - rect.min.x) > 0.0f,
                          "Zero-width glyph at size ", sz, " for ", fe.path);
            CHECK_MESSAGE((rect.max.y - rect.min.y) > 0.0f,
                          "Zero-height glyph at size ", sz, " for ", fe.path);
        }
    }
}

// ── Default font loading ───────────────────────────────────────────────────

/// Helper: register system font search paths so load_default() works in tests.
static void register_default_font_paths(FontAtlas& atlas)
{
    // Add common Linux font directories
    for (const char* dir : {"/usr/share/fonts/TTF",
                             "/usr/share/fonts/liberation",
                             "/usr/share/fonts/noto"}) {
        if (std::filesystem::exists(dir))
            atlas.add_font_search_path(dir);
    }
}

TEST_CASE("FontAtlas: load_default sans returns valid font") {
    FontAtlas atlas;
    register_default_font_paths(atlas);
    FontId font = atlas.load_default(DefaultFont::Sans);
    CHECK(font != 0);

    // Should be able to use it
    Bounds rect = atlas.rasterize_glyph(font, VALID_GLYPH_1, 24.0f);
    CHECK((rect.max.x - rect.min.x) > 0.0f);
}

TEST_CASE("FontAtlas: load_default serif returns valid font") {
    FontAtlas atlas;
    register_default_font_paths(atlas);
    FontId font = atlas.load_default(DefaultFont::Serif);
    CHECK(font != 0);
}

TEST_CASE("FontAtlas: load_default mono returns valid font") {
    FontAtlas atlas;
    register_default_font_paths(atlas);
    FontId font = atlas.load_default(DefaultFont::Mono);
    CHECK(font != 0);
}

TEST_CASE("FontAtlas: multiple default fonts coexist in same atlas") {
    FontAtlas atlas;
    register_default_font_paths(atlas);
    FontId sans  = atlas.load_default(DefaultFont::Sans);
    FontId serif = atlas.load_default(DefaultFont::Serif);
    FontId mono  = atlas.load_default(DefaultFont::Mono);
    REQUIRE(sans != 0);
    REQUIRE(serif != 0);
    REQUIRE(mono != 0);

    CHECK(sans != serif);
    CHECK(sans != mono);
    CHECK(serif != mono);

    auto shaper = create_harfbuzz_shaper(atlas);
    ShapedText s1 = shaper->shape("A", TextStyle{sans, 20.0f});
    ShapedText s2 = shaper->shape("A", TextStyle{serif, 20.0f});
    CHECK(!s1.glyphs.empty());
    CHECK(!s2.glyphs.empty());
}

// ── TextStyle color ────────────────────────────────────────────────────────

TEST_CASE("TextStyle: color defaults to white") {
    TextStyle style;
    CHECK(style.color.w == doctest::Approx(1.0f)); // R
    CHECK(style.color.x == doctest::Approx(1.0f)); // G
    CHECK(style.color.y == doctest::Approx(1.0f)); // B
    CHECK(style.color.z == doctest::Approx(1.0f)); // A
}

TEST_CASE("TextStyle: custom color propagates to glyph mesh vertices") {
    GlyphPlacement gp;
    gp.position = {0, 0, 0};
    gp.size = {16, 16, 0};
    gp.atlasRect = {{0, 0, 0}, {0.5f, 0.5f, 0}};

    Quat red{1.0f, 0.0f, 0.0f, 1.0f};

    auto mesh = generate_glyph_mesh(gp, red);
    REQUIRE(mesh.vertices.size() == 4);

    for (const auto& v : mesh.vertices) {
        CHECK(v.color.w == doctest::Approx(1.0f).epsilon(0.01f));
        CHECK(v.color.x == doctest::Approx(0.0f).epsilon(0.01f));
        CHECK(v.color.y == doctest::Approx(0.0f).epsilon(0.01f));
        CHECK(v.color.z == doctest::Approx(1.0f).epsilon(0.01f));
    }
}

TEST_CASE("TextStyle: default color arg produces white vertices") {
    GlyphPlacement gp;
    gp.position = {0, 0, 0};
    gp.size = {16, 16, 0};
    gp.atlasRect = {{0, 0, 0}, {0.5f, 0.5f, 0}};

    auto mesh = generate_glyph_mesh(gp);
    REQUIRE(mesh.vertices.size() == 4);
    for (const auto& v : mesh.vertices) {
        CHECK(v.color.w == doctest::Approx(1.0f).epsilon(0.01f));
        CHECK(v.color.x == doctest::Approx(1.0f).epsilon(0.01f));
    }
}

// ── Memory font loading ────────────────────────────────────────────────────

TEST_CASE("FontAtlas: load_font_memory with valid data") {
    std::ifstream file(TEST_FONT(), std::ios::binary | std::ios::ate);
    REQUIRE(file.good());
    size_t sz = file.tellg();
    file.seekg(0);
    std::vector<uint8_t> buffer(sz);
    file.read(reinterpret_cast<char*>(buffer.data()), sz);

    FontAtlas atlas;
    FontId font = atlas.load_font_memory(buffer.data(), buffer.size());
    CHECK(font != 0);

    Bounds rect = atlas.rasterize_glyph(font, VALID_GLYPH_1, 18.0f);
    CHECK((rect.max.x - rect.min.x) > 0.0f);
}

TEST_CASE("FontAtlas: load_font_memory null data returns 0") {
    FontAtlas atlas;
    CHECK(atlas.load_font_memory(nullptr, 100) == 0);
}

TEST_CASE("FontAtlas: load_font_memory zero size returns 0") {
    uint8_t dummy = 0;
    FontAtlas atlas;
    CHECK(atlas.load_font_memory(&dummy, 0) == 0);
}

// ── End-to-end: default font with color ────────────────────────────────────

TEST_CASE("end-to-end: default font with red color") {
    FontAtlas atlas;
    register_default_font_paths(atlas);
    FontId font = atlas.load_default(DefaultFont::Sans);
    REQUIRE(font != 0);

    auto shaper = create_harfbuzz_shaper(atlas);
    TextStyle style;
    style.font = font;
    style.size = 24.0f;
    style.color = {1.0f, 0.0f, 0.0f, 1.0f};

    ShapedText shaped = shaper->shape("Red", style);
    REQUIRE(shaped.glyphs.size() > 0);

    for (auto& gp : shaped.glyphs)
        gp.atlasRect = atlas.rasterize_glyph(font, gp.glyph, 24.0f);

    auto mesh = generate_text_mesh(shaped, atlas, style.color);
    REQUIRE(mesh.vertices.size() >= 4);

    for (const auto& v : mesh.vertices) {
        CHECK(v.color.w == doctest::Approx(1.0f).epsilon(0.01f));
        CHECK(v.color.x == doctest::Approx(0.0f).epsilon(0.01f));
    }
}

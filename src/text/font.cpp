#include <exd/geometry/font.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

#include <hb.h>
#include <hb-ft.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <unordered_map>
#include <vector>

namespace exd::geometry {
namespace {

// Inter-glyph gutter. GL_LINEAR sampling blends up to half a texel across a
// UV boundary, so a bare 1px gap lets a thin glyph pick up its neighbor's
// texel or the transparent gutter. With SDF packing the field already decays
// to far-outside (alpha 0) at the rect boundary, so the gutter only needs to
// absorb the 0.5-texel bilinear reach past the rect edge.
constexpr int kGlyphPadding = 2;

// A small opaque block reserved once at atlas construction, at a fixed
// location normal glyph packing never reaches (packing starts to its
// right). Solid-fill geometry that gets merged into an otherwise
// atlas-textured mesh — e.g. a math fraction bar sharing a draw call with
// its glyph quads — samples this instead of stretching UV (0,0)-(1,1)
// across whatever happens to be packed there, which is usually mostly
// transparent gutter and renders as an invisible hairline. Alpha 255 maps to
// "far inside" in the SDF shader, so the patch renders as solid fill.
constexpr int kSolidPatchSize = 4;

// ── Signed distance field helpers ──
//
// Exact squared-distance transform for 1D (Felzenszwalb & Huttenlocher,
// "Distance Transforms of Sampled Functions", 2006). O(n) per row/column.
// f[i] == 0 marks the "inside" sites; large finite values everywhere else.

// Large finite sentinel standing in for +inf in the F&H parabola envelope.
// Exact for distances << sqrt(kFar) (~1000 px); glyph margins are <= 8 px,
// and the field clamps to 0/1 well before any such distance.
constexpr float kFar = 1e6f;

void edt_1d(std::span<const float> f, std::span<float> d,
            std::vector<int>& v, std::vector<float>& z) {
    const int n = static_cast<int>(f.size());
    int k = 0;
    v[0] = 0;
    z[0] = -kFar;
    z[1] = kFar;
    for (int q = 1; q < n; ++q) {
        float s = ((f[q] + static_cast<float>(q) * q) -
                   (f[v[k]] + static_cast<float>(v[k]) * v[k])) /
                  (2.0f * q - 2.0f * v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q] + static_cast<float>(q) * q) -
                 (f[v[k]] + static_cast<float>(v[k]) * v[k])) /
                (2.0f * q - 2.0f * v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = kFar;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) ++k;
        const float diff = static_cast<float>(q - v[k]);
        d[q] = f[v[k]] + diff * diff;
    }
}

// Unsigned squared distance to the mask (mask==true marks inside sites).
// out holds the squared distances; sqrt() afterwards yields pixels.
void edt_2d(const std::vector<uint8_t>& mask, int w, int h,
            std::vector<float>& g) {
    std::vector<float> f(static_cast<size_t>(w) * h, kFar);
    std::vector<float> d(static_cast<size_t>(w) * h, 0.0f);
    std::vector<int> v;
    std::vector<float> z;

    // columns first (into f): work buffers sized to the COLUMN length h
    std::vector<float> col(h, kFar);
    v.resize(h);
    z.resize(h + 1);
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y)
            col[y] = mask[static_cast<size_t>(y) * w + x] ? 0.0f : kFar;
        edt_1d(col, d, v, z);
        for (int y = 0; y < h; ++y)
            f[static_cast<size_t>(y) * w + x] = d[y];
    }
    // then rows (into g): work buffers sized to the ROW length w
    std::vector<float> row(w, kFar);
    v.resize(w);
    z.resize(w + 1);
    for (int y = 0; y < h; ++y) {
        const float* fr = &f[static_cast<size_t>(y) * w];
        edt_1d(std::span<const float>(fr, w), row, v, z);
        for (int x = 0; x < w; ++x)
            g[static_cast<size_t>(y) * w + x] = row[x];
    }
    for (auto& val : g) val = std::sqrt(val);
}

// Signed distance field: negative inside the mask, positive outside, in
// pixels. Computed from the two unsigned transforms (to-ink and
// to-outside), which is exact and handles arbitrary mask topology.
void sdf_from_mask(const std::vector<uint8_t>& mask, int w, int h,
                   std::vector<float>& out) {
    std::vector<uint8_t> inv(mask.size());
    for (size_t i = 0; i < mask.size(); ++i) inv[i] = mask[i] ? 0 : 1;
    std::vector<float> din(mask.size()), dout(mask.size());
    edt_2d(mask, w, h, din);
    edt_2d(inv, w, h, dout);
    out.resize(mask.size());
    // inside: din == 0 (the pixel is in the ink) -> s = -dout < 0  (negative inside)
    // outside: dout == 0 -> s = +din > 0
    for (size_t i = 0; i < mask.size(); ++i)
        out[i] = din[i] - dout[i];
}

// Raster pixel size for a layout size: layout x sdfScale (FT floors).
float font_size_px(float fontSize, float sdfScale) {
    return fontSize * sdfScale;
}

} // namespace

// ── Cache key types ──

struct GlyphCacheKey {
    FontId  font;
    GlyphId glyph;
    float   fontSize;

    bool operator==(const GlyphCacheKey& o) const {
        return font == o.font && glyph == o.glyph && fontSize == o.fontSize;
    }
};

struct GlyphCacheKeyHash {
    size_t operator()(const GlyphCacheKey& k) const {
        size_t h = std::hash<uint64_t>{}(k.font);
        h ^= std::hash<uint32_t>{}(k.glyph) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<float>{}(k.fontSize) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

struct CachedMetrics {
    float advance;
    float width;
    float height;
    float bearingY;   // bbox yMin — negative for descenders (glyph extends below baseline)
};

// ── Impl ──

struct FontAtlas::Impl {
    FT_Library ftLibrary = nullptr;
    std::unordered_map<FontId, FT_Face> faces;
    FontId nextFontId = 1;

    // Atlas texture (RGBA8)
    std::vector<uint8_t> pixels;
    int width  = 512;
    int height = 512;
    int cursorX = 0;
    int cursorY = 0;
    int rowMaxHeight = 0;

    // SDF parameters (see FontAtlas class docs)
    float sdfScale = 1.0f;
    int   sdfMargin = 4;   // rasterized pixels

    // UV of the reserved opaque white patch — see kSolidPatchSize above.
    math::Vec3f solidUv{};

    // Glyph rect cache
    std::unordered_map<GlyphCacheKey, Bounds, GlyphCacheKeyHash> glyphCache;

    // Metrics cache
    std::unordered_map<GlyphCacheKey, CachedMetrics, GlyphCacheKeyHash> metricsCache;

    // Font search paths for load_default() — user-populated via add_font_search_path()
    std::vector<std::string> fontSearchPaths;

    Impl(int w, int h, float sdfScale_, int sdfMargin_)
        : width(w), height(h), pixels(static_cast<size_t>(w) * h * 4, 0),
          sdfScale(sdfScale_), sdfMargin(sdfMargin_)
    {
        FT_Error err = FT_Init_FreeType(&ftLibrary);
        if (err != FT_Err_Ok) {
            ftLibrary = nullptr;
        }

        // Reserve the solid white patch before any glyph packing starts, so
        // rasterize_glyph's cursor never overwrites it.
        const int patch = std::min({kSolidPatchSize, width, height});
        for (int y = 0; y < patch; ++y) {
            for (int x = 0; x < patch; ++x) {
                const int idx = (y * width + x) * 4;
                pixels[idx + 0] = 255;
                pixels[idx + 1] = 255;
                pixels[idx + 2] = 255;
                pixels[idx + 3] = 255;
            }
        }
        solidUv = {(patch * 0.5f) / static_cast<float>(width),
                   (patch * 0.5f) / static_cast<float>(height), 0.0f};
        cursorX = patch + kGlyphPadding;
        rowMaxHeight = patch;
    }

    ~Impl() {
        for (auto& [id, face] : faces) {
            if (face) FT_Done_Face(face);
        }
        if (ftLibrary) FT_Done_FreeType(ftLibrary);
    }

    Impl(Impl&& other) noexcept
        : ftLibrary(other.ftLibrary),
          faces(std::move(other.faces)),
          nextFontId(other.nextFontId),
          pixels(std::move(other.pixels)),
          width(other.width),
          height(other.height),
          cursorX(other.cursorX),
          cursorY(other.cursorY),
          rowMaxHeight(other.rowMaxHeight),
          sdfScale(other.sdfScale),
          sdfMargin(other.sdfMargin),
          solidUv(other.solidUv),
          glyphCache(std::move(other.glyphCache)),
          metricsCache(std::move(other.metricsCache)),
          fontSearchPaths(std::move(other.fontSearchPaths))
    {
        other.ftLibrary = nullptr;
        other.width = 0;
        other.height = 0;
        other.cursorX = 0;
        other.cursorY = 0;
        other.rowMaxHeight = 0;
        other.nextFontId = 1;
    }

    Impl& operator=(Impl&& other) noexcept {
        if (this != &other) {
            // Clean up current resources
            for (auto& [id, face] : faces) {
                if (face) FT_Done_Face(face);
            }
            if (ftLibrary) FT_Done_FreeType(ftLibrary);

            ftLibrary = other.ftLibrary;
            faces = std::move(other.faces);
            nextFontId = other.nextFontId;
            pixels = std::move(other.pixels);
            width = other.width;
            height = other.height;
            cursorX = other.cursorX;
            cursorY = other.cursorY;
            rowMaxHeight = other.rowMaxHeight;
            sdfScale = other.sdfScale;
            sdfMargin = other.sdfMargin;
            solidUv = other.solidUv;
            glyphCache = std::move(other.glyphCache);
            metricsCache = std::move(other.metricsCache);
            fontSearchPaths = std::move(other.fontSearchPaths);

            other.ftLibrary = nullptr;
            other.width = 0;
            other.height = 0;
            other.cursorX = 0;
            other.cursorY = 0;
            other.rowMaxHeight = 0;
            other.nextFontId = 1;
        }
        return *this;
    }
};

// ── FontAtlas ──

FontAtlas::FontAtlas(int atlasWidth, int atlasHeight,
                       float sdfScale, int sdfMargin)
    // Clamp: sdfScale < 1 would rasterize below layout resolution (defeating
    // minification headroom); sdfMargin < 1 would remove the outside field
    // band and produce a degenerate (NaN/invisible) field.
    : impl_(std::make_unique<Impl>(atlasWidth, atlasHeight,
                                   std::max(sdfScale, 1.0f),
                                   std::max(sdfMargin, 1)))
{
    // Initialize default font paths including bundled fonts
    initialize_default_paths();
}

FontAtlas::~FontAtlas() = default;

FontAtlas::FontAtlas(FontAtlas&&) noexcept = default;
FontAtlas& FontAtlas::operator=(FontAtlas&&) noexcept = default;

FontId FontAtlas::load_font(const std::string& path, int faceIndex) {
    if (!impl_ || !impl_->ftLibrary) return 0;

    FT_Face face = nullptr;
    FT_Error err = FT_New_Face(impl_->ftLibrary, path.c_str(), faceIndex, &face);
    if (err != FT_Err_Ok || !face) return 0;

    FontId id = impl_->nextFontId++;
    impl_->faces[id] = face;
    return id;
}

FontId FontAtlas::load_font_memory(const uint8_t* data, size_t size, int faceIndex) {
    if (!impl_ || !impl_->ftLibrary || !data || size == 0) return 0;

    FT_Face face = nullptr;
    FT_Error err = FT_New_Memory_Face(impl_->ftLibrary,
                                      static_cast<const FT_Byte*>(data),
                                      static_cast<FT_Long>(size),
                                      faceIndex, &face);
    if (err != FT_Err_Ok || !face) return 0;

    FontId id = impl_->nextFontId++;
    impl_->faces[id] = face;
    return id;
}

FontId FontAtlas::load_default(DefaultFont which) {
    if (!impl_) return 0;

    // File name patterns to search for each default font category
    struct Pattern { DefaultFont kind; const char* name; };
    static const Pattern kPatterns[] = {
        {DefaultFont::Sans,  "Inter-Regular.ttf"},
        {DefaultFont::Sans,  "DejaVuSans.ttf"},
        {DefaultFont::Sans,  "LiberationSans-Regular.ttf"},
        {DefaultFont::Sans,  "NotoSans-Regular.ttf"},
        {DefaultFont::Serif, "DejaVuSerif.ttf"},
        {DefaultFont::Serif, "LiberationSerif-Regular.ttf"},
        {DefaultFont::Serif, "NotoSerif-Regular.ttf"},
        {DefaultFont::Mono,  "DejaVuSansMono.ttf"},
        {DefaultFont::Mono,  "LiberationMono-Regular.ttf"},
        {DefaultFont::Mono,  "NotoSansMono-Regular.ttf"},
    };

    for (const auto& dir : impl_->fontSearchPaths) {
        for (const auto& pat : kPatterns) {
            if (pat.kind != which) continue;
            std::string path = dir + "/" + pat.name;
            if (std::filesystem::exists(path)) {
                FontId id = load_font(path, 0);
                if (id != 0) return id;
            }
        }
    }

    return 0;
}

void FontAtlas::add_font_search_path(const std::string& directory) {
    if (impl_)
        impl_->fontSearchPaths.push_back(directory);
}

Bounds FontAtlas::rasterize_glyph(FontId font, GlyphId glyph, float fontSize) {
    Bounds zeroRect{{0, 0, 0}, {0, 0, 0}};

    if (!impl_ || !impl_->ftLibrary) return zeroRect;

    // Check cache first
    GlyphCacheKey key{font, glyph, fontSize};
    auto cacheIt = impl_->glyphCache.find(key);
    if (cacheIt != impl_->glyphCache.end()) {
        return cacheIt->second;
    }

    // Get FT_Face
    auto faceIt = impl_->faces.find(font);
    if (faceIt == impl_->faces.end()) return zeroRect;

    FT_Face face = faceIt->second;

    // Set pixel size: an EXACT multiple of the layout size used by shaping
    // and metrics (static_cast<FT_UInt> floors). Using floor(fs) * sdfScale
    // — instead of floor(fs * sdfScale) — keeps the quad<->uv ratio exactly
    // sdfScale for fractional font sizes (e.g. 12.5 -> 12*2 = 24, not
    // floor(25)=25 which would inflate the ink ~4-6% against 12px metrics).
    const FT_UInt baseSize = static_cast<FT_UInt>(fontSize);
    const FT_UInt rasterSize = baseSize > 0 ? static_cast<FT_UInt>(baseSize * impl_->sdfScale) : 0;
    if (rasterSize == 0) return zeroRect;
    FT_Error err = FT_Set_Pixel_Sizes(face, 0, rasterSize);
    if (err != FT_Err_Ok) return zeroRect;

    // Load glyph (same flags as metric lookup, so bitmap and metrics agree)
    err = FT_Load_Glyph(face, glyph, FT_LOAD_DEFAULT);
    if (err != FT_Err_Ok) return zeroRect;

    // Render glyph coverage at the raster size; the SDF is derived from it
    err = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL);
    if (err != FT_Err_Ok) return zeroRect;

    FT_Bitmap& bitmap = face->glyph->bitmap;
    if (bitmap.width == 0 || bitmap.rows == 0) return zeroRect;

    int bw = static_cast<int>(bitmap.width);
    int bh = static_cast<int>(bitmap.rows);
    const int M = std::max(impl_->sdfMargin, 0);
    const int wp = bw + 2 * M;   // padded rect: ink + margin band
    const int hp = bh + 2 * M;

    // Check if glyph fits on current row
    if (impl_->cursorX + wp > impl_->width) {
        // Advance to next row
        impl_->cursorY += impl_->rowMaxHeight + kGlyphPadding;
        impl_->cursorX = 0;
        impl_->rowMaxHeight = 0;
    }

    // Check if glyph fits vertically
    if (impl_->cursorY + hp > impl_->height) {
        // Atlas is full - return zero rect for v1
        return zeroRect;
    }

    // Build the inside-mask over the padded rect (ink at (M, M)) from the
    // rendered coverage, then convert to a signed distance field.
    std::vector<uint8_t> mask(static_cast<size_t>(wp) * hp, 0);
    const int pitch = static_cast<int>(bitmap.pitch);
    for (int y = 0; y < bh; ++y) {
        for (int x = 0; x < bw; ++x) {
            const uint8_t cov = bitmap.buffer[static_cast<size_t>(y) * pitch + x];
            if (cov >= 128)
                mask[static_cast<size_t>(y + M) * wp + (x + M)] = 1;
        }
    }
    std::vector<float> sdf;
    sdf_from_mask(mask, wp, hp, sdf);

    // Encode the field into the alpha channel:
    //   alpha = clamp(0.5 - signed_dist / (2 * margin), 0, 1)
    // so the outline sits at 0.5, far-inside at 1, far-outside at 0. The
    // shader reconstructs the edge with fwidth-based smoothing, yielding
    // sub-pixel crisp glyphs at any render scale. RGB stays opaque white.
    const float inv2M = 1.0f / (2.0f * static_cast<float>(M));
    for (int y = 0; y < hp; ++y) {
        for (int x = 0; x < wp; ++x) {
            const float dist = sdf[static_cast<size_t>(y) * wp + x];
            const float a = std::clamp(0.5f - dist * inv2M, 0.0f, 1.0f);
            const int dstIdx = ((impl_->cursorY + y) * impl_->width +
                                (impl_->cursorX + x)) * 4;
            impl_->pixels[dstIdx + 0] = 255; // R
            impl_->pixels[dstIdx + 1] = 255; // G
            impl_->pixels[dstIdx + 2] = 255; // B
            impl_->pixels[dstIdx + 3] = static_cast<uint8_t>(a * 255.0f + 0.5f); // A
        }
    }

    // Compute normalized UV rect over the PADDED region (the ink sits at
    // the rect center; quad generation must map quads to this full rect,
    // see sdf_margin_layout()). kGlyphPadding keeps the 0.5-texel bilinear
    // reach past the rect boundary safely inside the gutter.
    Bounds rect;
    rect.min.x = static_cast<float>(impl_->cursorX) / static_cast<float>(impl_->width);
    rect.min.y = static_cast<float>(impl_->cursorY) / static_cast<float>(impl_->height);
    rect.max.x = static_cast<float>(impl_->cursorX + wp) / static_cast<float>(impl_->width);
    rect.max.y = static_cast<float>(impl_->cursorY + hp) / static_cast<float>(impl_->height);
    rect.min.z = 0.0f;
    rect.max.z = 0.0f;

    // Update cursor
    impl_->cursorX += wp + kGlyphPadding;
    impl_->rowMaxHeight = std::max(impl_->rowMaxHeight, hp);

    // Cache metrics (in LAYOUT space — the cache is shared with
    // get_glyph_metrics, which reports layout-scale values; the raster slot
    // is sdfScale larger). Only seed when shaping has not populated the
    // entry already: measurement and rendering must observe one stable
    // metric set, and the rounded bitmap dims are an approximation.
    float advance = static_cast<float>(face->glyph->advance.x) / 64.0f;
    float bearingY = 0.0f;
    {
        FT_BBox bbox;
        FT_Outline_Get_CBox(&face->glyph->outline, &bbox);
        bearingY = static_cast<float>(bbox.yMin) / 64.0f;
    }
    CachedMetrics metrics{
        advance / impl_->sdfScale,
        static_cast<float>(bw) / impl_->sdfScale,
        static_cast<float>(bh) / impl_->sdfScale,
        bearingY / impl_->sdfScale
    };
    impl_->metricsCache.try_emplace(key, metrics);

    // Cache rect
    impl_->glyphCache[key] = rect;

    return rect;
}

bool FontAtlas::get_glyph_rect(FontId font, GlyphId glyph, float fontSize, Bounds& outRect) const {
    if (!impl_) return false;

    GlyphCacheKey key{font, glyph, fontSize};
    auto it = impl_->glyphCache.find(key);
    if (it == impl_->glyphCache.end()) return false;

    outRect = it->second;
    return true;
}

std::span<const uint8_t> FontAtlas::atlas_data() const {
    if (!impl_) return {};
    return std::span<const uint8_t>(impl_->pixels.data(), impl_->pixels.size());
}

int FontAtlas::atlas_width() const {
    return impl_ ? impl_->width : 0;
}

int FontAtlas::atlas_height() const {
    return impl_ ? impl_->height : 0;
}

bool FontAtlas::get_glyph_metrics(FontId font, GlyphId glyph, float fontSize,
                                   float& outAdvance, math::Vec3f& outSize) const {
    if (!impl_ || !impl_->ftLibrary) return false;

    // Check metrics cache first
    GlyphCacheKey key{font, glyph, fontSize};
    auto cacheIt = impl_->metricsCache.find(key);
    if (cacheIt != impl_->metricsCache.end()) {
        outAdvance = cacheIt->second.advance;
        outSize = {cacheIt->second.width, cacheIt->second.height, cacheIt->second.bearingY};
        return true;
    }

    // Get FT_Face
    auto faceIt = impl_->faces.find(font);
    if (faceIt == impl_->faces.end()) return false;

    FT_Face face = faceIt->second;

    // Temporarily set pixel size and load glyph
    FT_Error err = FT_Set_Pixel_Sizes(face, 0, static_cast<FT_UInt>(fontSize));
    if (err != FT_Err_Ok) return false;

    // Must match rasterize_glyph()'s load flags so metrics and the
    // rendered bitmap agree on the same outline.
    err = FT_Load_Glyph(face, glyph, FT_LOAD_DEFAULT);
    if (err != FT_Err_Ok) return false;

    FT_GlyphSlot slot = face->glyph;
    outAdvance = static_cast<float>(slot->advance.x) / 64.0f;

    // Get dimensions from the glyph metrics (in pixels)
    // The metrics are in 26.6 fixed point format
    FT_BBox bbox;
    FT_Outline_Get_CBox(&slot->outline, &bbox);
    float w = static_cast<float>(bbox.xMax - bbox.xMin) / 64.0f;
    float h = static_cast<float>(bbox.yMax - bbox.yMin) / 64.0f;
    float bearingY = static_cast<float>(bbox.yMin) / 64.0f;  // negative for descenders

    // If outline is empty (e.g., bitmap-only glyphs), try bitmap dimensions
    if (w <= 0.0f || h <= 0.0f) {
        // Render to get bitmap dimensions
        err = FT_Render_Glyph(slot, FT_RENDER_MODE_NORMAL);
        if (err == FT_Err_Ok) {
            w = static_cast<float>(slot->bitmap.width);
            h = static_cast<float>(slot->bitmap.rows);
            // bitmap_top is the offset from baseline to top of bitmap;
            // bearing = -(bitmap_top - rows) = rows - bitmap_top
            bearingY = static_cast<float>(slot->bitmap.rows - slot->bitmap_top);
        }
    }

    outSize = {w, h, bearingY};

    // Cache metrics
    impl_->metricsCache[key] = {outAdvance, w, h, bearingY};

    return true;
}

void* FontAtlas::create_hb_font(FontId font, float fontSize) const {
    if (!impl_) return nullptr;

    auto it = impl_->faces.find(font);
    if (it == impl_->faces.end()) return nullptr;

    // Shape at the SAME pixel size the rasterizer uses (static_cast<FT_UInt>
    // floors). Shaping at ceil() while rasterizing at floor() inflates every
    // inter-glyph advance relative to the ink, which reads as visible gaps
    // between letters (e.g. "flow" -> "fl ow").
    FT_Set_Pixel_Sizes(it->second, 0, static_cast<FT_UInt>(fontSize));
    return hb_ft_font_create_referenced(it->second);
}

math::Vec3f FontAtlas::solid_uv() const {
    return impl_ ? impl_->solidUv : math::Vec3f{};
}

float FontAtlas::sdf_scale() const {
    return impl_ ? impl_->sdfScale : 1.0f;
}

int FontAtlas::sdf_margin() const {
    return impl_ ? impl_->sdfMargin : 0;
}

float FontAtlas::sdf_margin_layout() const {
    if (!impl_ || impl_->sdfScale <= 0.0f) return 0.0f;
    // margin is specified in raster pixels; in layout units it shrinks by
    // the raster scale (margin/scale), i.e. equals the margin in device
    // pixels when scale == 1.
    return static_cast<float>(impl_->sdfMargin) / impl_->sdfScale;
}

void FontAtlas::initialize_default_paths() {
    // Add our bundled font search path
    add_font_search_path("assets/fonts/");
}

} // namespace exd::geometry

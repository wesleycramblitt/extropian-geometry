#include <exd/geometry/font.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

#include <hb.h>
#include <hb-ft.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <unordered_map>
#include <vector>

namespace exd::geometry {

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

    // Glyph rect cache
    std::unordered_map<GlyphCacheKey, Bounds, GlyphCacheKeyHash> glyphCache;

    // Metrics cache
    std::unordered_map<GlyphCacheKey, CachedMetrics, GlyphCacheKeyHash> metricsCache;

    // Font search paths for load_default() — user-populated via add_font_search_path()
    std::vector<std::string> fontSearchPaths;

    Impl(int w, int h)
        : width(w), height(h), pixels(static_cast<size_t>(w) * h * 4, 0)
    {
        FT_Error err = FT_Init_FreeType(&ftLibrary);
        if (err != FT_Err_Ok) {
            ftLibrary = nullptr;
        }
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

FontAtlas::FontAtlas(int atlasWidth, int atlasHeight)
    : impl_(std::make_unique<Impl>(atlasWidth, atlasHeight))
{
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

    // Set pixel size
    FT_Error err = FT_Set_Pixel_Sizes(face, 0, static_cast<FT_UInt>(fontSize));
    if (err != FT_Err_Ok) return zeroRect;

    // Load glyph
    err = FT_Load_Glyph(face, glyph, FT_LOAD_DEFAULT);
    if (err != FT_Err_Ok) return zeroRect;

    // Render glyph
    err = FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL);
    if (err != FT_Err_Ok) return zeroRect;

    FT_Bitmap& bitmap = face->glyph->bitmap;
    if (bitmap.width == 0 || bitmap.rows == 0) return zeroRect;

    int bw = static_cast<int>(bitmap.width);
    int bh = static_cast<int>(bitmap.rows);

    // Check if glyph fits on current row
    if (impl_->cursorX + bw > impl_->width) {
        // Advance to next row
        impl_->cursorY += impl_->rowMaxHeight;
        impl_->cursorX = 0;
        impl_->rowMaxHeight = 0;
    }

    // Check if glyph fits vertically
    if (impl_->cursorY + bh > impl_->height) {
        // Atlas is full - return zero rect for v1
        return zeroRect;
    }

    // Copy bitmap into atlas (grayscale -> RGBA with white color)
    int pitch = static_cast<int>(bitmap.pitch);
    for (int y = 0; y < bh; ++y) {
        for (int x = 0; x < bw; ++x) {
            int srcIdx = y * pitch + x;
            int dstIdx = ((impl_->cursorY + y) * impl_->width + (impl_->cursorX + x)) * 4;
            uint8_t alpha = bitmap.buffer[srcIdx];
            impl_->pixels[dstIdx + 0] = 255; // R
            impl_->pixels[dstIdx + 1] = 255; // G
            impl_->pixels[dstIdx + 2] = 255; // B
            impl_->pixels[dstIdx + 3] = alpha; // A
        }
    }

    // Compute normalized UV rect
    Bounds rect;
    rect.min.x = static_cast<float>(impl_->cursorX) / static_cast<float>(impl_->width);
    rect.min.y = static_cast<float>(impl_->cursorY) / static_cast<float>(impl_->height);
    rect.max.x = static_cast<float>(impl_->cursorX + bw) / static_cast<float>(impl_->width);
    rect.max.y = static_cast<float>(impl_->cursorY + bh) / static_cast<float>(impl_->height);
    rect.min.z = 0.0f;
    rect.max.z = 0.0f;

    // Update cursor
    impl_->cursorX += bw + 1; // 1px padding
    impl_->rowMaxHeight = std::max(impl_->rowMaxHeight, bh);

    // Cache metrics
    float advance = static_cast<float>(face->glyph->advance.x) / 64.0f;
    float bearingY = 0.0f;
    {
        FT_BBox bbox;
        FT_Outline_Get_CBox(&face->glyph->outline, &bbox);
        bearingY = static_cast<float>(bbox.yMin) / 64.0f;
    }
    CachedMetrics metrics{
        advance,
        static_cast<float>(bw),
        static_cast<float>(bh),
        bearingY
    };
    impl_->metricsCache[key] = metrics;

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

void* FontAtlas::create_hb_font(FontId font) const {
    if (!impl_) return nullptr;

    auto it = impl_->faces.find(font);
    if (it == impl_->faces.end()) return nullptr;

    return hb_ft_font_create_referenced(it->second);
}

} // namespace exd::geometry

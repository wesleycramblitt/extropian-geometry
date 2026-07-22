#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace exd::geometry
{

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

// ── Font / glyph identity ──

using FontId  = uint64_t;
using GlyphId = uint32_t;

// ── Text style ──

struct TextStyle
{
    FontId        font = 0;
    float         size = 16.0f;
    FontWeight    weight = FontWeight::Regular;
    TextAlignment alignment = TextAlignment::Left;
    float         lineHeight = 1.2f;
    float         letterSpacing = 0.0f;
};

// ── Glyph placement ──

struct GlyphPlacement
{
    GlyphId     glyph = 0;
    math::Vec3f position = {0.0f, 0.0f, 0.0f};
    math::Vec3f size     = {0.0f, 0.0f, 0.0f};
    Bounds      atlasRect; // UV rectangle in glyph atlas
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

} // namespace exd::geometry

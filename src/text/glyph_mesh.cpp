#include <exd/geometry/text.hpp>
#include <exd/geometry/font.hpp>
#include <exd/geometry/mesh_builder.hpp>

namespace exd::geometry {

MeshData generate_glyph_mesh(const GlyphPlacement& glyph, math::Quat color) {
    MeshBuilder builder;
    builder.reserve(4, 6);

    float x = glyph.position.x;
    float y = glyph.position.y + glyph.size.z;  // offset by descender bearing
    float w = glyph.size.x;
    float h = glyph.size.y;

    float u0 = glyph.atlasRect.min.x;
    float v0 = glyph.atlasRect.min.y;
    float u1 = glyph.atlasRect.max.x;
    float v1 = glyph.atlasRect.max.y;

    Vertex va, vb, vc, vd;
    va.position = {x,     y,     0.0f}; va.uv = {u0, v1, 0.0f}; va.color = color;
    vb.position = {x + w, y,     0.0f}; vb.uv = {u1, v1, 0.0f}; vb.color = color;
    vc.position = {x + w, y + h, 0.0f}; vc.uv = {u1, v0, 0.0f}; vc.color = color;
    vd.position = {x,     y + h, 0.0f}; vd.uv = {u0, v0, 0.0f}; vd.color = color;

    va.normal = vb.normal = vc.normal = vd.normal = {0.0f, 0.0f, 1.0f};

    auto i0 = builder.add_vertex(va);
    auto i1 = builder.add_vertex(vb);
    auto i2 = builder.add_vertex(vc);
    auto i3 = builder.add_vertex(vd);

    builder.add_triangle(i0, i1, i2);
    builder.add_triangle(i0, i2, i3);

    return builder.build();
}

MeshData generate_text_mesh(const ShapedText& shaped, FontAtlas& atlas, math::Quat color) {
    MeshBuilder builder;

    for (const auto& gp : shaped.glyphs) {
        Bounds rect = gp.atlasRect;

        // If atlasRect is zero (not yet rasterized), rasterize it now
        if (rect.min.x == 0.0f && rect.max.x == 0.0f &&
            rect.min.y == 0.0f && rect.max.y == 0.0f) {
            // We need font and size info, but GlyphPlacement doesn't carry it.
            // For v1, skip glyphs with zero atlas rect.
            // Callers should pre-rasterize via FontAtlas::rasterize_glyph.
            continue;
        }

        // gp.position.y is the baseline (0 for horizontal text).
        // gp.size.z is the yMin bearing from the bbox (negative for descenders).
        // Offset the quad downward by the bearing so the glyph content
        // (including descenders) is fully covered.
        float x = gp.position.x;
        float y = gp.position.y + gp.size.z;
        float w = gp.size.x;
        float h = gp.size.y;

        float u0 = gp.atlasRect.min.x;
        float v0 = gp.atlasRect.min.y;
        float u1 = gp.atlasRect.max.x;
        float v1 = gp.atlasRect.max.y;

        // HarfBuzz places glyphs with Y-up convention (ascenders +Y, descenders -Y).
        // The atlas stores glyphs with v increasing downward (standard texture coords).
        // We map atlas bottom (v1) to glyph bottom (y) and atlas top (v0) to glyph top (y+h),
        // so the glyph renders right-side-up without needing a Y-axis flip in the world transform.
        const auto glyphColor = gp.hasColorOverride ? gp.color : color;
        Vertex va, vb, vc, vd;
        va.position = {x,     y,     0.0f}; va.uv = {u0, v1, 0.0f}; va.color = glyphColor;
        vb.position = {x + w, y,     0.0f}; vb.uv = {u1, v1, 0.0f}; vb.color = glyphColor;
        vc.position = {x + w, y + h, 0.0f}; vc.uv = {u1, v0, 0.0f}; vc.color = glyphColor;
        vd.position = {x,     y + h, 0.0f}; vd.uv = {u0, v0, 0.0f}; vd.color = glyphColor;

        va.normal = vb.normal = vc.normal = vd.normal = {0.0f, 0.0f, 1.0f};

        auto i0 = builder.add_vertex(va);
        auto i1 = builder.add_vertex(vb);
        auto i2 = builder.add_vertex(vc);
        auto i3 = builder.add_vertex(vd);

        builder.add_triangle(i0, i1, i2);
        builder.add_triangle(i0, i2, i3);
    }

    return builder.build(PrimitiveTopology::Triangles);
}

} // namespace exd::geometry

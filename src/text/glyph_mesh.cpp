#include <exd/geometry/text.hpp>
#include <exd/geometry/font.hpp>
#include <exd/geometry/mesh_builder.hpp>

namespace exd::geometry {

MeshData generate_glyph_mesh(const GlyphPlacement& glyph) {
    MeshBuilder builder;
    builder.reserve(4, 6);

    float x = glyph.position.x;
    float y = glyph.position.y;
    float w = glyph.size.x;
    float h = glyph.size.y;

    float u0 = glyph.atlasRect.min.x;
    float v0 = glyph.atlasRect.min.y;
    float u1 = glyph.atlasRect.max.x;
    float v1 = glyph.atlasRect.max.y;

    Vertex va, vb, vc, vd;
    va.position = {x,     y,     0.0f}; va.uv = {u0, v0, 0.0f};
    vb.position = {x + w, y,     0.0f}; vb.uv = {u1, v0, 0.0f};
    vc.position = {x + w, y + h, 0.0f}; vc.uv = {u1, v1, 0.0f};
    vd.position = {x,     y + h, 0.0f}; vd.uv = {u0, v1, 0.0f};

    va.normal = vb.normal = vc.normal = vd.normal = {0.0f, 0.0f, 1.0f};

    auto i0 = builder.add_vertex(va);
    auto i1 = builder.add_vertex(vb);
    auto i2 = builder.add_vertex(vc);
    auto i3 = builder.add_vertex(vd);

    builder.add_triangle(i0, i1, i2);
    builder.add_triangle(i0, i2, i3);

    return builder.build();
}

MeshData generate_text_mesh(const ShapedText& shaped, FontAtlas& atlas) {
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

        float x = gp.position.x;
        float y = gp.position.y;
        float w = gp.size.x;
        float h = gp.size.y;

        float u0 = rect.min.x;
        float v0 = rect.min.y;
        float u1 = rect.max.x;
        float v1 = rect.max.y;

        Vertex va, vb, vc, vd;
        va.position = {x,     y,     0.0f}; va.uv = {u0, v0, 0.0f};
        vb.position = {x + w, y,     0.0f}; vb.uv = {u1, v0, 0.0f};
        vc.position = {x + w, y + h, 0.0f}; vc.uv = {u1, v1, 0.0f};
        vd.position = {x,     y + h, 0.0f}; vd.uv = {u0, v1, 0.0f};

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

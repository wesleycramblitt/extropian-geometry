#include <exd/geometry/primitives2d.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>

namespace exd::geometry
{

MeshData generate_arrow_mesh(const ArrowGeometry& geom)
{
    const math::Vec3f dir = geom.end - geom.start;
    const float len = dir.length();

    if (len < 1e-6f)
    {
        // Degenerate arrow — return empty mesh
        return {};
    }

    const math::Vec3f forward = dir / len;
    // Perpendicular in XY plane: rotate forward by +90° around Z
    const math::Vec3f right = {-forward.y, forward.x, 0.0f};

    // Clamp headLength to not exceed total length
    const float headLen = std::min(geom.headLength, len);
    const float shaftLen = len - headLen;

    // Compute key positions along the arrow
    const math::Vec3f shaftEnd = geom.start + forward * shaftLen;
    const math::Vec3f headBaseLeft  = shaftEnd - right * (geom.headWidth * 0.5f);
    const math::Vec3f headBaseRight = shaftEnd + right * (geom.headWidth * 0.5f);
    const float shaftHW = geom.shaftWidth * 0.5f;

    // Reserve: shaft = 4 verts + 6 idx, head = 3 verts + 3 idx
    MeshBuilder builder;
    builder.reserve(7, 9);

    // ── Shaft: rectangle from start to shaftEnd ──
    Vertex sv0, sv1, sv2, sv3;

    sv0.position = geom.start - right * shaftHW;
    sv1.position = geom.start + right * shaftHW;
    sv2.position = shaftEnd   + right * shaftHW;
    sv3.position = shaftEnd   - right * shaftHW;

    sv0.normal = {0.0f, 0.0f, 1.0f};
    sv1.normal = {0.0f, 0.0f, 1.0f};
    sv2.normal = {0.0f, 0.0f, 1.0f};
    sv3.normal = {0.0f, 0.0f, 1.0f};

    // UVs: u along arrow (0 at start, 1 at tip), v across
    sv0.uv = {0.0f, 0.0f, 0.0f};
    sv1.uv = {0.0f, 1.0f, 0.0f};
    sv2.uv = {shaftLen / len, 1.0f, 0.0f};
    sv3.uv = {shaftLen / len, 0.0f, 0.0f};

    sv0.color = geom.color;
    sv1.color = geom.color;
    sv2.color = geom.color;
    sv3.color = geom.color;

    const auto sa = builder.add_vertex(sv0);
    const auto sb = builder.add_vertex(sv1);
    const auto sc = builder.add_vertex(sv2);
    const auto sd = builder.add_vertex(sv3);

    builder.add_triangle(sa, sb, sc);
    builder.add_triangle(sa, sc, sd);

    // ── Head: triangle from shaft end to arrow tip ──
    Vertex hv0, hv1, hv2;

    hv0.position = geom.end;              // tip
    hv1.position = headBaseLeft;           // base-left
    hv2.position = headBaseRight;          // base-right

    hv0.normal = {0.0f, 0.0f, 1.0f};
    hv1.normal = {0.0f, 0.0f, 1.0f};
    hv2.normal = {0.0f, 0.0f, 1.0f};

    // UVs: u=1 at tip, u=shaftLen/len at base
    hv0.uv = {1.0f, 0.5f, 0.0f};
    hv1.uv = {shaftLen / len, 0.0f, 0.0f};
    hv2.uv = {shaftLen / len, 1.0f, 0.0f};

    hv0.color = geom.color;
    hv1.color = geom.color;
    hv2.color = geom.color;

    const auto ha = builder.add_vertex(hv0);
    const auto hb = builder.add_vertex(hv1);
    const auto hc = builder.add_vertex(hv2);

    // CCW winding from Z+: tip, base-right, base-left
    builder.add_triangle(ha, hc, hb);

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
}

} // namespace exd::geometry

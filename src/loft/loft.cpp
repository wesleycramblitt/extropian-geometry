#include <exd/geometry/loft.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace exd::geometry
{

namespace
{

math::Vec3f ring_centroid(const std::vector<math::Vec3f>& ring)
{
    math::Vec3f c{0.0f, 0.0f, 0.0f};
    if (ring.empty())
        return c;
    for (const auto& p : ring)
        c += p;
    return c / static_cast<float>(ring.size());
}

} // namespace

MeshData generate_loft_mesh(const LoftGeometry& geometry)
{
    const auto& sections = geometry.sections;

    // ── Guards: count, sizes, point count, spine separation ──
    if (sections.size() < 2)
        return {};
    const size_t n = sections[0].size();
    if (n < 3)
        return {};
    for (const auto& sec : sections)
        if (sec.size() != n)
            return {};

    const size_t ns = sections.size();
    for (size_t k = 0; k + 1 < ns; ++k)
    {
        const math::Vec3f dir = ring_centroid(sections[k + 1]) - ring_centroid(sections[k]);
        if (dir.length() <= 1e-6f)
            return {};   // coincident consecutive sections
    }

    // ── Smooth skin normals: accumulate incident-quad face normals ──
    auto quad_normal = [](const math::Vec3f& a, const math::Vec3f& b,
                          const math::Vec3f& c, const math::Vec3f& d) -> math::Vec3f
    {
        const math::Vec3f f = (c - a).cross(d - b);   // diagonals AC × BD
        const float       l = f.length();
        return l > 1e-8f ? f / l : math::Vec3f{0.0f, 1.0f, 0.0f};
    };

    std::vector<std::vector<math::Vec3f>> acc(ns, std::vector<math::Vec3f>(n, math::Vec3f{}));
    for (size_t k = 0; k + 1 < ns; ++k)
    {
        for (size_t i = 0; i < n; ++i)
        {
            const size_t j = (i + 1) % n;
            const math::Vec3f& a = sections[k][i];
            const math::Vec3f& b = sections[k][j];
            const math::Vec3f& c = sections[k + 1][j];
            const math::Vec3f& d = sections[k + 1][i];
            const math::Vec3f  fn = quad_normal(a, b, c, d);
            acc[k][i]         += fn;   // a
            acc[k][j]         += fn;   // b
            acc[k + 1][j]     += fn;   // c
            acc[k + 1][i]     += fn;   // d
        }
    }

    MeshBuilder builder;
    builder.reserve(ns * n + (geometry.capped ? 2 * n : 0),
                    (ns - 1) * n * 6 + (geometry.capped ? 2 * (n - 2) * 3 : 0));

    // ── Skin vertices (smooth normals from the accumulation) ──
    std::vector<std::vector<uint32_t>> idx(ns, std::vector<uint32_t>(n));
    const float du = 1.0f / static_cast<float>(ns - 1);
    for (size_t k = 0; k < ns; ++k)
    {
        const float uk = du * static_cast<float>(k);
        for (size_t i = 0; i < n; ++i)
        {
            Vertex v;
            v.position = sections[k][i];
            const float l = acc[k][i].length();
            v.normal = l > 1e-8f ? acc[k][i] / l : math::Vec3f{0.0f, 1.0f, 0.0f};
            v.uv     = {uk, static_cast<float>(i) / static_cast<float>(n), 0.0f};
            v.color  = geometry.color;
            idx[k][i] = builder.add_vertex(v);
        }
    }

    // ── Skin walls (turbine winding: matching quad order) ──
    for (size_t k = 0; k + 1 < ns; ++k)
    {
        for (size_t i = 0; i < n; ++i)
        {
            const size_t j = (i + 1) % n;
            builder.add_quad(idx[k][i], idx[k][j], idx[k + 1][j], idx[k + 1][i]);
        }
    }

    // ── Caps: triangulated section rings with explicit plane normals ──
    if (geometry.capped)
    {
        const math::Vec3f spineStart =
            (ring_centroid(sections[1]) - ring_centroid(sections[0])).normalized();
        const math::Vec3f spineEnd =
            (ring_centroid(sections[ns - 1]) - ring_centroid(sections[ns - 2])).normalized();

        auto add_cap = [&](const std::vector<math::Vec3f>& ring,
                           const math::Vec3f& capNormal)
        {
            std::vector<uint32_t> ringIdx;
            ringIdx.reserve(ring.size());
            for (const auto& p : ring)
            {
                Vertex v;
                v.position = p;
                v.normal   = capNormal;
                v.uv       = {0.5f, 0.5f, 0.0f};
                v.color    = geometry.color;
                ringIdx.push_back(builder.add_vertex(v));
            }

            const std::vector<uint32_t> tris = triangulate_polygon(ring);
            for (size_t t = 0; t + 2 < tris.size(); t += 3)
            {
                const math::Vec3f& p0 = ring[tris[t]];
                const math::Vec3f& p1 = ring[tris[t + 1]];
                const math::Vec3f& p2 = ring[tris[t + 2]];
                const math::Vec3f  gn = (p1 - p0).cross(p2 - p0);
                if (gn.dot(capNormal) < 0.0f)
                    builder.add_triangle(ringIdx[tris[t]], ringIdx[tris[t + 2]], ringIdx[tris[t + 1]]);
                else
                    builder.add_triangle(ringIdx[tris[t]], ringIdx[tris[t + 1]], ringIdx[tris[t + 2]]);
            }
        };

        add_cap(sections.front(), -spineStart);   // cap_start: normal opposes the spine
        add_cap(sections.back(),  spineEnd);      // cap_end:   normal along the spine
    }

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
}

Part generate_loft_part(const LoftGeometry& geometry)
{
    Part part = as_part("loft", generate_loft_mesh(geometry));
    if (part.mesh.vertices.empty())
        return part;

    const uint32_t n         = static_cast<uint32_t>(geometry.sections.front().size());
    const uint32_t ns        = static_cast<uint32_t>(geometry.sections.size());
    const uint32_t skinFaces = 2 * (ns - 1) * n;

    part.patches.push_back(make_patch_range("wall", 0, skinFaces));
    if (geometry.capped)
    {
        const uint32_t capFaces = n - 2;
        part.patches.push_back(make_patch_range("cap_start", skinFaces, capFaces));
        part.patches.push_back(make_patch_range("cap_end", skinFaces + capFaces, capFaces));
    }
    return part;
}

std::vector<math::Vec3f> resample_ring(
    const std::vector<math::Vec3f>& points, uint32_t count)
{
    if (count < 3 || points.size() < 3)
        return {};

    const size_t n = points.size();

    // Perimeter segments (closed ring: last wraps to first).
    std::vector<float> seg(n);
    float total = 0.0f;
    for (size_t i = 0; i < n; ++i)
    {
        seg[i] = (points[(i + 1) % n] - points[i]).length();
        total += seg[i];
    }
    if (total <= 1e-9f)
        return {};   // all points coincident

    std::vector<math::Vec3f> out;
    out.reserve(count);
    out.push_back(points[0]);   // first point stays

    size_t k = 1;
    float  cumulative = 0.0f;
    for (size_t s = 0; s < n && k < count; ++s)
    {
        if (seg[s] <= 1e-12f)
            continue;   // skip zero-length edges
        const math::Vec3f& a = points[s];
        const math::Vec3f& b = points[(s + 1) % n];
        while (k < count)
        {
            const float target = total * static_cast<float>(k) / static_cast<float>(count);
            if (target > cumulative + seg[s] + 1e-6f)
                break;
            const float t = std::clamp((target - cumulative) / seg[s], 0.0f, 1.0f);
            out.push_back(a * (1.0f - t) + b * t);
            ++k;
        }
        cumulative += seg[s];
    }
    // Float-safety: never return fewer than `count` points.
    while (out.size() < count)
        out.push_back(points[0]);
    return out;
}

} // namespace exd::geometry
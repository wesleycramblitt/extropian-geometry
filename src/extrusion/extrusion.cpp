#include <exd/geometry/extrusion.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>
#include <exd/geometry/part.hpp>

#include <cmath>
#include <numbers>

namespace exd::geometry
{

// ============================================================================
// Extrusion
// ============================================================================

MeshData generate_extrusion_mesh(const ExtrusionGeometry& geometry)
{
    const auto& profileVerts = geometry.profile;
    if (profileVerts.size() < 3) return {};

    const float halfD = geometry.depth * 0.5f;
    const size_t n = profileVerts.size();

    MeshBuilder builder;

    // Front face (Z = -halfD, facing -Z)
    for (const auto& p : profileVerts) {
        Vertex v;
        v.position = {p.x, p.y, -halfD};
        v.normal   = {0, 0, -1};
        v.color    = geometry.color;
        builder.add_vertex(v);
    }
    // Back face (Z = +halfD, facing +Z)
    for (const auto& p : profileVerts) {
        Vertex v;
        v.position = {p.x, p.y, halfD};
        v.normal   = {0, 0, 1};
        v.color    = geometry.color;
        builder.add_vertex(v);
    }

    // Side quads connecting front and back rings
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        uint32_t f0 = static_cast<uint32_t>(i);
        uint32_t f1 = static_cast<uint32_t>(j);
        uint32_t b0 = static_cast<uint32_t>(n + i);
        uint32_t b1 = static_cast<uint32_t>(n + j);

        builder.add_triangle(f0, b0, f1);
        builder.add_triangle(f1, b0, b1);
    }

    // Cap faces — use a dedicated center vertex (at origin in XY) so the
    // triangle fan covers the shape cleanly without degenerate perimeter-to-perimeter
    // triangles. Both caps use the same winding formula: (center, next, current)
    // gives CCW winding in the XY plane, which is front-facing from the cap's
    // normal direction.
    if (geometry.capped) {
        // Front cap center
        Vertex fc;
        fc.position = {0.0f, 0.0f, -halfD};
        fc.normal   = {0.0f, 0.0f, -1.0f};
        fc.color    = geometry.color;
        const uint32_t frontCenter = builder.add_vertex(fc);

        // Back cap center
        Vertex bc;
        bc.position = {0.0f, 0.0f, halfD};
        bc.normal   = {0.0f, 0.0f, 1.0f};
        bc.color    = geometry.color;
        const uint32_t backCenter = builder.add_vertex(bc);

        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;

            // Front cap: facing -Z, CCW when viewed from -Z
            builder.add_triangle(frontCenter,
                                 static_cast<uint32_t>(j),
                                 static_cast<uint32_t>(i));

            // Back cap: facing +Z, CCW when viewed from +Z
            builder.add_triangle(backCenter,
                                 static_cast<uint32_t>(n + j),
                                 static_cast<uint32_t>(n + i));
        }
    }

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
}

Part generate_extrusion_part(const ExtrusionGeometry& geometry)
{
    Part part = as_part("extrusion", generate_extrusion_mesh(geometry));
    if (part.mesh.vertices.empty())
        return part;

    // Profile n verts; side wall = 2n tris [0, 2n); front cap (-Z) = n tris
    // [2n, 3n); back cap (+Z) = n tris [3n, 4n) when capped.
    const uint32_t n = static_cast<uint32_t>(geometry.profile.size());
    part.patches.push_back(make_patch_range("wall", 0, 2 * n));
    if (geometry.capped)
    {
        part.patches.push_back(make_patch_range("cap_start", 2 * n, n));
        part.patches.push_back(make_patch_range("cap_end", 3 * n, n));
    }
    return part;
}

// ============================================================================
// Lathe / Revolve
// ============================================================================

namespace {

/// Revolve a profile point (x = revolve radius, y = axis coordinate) by (ca, sa).
math::Vec3f revolve_point(LatheAxis axis, const math::Vec3f& p, float ca, float sa)
{
    switch (axis) {
    case LatheAxis::X: return {p.y, p.x * ca, p.x * sa};
    case LatheAxis::Y: return {p.x * ca, p.y, p.x * sa};
    case LatheAxis::Z: return {p.x * ca, p.x * sa, p.y};
    }
    return {};
}

/// Outward surface normal at a profile point: the radial direction tilted by
/// the local profile slope dr/d(axis), so cones and curved profiles shade
/// correctly instead of always pointing straight out of the revolve axis.
math::Vec3f revolve_normal(LatheAxis axis, float ca, float sa, float slope)
{
    switch (axis) {
    case LatheAxis::X: return math::Vec3f{-slope, ca, sa}.normalized();
    case LatheAxis::Y: return math::Vec3f{ca, -slope, sa}.normalized();
    case LatheAxis::Z: return math::Vec3f{ca, sa, -slope}.normalized();
    }
    return {0.0f, 1.0f, 0.0f};
}

} // namespace

MeshData generate_lathe_mesh(const LatheGeometry& geometry)
{
    const auto& profileVerts = geometry.profile;
    if (profileVerts.size() < 2) return {};

    const uint32_t segs = std::max(3u, geometry.segments);
    const size_t nProf = profileVerts.size();
    const float sweep = geometry.endAngle - geometry.startAngle;

    // Local profile slope dr/d(axis) at each point (finite differences).
    std::vector<float> slope(nProf, 0.0f);
    for (size_t i = 0; i < nProf; ++i) {
        const size_t j = (i + 1 < nProf) ? i + 1 : i - 1;
        const float dr = profileVerts[j].x - profileVerts[i].x;
        const float dz = profileVerts[j].y - profileVerts[i].y;
        if (std::abs(dz) > 1e-6f) slope[i] = dr / dz;
    }

    MeshBuilder builder;
    builder.reserve(nProf * (segs + 1) + 2 * (segs + 2),
                    nProf * segs * 6 + 2 * segs * 3);

    // Side surface: one angular ring per profile point (profiles ordered
    // bottom → top; ring 0 = start angle, ring segs = end angle).
    for (uint32_t r = 0; r <= segs; ++r) {
        float angle = geometry.startAngle + sweep * static_cast<float>(r) / static_cast<float>(segs);
        float ca = std::cos(angle), sa = std::sin(angle);

        for (size_t i = 0; i < nProf; ++i) {
            const auto& p = profileVerts[i];

            Vertex v;
            v.position = revolve_point(geometry.axis, p, ca, sa);
            v.normal   = revolve_normal(geometry.axis, ca, sa, slope[i]);
            v.color    = geometry.color;
            builder.add_vertex(v);
        }
    }

    // Connect rings with quads
    for (uint32_t r = 0; r < segs; ++r) {
        uint32_t base0 = r * static_cast<uint32_t>(nProf);
        uint32_t base1 = (r + 1) * static_cast<uint32_t>(nProf);

        for (size_t i = 0; i < nProf - 1; ++i) {
            uint32_t a = base0 + static_cast<uint32_t>(i);
            uint32_t b = base0 + static_cast<uint32_t>(i + 1);
            uint32_t c = base1 + static_cast<uint32_t>(i);
            uint32_t d = base1 + static_cast<uint32_t>(i + 1);
            builder.add_triangle(a, c, b);
            builder.add_triangle(b, c, d);
        }
    }

    // End caps: a triangle fan per profile end. Ring copies get the axial cap
    // normal so caps render flat with a hard edge at the rim. The cap at the
    // first profile point faces -axis, the cap at the last faces +axis.
    // Points on the revolve axis (|r| ~ 0) are pointed and need no cap.
    if (geometry.capped) {
        const float kEps = 1e-5f;

        auto add_cap = [&](size_t k, bool bottom) {
            if (std::abs(profileVerts[k].x) < kEps) return;

            math::Vec3f axis_n;
            switch (geometry.axis) {
            case LatheAxis::X: axis_n = {1.0f, 0.0f, 0.0f}; break;
            case LatheAxis::Y: axis_n = {0.0f, 1.0f, 0.0f}; break;
            case LatheAxis::Z: axis_n = {0.0f, 0.0f, 1.0f}; break;
            }
            if (bottom) axis_n = -axis_n;

            // Fan hub sits on the revolve axis (radius 0) at the cap plane.
            const math::Vec3f axis_pt{0.0f, profileVerts[k].y, 0.0f};
            Vertex center;
            center.position = revolve_point(geometry.axis, axis_pt, 1.0f, 0.0f);
            center.normal   = axis_n;
            center.color    = geometry.color;
            const uint32_t c = builder.add_vertex(center);

            // Angular ring copies at this profile end, re-normalized to the cap.
            std::vector<uint32_t> ring;
            ring.reserve(segs + 1);
            for (uint32_t r = 0; r <= segs; ++r) {
                const float angle = geometry.startAngle + sweep * static_cast<float>(r) / static_cast<float>(segs);
                const float ca = std::cos(angle), sa = std::sin(angle);
                Vertex v;
                v.position = revolve_point(geometry.axis, profileVerts[k], ca, sa);
                v.normal   = axis_n;
                v.color    = geometry.color;
                ring.push_back(builder.add_vertex(v));
            }

            // Winding chosen so the fan's geometric normal matches the cap
            // normal (the ring's handedness differs between the Y and X/Z axes).
            const bool flip = (bottom != (geometry.axis == LatheAxis::Y));
            for (uint32_t r = 0; r < segs; ++r) {
                if (flip)
                    builder.add_triangle(c, ring[r + 1], ring[r]);
                else
                    builder.add_triangle(c, ring[r], ring[r + 1]);
            }
        };

        add_cap(0, true);            // first profile point: -axis cap
        add_cap(nProf - 1, false);   // last profile point:  +axis cap
    }

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
}

Part generate_lathe_part(const LatheGeometry& geometry)
{
    Part part = as_part("lathe", generate_lathe_mesh(geometry));
    if (part.mesh.vertices.empty())
        return part;

    // nProf profile points, SEG segments; side surface = 2*SEG*(nProf-1) tris
    // [0, sideCount); caps are triangle fans added per profile end when
    // capped AND |profile[k].x| >= 1e-5 — bottom (first profile point) fan
    // first (cap_start, SEG tris), then top (last profile point) fan
    // (cap_end, SEG tris).
    const uint32_t SEG    = std::max(3u, geometry.segments);
    const uint32_t nProf  = static_cast<uint32_t>(geometry.profile.size());
    const uint32_t sideCount = 2u * SEG * (nProf - 1u);

    part.patches.push_back(make_patch_range("surface", 0, sideCount));

    const bool hasBottom = geometry.capped && std::abs(geometry.profile.front().x) >= 1e-5f;
    const bool hasTop    = geometry.capped && std::abs(geometry.profile.back().x)  >= 1e-5f;

    if (hasBottom)
        part.patches.push_back(make_patch_range("cap_start", sideCount, SEG));
    if (hasTop)
        part.patches.push_back(make_patch_range("cap_end", sideCount + (hasBottom ? SEG : 0u), SEG));
    return part;
}

// ============================================================================
// Helix
// ============================================================================

MeshData generate_helix_mesh(const HelixGeometry& geometry)
{
    const auto& profileVerts = geometry.profile;
    if (profileVerts.size() < 3) return {};

    const size_t nProf = profileVerts.size();
    const uint32_t steps = std::max(4u, geometry.pathSteps);
    const float pi = std::numbers::pi_v<float>;

    MeshBuilder builder;
    builder.reserve(nProf * (steps + 1), nProf * steps * 6);

    for (uint32_t s = 0; s <= steps; ++s) {
        float t = static_cast<float>(s) / static_cast<float>(steps);
        float angle = t * geometry.turns * 2.0f * pi;
        float y = t * geometry.height;
        float ca = std::cos(angle), sa = std::sin(angle);

        math::Vec3f posOnPath = {geometry.radius * ca, y, geometry.radius * sa};
        math::Vec3f tangent = {-geometry.radius * sa,
                                geometry.height / (geometry.turns * 2.0f * pi),
                                geometry.radius * ca};
        float tl = tangent.length();
        if (tl > 1e-8f) tangent = tangent / tl;

        math::Vec3f up{0, 1, 0};
        if (std::abs(tangent.y) > 0.999f) up = {0, 0, 1};
        math::Vec3f normal = up.cross(tangent);
        float nl = normal.length();
        if (nl > 1e-8f) normal = normal / nl;
        math::Vec3f binormal = tangent.cross(normal);

        for (size_t i = 0; i < nProf; ++i) {
            const auto& p = profileVerts[i];
            Vertex v;
            v.position = posOnPath + normal * p.x + binormal * p.y;
            v.normal   = {0, 1, 0};
            v.color = geometry.color;
            builder.add_vertex(v);
        }
    }

    for (uint32_t s = 0; s < steps; ++s) {
        uint32_t base0 = s * static_cast<uint32_t>(nProf);
        uint32_t base1 = (s + 1) * static_cast<uint32_t>(nProf);
        for (size_t i = 0; i < nProf; ++i) {
            size_t j = (i + 1) % nProf;
            uint32_t a = base0 + static_cast<uint32_t>(i);
            uint32_t b = base0 + static_cast<uint32_t>(j);
            uint32_t c = base1 + static_cast<uint32_t>(i);
            uint32_t d = base1 + static_cast<uint32_t>(j);
            builder.add_triangle(a, c, b);
            builder.add_triangle(b, c, d);
        }
    }

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
}

} // namespace exd::geometry

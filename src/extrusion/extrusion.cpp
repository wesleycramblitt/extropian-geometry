#include <exd/geometry/extrusion.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>

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

// ============================================================================
// Lathe / Revolve
// ============================================================================

MeshData generate_lathe_mesh(const LatheGeometry& geometry)
{
    const auto& profileVerts = geometry.profile;
    if (profileVerts.size() < 2) return {};

    const uint32_t segs = std::max(3u, geometry.segments);
    const size_t nProf = profileVerts.size();
    const float sweep = geometry.endAngle - geometry.startAngle;

    MeshBuilder builder;
    builder.reserve(nProf * (segs + 1), nProf * segs * 6);

    // Generate rings
    for (uint32_t r = 0; r <= segs; ++r) {
        float angle = geometry.startAngle + sweep * static_cast<float>(r) / static_cast<float>(segs);
        float ca = std::cos(angle), sa = std::sin(angle);

        for (size_t i = 0; i < nProf; ++i) {
            const auto& p = profileVerts[i];
            math::Vec3f pos;
            math::Vec3f normal;

            switch (geometry.axis) {
            case LatheAxis::Y:
                pos    = {p.x * ca, p.y, p.x * sa};
                normal = {ca, 0, sa};
                break;
            case LatheAxis::X:
                pos    = {p.y, p.x * ca, p.x * sa};
                normal = {0, ca, sa};
                break;
            case LatheAxis::Z:
                pos    = {p.x * ca, p.x * sa, p.y};
                normal = {ca, sa, 0};
                break;
            }

            Vertex v;
            v.position = pos;
            v.normal   = normal;
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

    auto result = builder.build();
    result.bounds = compute_bounds(result.vertices);
    return result;
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

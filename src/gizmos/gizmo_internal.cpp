#include "gizmo_internal.hpp"

#include <exd/geometry/primitives3d.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace exd::geometry::detail {

namespace {

constexpr float pi = 3.14159265358979323846f;
const math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

Vertex make_vertex(const math::Vec3f& pos, const math::Vec3f& nrm,
                   const math::Vec3f& uv, const math::Quat& color)
{
    Vertex v;
    v.position = pos;
    v.normal   = nrm;
    v.uv       = uv;
    v.tangent  = tangent_id;
    v.color    = color;
    return v;
}

} // namespace

AxisFrame make_frame(const math::Vec3f& dir)
{
    math::Vec3f d = dir.normalized();
    if (d.length_sq() < 1e-8f)
        d = {0, 1, 0};

    math::Vec3f ref = {0, 1, 0};
    if (std::abs(d.dot(ref)) > 0.999f)
        ref = {0, 0, 1};

    AxisFrame f;
    f.dir   = d;
    f.right = ref.cross(d).normalized();
    f.up    = d.cross(f.right);
    return f;
}

MeshData build_capped_cylinder(const math::Vec3f& from, const math::Vec3f& to,
                               float radius, uint32_t slices, const math::Quat& color)
{
    if (radius <= 0.0f)
        return {};

    math::Vec3f span = to - from;
    float len = span.length();
    if (len < 1e-8f)
        return {};

    if (slices < 3)
        slices = 3;

    math::Vec3f dir = span / len;
    AxisFrame fr    = make_frame(dir);

    const uint32_t vertsPerRing = slices + 1;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(2 * vertsPerRing + 2);
    indices.reserve(slices * 12);

    // Ring 0 at `from`, ring 1 at `to`; side normals radial, uv = (u along length, v around).
    for (uint32_t ring = 0; ring < 2; ++ring)
    {
        math::Vec3f center = (ring == 0) ? from : to;
        float uu           = (ring == 0) ? 0.0f : 1.0f;

        for (uint32_t i = 0; i <= slices; ++i)
        {
            float a = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
            math::Vec3f radial = fr.right * std::cos(a) + fr.up * std::sin(a);
            vertices.push_back(make_vertex(center + radial * radius, radial,
                                           {uu, static_cast<float>(i) / static_cast<float>(slices), 0.0f},
                                           color));
        }
    }

    // Side quads
    for (uint32_t i = 0; i < slices; ++i)
    {
        uint32_t b0 = i, b1 = i + 1;
        uint32_t t0 = vertsPerRing + i, t1 = t0 + 1;

        indices.push_back(b0); indices.push_back(t0); indices.push_back(b1);
        indices.push_back(b1); indices.push_back(t0); indices.push_back(t1);
    }

    // Cap centers (normals ±dir)
    uint32_t fromCap = static_cast<uint32_t>(vertices.size());
    vertices.push_back(make_vertex(from, -dir, {0.5f, 0.5f, 0.0f}, color));
    uint32_t toCap = static_cast<uint32_t>(vertices.size());
    vertices.push_back(make_vertex(to, dir, {0.5f, 0.5f, 0.0f}, color));

    // Cap fans
    for (uint32_t i = 0; i < slices; ++i)
    {
        // from cap (normal -dir): center, next, curr
        indices.push_back(fromCap);
        indices.push_back(i + 1);
        indices.push_back(i);

        // to cap (normal +dir): center, curr, next
        indices.push_back(toCap);
        indices.push_back(vertsPerRing + i);
        indices.push_back(vertsPerRing + i + 1);
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = compute_bounds(mesh.vertices);
    return mesh;
}

MeshData build_cone(const math::Vec3f& baseCenter, const math::Vec3f& tip,
                    float baseRadius, uint32_t slices, const math::Quat& color,
                    bool baseCap)
{
    if (baseRadius <= 0.0f)
        return {};

    math::Vec3f span = tip - baseCenter;
    float coneLen    = span.length();
    if (coneLen < 1e-8f)
        return {};

    if (slices < 3)
        slices = 3;

    math::Vec3f dir = span / coneLen;
    AxisFrame fr    = make_frame(dir);

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(slices + 2);
    indices.reserve(baseCap ? slices * 6 : slices * 3);

    // Base ring
    for (uint32_t i = 0; i <= slices; ++i)
    {
        float a = static_cast<float>(i) * 2.0f * pi / static_cast<float>(slices);
        math::Vec3f radial = fr.right * std::cos(a) + fr.up * std::sin(a);
        // axes.cpp side-normal formula: (radial*coneLen + dir*baseRadius).normalized()
        math::Vec3f nrm = (radial * coneLen + dir * baseRadius).normalized();
        vertices.push_back(make_vertex(baseCenter + radial * baseRadius, nrm,
                                       {static_cast<float>(i) / static_cast<float>(slices), 0.0f, 0.0f},
                                       color));
    }

    uint32_t tipIdx = static_cast<uint32_t>(vertices.size());
    vertices.push_back(make_vertex(tip, dir, {0.5f, 1.0f, 0.0f}, color));

    for (uint32_t i = 0; i < slices; ++i)
    {
        indices.push_back(tipIdx);
        indices.push_back(i);
        indices.push_back(i + 1);
    }

    if (baseCap)
    {
        uint32_t capCenter = static_cast<uint32_t>(vertices.size());
        vertices.push_back(make_vertex(baseCenter, -dir, {0.5f, 0.5f, 0.0f}, color));

        for (uint32_t i = 0; i < slices; ++i)
        {
            indices.push_back(capCenter);
            indices.push_back(i + 1);
            indices.push_back(i);
        }
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = compute_bounds(mesh.vertices);
    return mesh;
}

MeshData build_cone_with_fillet(const math::Vec3f& baseCenter, const math::Vec3f& dir,
                                const math::Vec3f& tip, float baseRadius,
                                uint32_t slices, const math::Quat& color)
{
    if (baseRadius <= 0.0f)
        return {};

    math::Vec3f d = dir.normalized();
    if (d.length_sq() < 1e-8f)
        d = {0, 1, 0};

    AxisFrame fr = make_frame(d);
    uint32_t segs = std::max<uint32_t>(6, slices);

    float filletR = std::clamp(baseRadius * 0.25f, 1e-4f, baseRadius * 0.49f);

    // Full torus bead in the plane of the base, centered at baseCenter.
    MeshData bead = build_torus_arc(d, fr.right, baseRadius - filletR, filletR,
                                    0.0f, 2.0f * pi, segs, segs, color);
    for (auto& v : bead.vertices)
        v.position += baseCenter;
    bead.bounds = compute_bounds(bead.vertices);

    // Cone without base cap (the fillet hides the base seam).
    MeshData cone = build_cone(baseCenter, tip, baseRadius, segs, color, /*baseCap=*/false);

    return concat_meshes(std::move(cone), std::move(bead));
}

MeshData build_torus_arc(const math::Vec3f& axis, const math::Vec3f& zeroDir,
                         float majorR, float minorR, float start, float sweep,
                         uint32_t majorSegs, uint32_t minorSegs, const math::Quat& color)
{
    if (majorR <= 0.0f || minorR <= 0.0f)
        return {};
    if (majorSegs < 1)
        majorSegs = 1;
    if (minorSegs < 3)
        minorSegs = 3;

    math::Vec3f ax = axis.normalized();
    math::Vec3f z0 = zeroDir.normalized();
    math::Vec3f z1 = ax.cross(z0);

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(static_cast<size_t>(majorSegs + 1) * (minorSegs + 1));
    indices.reserve(static_cast<size_t>(majorSegs) * minorSegs * 6);

    for (uint32_t i = 0; i <= majorSegs; ++i)
    {
        float theta = start + static_cast<float>(i) * sweep / static_cast<float>(majorSegs);
        math::Vec3f c = z0 * std::cos(theta) + z1 * std::sin(theta);

        for (uint32_t j = 0; j <= minorSegs; ++j)
        {
            float phi = static_cast<float>(j) * 2.0f * pi / static_cast<float>(minorSegs);
            math::Vec3f crossNormal = c * std::cos(phi) + ax * std::sin(phi);

            math::Vec3f pos = c * majorR + crossNormal * minorR;
            vertices.push_back(make_vertex(pos, crossNormal,
                                           {static_cast<float>(i) / static_cast<float>(majorSegs),
                                            static_cast<float>(j) / static_cast<float>(minorSegs),
                                            0.0f},
                                           color));
        }
    }

    for (uint32_t i = 0; i < majorSegs; ++i)
    {
        for (uint32_t j = 0; j < minorSegs; ++j)
        {
            uint32_t i0 = static_cast<uint32_t>(i * (minorSegs + 1) + j);
            uint32_t i1 = i0 + 1;
            uint32_t i2 = i0 + static_cast<uint32_t>(minorSegs + 1);
            uint32_t i3 = i2 + 1;

            indices.push_back(i0); indices.push_back(i2); indices.push_back(i1);
            indices.push_back(i1); indices.push_back(i2); indices.push_back(i3);
        }
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = compute_bounds(mesh.vertices);
    return mesh;
}

MeshData build_box(const math::Vec3f& center, const math::Vec3f& halfSize, const math::Quat& color)
{
    float hx = halfSize.x, hy = halfSize.y, hz = halfSize.z;

    struct Face
    {
        math::Vec3f n;
        math::Vec3f v0, v1, v2, v3;
    };
    const Face faces[6] = {
        {{ 1,  0,  0}, { hx, -hy, -hz}, { hx,  hy, -hz}, { hx,  hy,  hz}, { hx, -hy,  hz}},
        {{-1,  0,  0}, {-hx, -hy,  hz}, {-hx,  hy,  hz}, {-hx,  hy, -hz}, {-hx, -hy, -hz}},
        {{ 0,  1,  0}, {-hx,  hy, -hz}, {-hx,  hy,  hz}, { hx,  hy,  hz}, { hx,  hy, -hz}},
        {{ 0, -1,  0}, {-hx, -hy,  hz}, {-hx, -hy, -hz}, { hx, -hy, -hz}, { hx, -hy,  hz}},
        {{ 0,  0,  1}, {-hx, -hy,  hz}, { hx, -hy,  hz}, { hx,  hy,  hz}, {-hx,  hy,  hz}},
        {{ 0,  0, -1}, {-hx, -hy, -hz}, {-hx,  hy, -hz}, { hx,  hy, -hz}, { hx, -hy, -hz}},
    };

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(24);
    indices.reserve(36);

    for (const auto& f : faces)
    {
        uint32_t base = static_cast<uint32_t>(vertices.size());
        const math::Vec3f corners[4] = {f.v0, f.v1, f.v2, f.v3};
        for (const auto& c : corners)
            vertices.push_back(make_vertex(center + c, f.n, {0.5f, 0.5f, 0.0f}, color));

        indices.push_back(base + 0); indices.push_back(base + 1); indices.push_back(base + 2);
        indices.push_back(base + 0); indices.push_back(base + 2); indices.push_back(base + 3);
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = compute_bounds(mesh.vertices);
    return mesh;
}

MeshData build_icosphere(float radius, const math::Quat& color, uint32_t subdivisions)
{
    return generate_icosahedron_mesh(radius, static_cast<int>(subdivisions), color);
}

MeshData concat_meshes(const MeshData& a, const MeshData& b)
{
    if (b.vertices.empty())
        return a;
    if (a.vertices.empty())
        return b;

    MeshData r = a;
    r.indices.reserve(a.indices.size() + b.indices.size());
    uint32_t offset = static_cast<uint32_t>(a.vertices.size());
    for (auto idx : b.indices)
        r.indices.push_back(offset + idx);
    r.vertices.insert(r.vertices.end(), b.vertices.begin(), b.vertices.end());
    r.topology = PrimitiveTopology::Triangles;
    r.bounds   = compute_bounds(r.vertices);
    return r;
}

GizmoPart make_part(GizmoAxis axis, GizmoPartKind kind, uint32_t partId, MeshData&& mesh)
{
    GizmoPart part;
    part.axis   = axis;
    part.kind   = kind;
    part.partId = partId;
    if (!mesh.vertices.empty())
    {
        mesh.topology = PrimitiveTopology::Triangles;
        mesh.bounds   = compute_bounds(mesh.vertices);
    }
    part.mesh = std::move(mesh);
    return part;
}

} // namespace exd::geometry::detail
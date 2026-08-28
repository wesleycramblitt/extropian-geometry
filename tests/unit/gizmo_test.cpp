#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

using namespace exd::geometry;

namespace {

bool normals_unit(const GizmoParts& parts)
{
    for (const auto& p : parts)
        for (const auto& v : p.mesh.vertices)
        {
            float l = std::sqrt(v.normal.x * v.normal.x +
                                v.normal.y * v.normal.y +
                                v.normal.z * v.normal.z);
            if (std::abs(l - 1.0f) > 0.001f)
                return false;
        }
    return true;
}

float min_dist_to_point(const MeshData& mesh, const exd::math::Vec3f& pt)
{
    float best = std::numeric_limits<float>::max();
    for (const auto& v : mesh.vertices)
        best = std::min(best, (v.position - pt).length());
    return best;
}

} // namespace

// ── Translation gizmo ───────────────────────────────────────────────────────

TEST_CASE("translation: default parts")
{
    auto parts = generate_translation_gizmo({});
    REQUIRE(parts.size() == 6);

    // Kinds in order: Shaft x3, Handle x3.
    for (int i = 0; i < 3; ++i)
        CHECK(parts[i].kind == GizmoPartKind::Shaft);
    for (int i = 3; i < 6; ++i)
        CHECK(parts[i].kind == GizmoPartKind::Handle);

    // Axes: X, Y, Z for shafts and handles.
    CHECK(parts[0].axis == GizmoAxis::X);
    CHECK(parts[1].axis == GizmoAxis::Y);
    CHECK(parts[2].axis == GizmoAxis::Z);
    CHECK(parts[3].axis == GizmoAxis::X);
    CHECK(parts[4].axis == GizmoAxis::Y);
    CHECK(parts[5].axis == GizmoAxis::Z);

    // partIds 0..5.
    for (uint32_t i = 0; i < parts.size(); ++i)
        CHECK(parts[i].partId == i);

    // Every part mesh is non-empty, Triangles topology, sane bounds.
    for (const auto& p : parts)
    {
        CHECK(!p.mesh.vertices.empty());
        CHECK(!p.mesh.indices.empty());
        CHECK(p.mesh.topology == PrimitiveTopology::Triangles);
        CHECK(p.mesh.bounds.min.x <= p.mesh.bounds.max.x + 1e-6f);
        CHECK(p.mesh.bounds.min.y <= p.mesh.bounds.max.y + 1e-6f);
        CHECK(p.mesh.bounds.min.z <= p.mesh.bounds.max.z + 1e-6f);
    }

    // Per-axis colors on shaft parts: X red, Y green, Z blue (Quat: w=R,x=G,y=B).
    auto hasColor = [](const MeshData& m, float R, float G, float B)
    {
        for (const auto& v : m.vertices)
            if (std::abs(v.color.w - R) < 0.01f &&
                std::abs(v.color.x - G) < 0.01f &&
                std::abs(v.color.y - B) < 0.01f)
                return true;
        return false;
    };
    CHECK(hasColor(parts[0].mesh, 1.0f, 0.0f, 0.0f));
    CHECK(hasColor(parts[1].mesh, 0.0f, 1.0f, 0.0f));
    CHECK(hasColor(parts[2].mesh, 0.0f, 0.0f, 1.0f));
}

TEST_CASE("translation: degenerate")
{
    TranslationGizmoGeometry g;
    g.length = 0.0f;
    CHECK(generate_translation_gizmo(g).empty());

    // coneLength beyond the clamp still emits the full 6 parts.
    TranslationGizmoGeometry g2;
    g2.coneLength = 2.0f * g2.length;
    CHECK(generate_translation_gizmo(g2).size() == 6);

    // No cones → shaft-only: 3 parts with ids 0..2.
    TranslationGizmoGeometry g3;
    g3.coneRadius = 0.0f;
    auto parts = generate_translation_gizmo(g3);
    REQUIRE(parts.size() == 3);
    for (uint32_t i = 0; i < 3; ++i)
    {
        CHECK(parts[i].kind == GizmoPartKind::Shaft);
        CHECK(parts[i].partId == i);
    }

    // No shafts → handle-only: 3 parts.
    TranslationGizmoGeometry g4;
    g4.shaftRadius = 0.0f;
    auto parts4 = generate_translation_gizmo(g4);
    REQUIRE(parts4.size() == 3);
    CHECK(parts4[0].kind == GizmoPartKind::Handle);
}

TEST_CASE("translation: normals unit length")
{
    auto parts = generate_translation_gizmo({});
    CHECK(normals_unit(parts));
}

// ── Scale gizmo ─────────────────────────────────────────────────────────────

TEST_CASE("scale: default parts")
{
    auto parts = generate_scale_gizmo({});
    REQUIRE(parts.size() == 6);

    // Shafts then Handle parts are axis-aligned cubes: 24 verts / 36 idx.
    const float cubeSize = 0.12f;
    const float length   = 1.0f;
    const exd::math::Vec3f axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    for (int i = 0; i < 3; ++i)
    {
        CHECK(parts[3 + i].kind == GizmoPartKind::Handle);
        CHECK(parts[3 + i].mesh.vertices.size() == 24);
        CHECK(parts[3 + i].mesh.indices.size() == 36);

        // Cube centered at dir*(length - cubeSize/2), axis-aligned to world axes.
        const exd::math::Vec3f center = axes[i] * (length - cubeSize * 0.5f);
        const float half = cubeSize * 0.5f;
        for (const auto& v : parts[3 + i].mesh.vertices)
        {
            CHECK(std::abs(v.position.x - center.x) <= half + 1e-4f);
            CHECK(std::abs(v.position.y - center.y) <= half + 1e-4f);
            CHECK(std::abs(v.position.z - center.z) <= half + 1e-4f);
        }

        // The cube geometry contains the axis point dir*(length - cubeSize/2).
        CHECK(parts[3 + i].mesh.bounds.min.x <= center.x + 1e-4f);
        CHECK(parts[3 + i].mesh.bounds.max.x >= center.x - 1e-4f);
        CHECK(parts[3 + i].mesh.bounds.min.y <= center.y + 1e-4f);
        CHECK(parts[3 + i].mesh.bounds.max.y >= center.y - 1e-4f);
        CHECK(parts[3 + i].mesh.bounds.min.z <= center.z + 1e-4f);
        CHECK(parts[3 + i].mesh.bounds.max.z >= center.z - 1e-4f);
    }
}

// ── Rotation gizmo ──────────────────────────────────────────────────────────

TEST_CASE("rotation: default parts")
{
    auto parts = generate_rotation_gizmo({});
    REQUIRE(parts.size() == 6);

    // 3 Ring, 3 Knob.
    for (int i = 0; i < 3; ++i)
        CHECK(parts[i].kind == GizmoPartKind::Ring);
    for (int i = 3; i < 6; ++i)
        CHECK(parts[i].kind == GizmoPartKind::Knob);

    // X-ring lives in the YZ plane: |x| <= ringThickness + eps.
    const float t = 0.035f;
    for (const auto& v : parts[0].mesh.vertices)
        CHECK(std::abs(v.position.x) <= t + 1e-4f);
    for (const auto& v : parts[1].mesh.vertices)
        CHECK(std::abs(v.position.y) <= t + 1e-4f);
    for (const auto& v : parts[2].mesh.vertices)
        CHECK(std::abs(v.position.z) <= t + 1e-4f);

    // Knob centers sit at radius*zeroDir per the angle-0 convention.
    const struct { GizmoAxis axis; exd::math::Vec3f pt; } knobs[3] = {
        {GizmoAxis::X, {0, 1, 0}},
        {GizmoAxis::Y, {0, 0, 1}},
        {GizmoAxis::Z, {1, 0, 0}},
    };
    for (int i = 0; i < 3; ++i)
    {
        CHECK(parts[3 + i].axis == knobs[i].axis);
        CHECK(min_dist_to_point(parts[3 + i].mesh, knobs[i].pt) < 0.05f);
    }
}

TEST_CASE("rotation: degenerate")
{
    RotationGizmoGeometry g;
    g.radius = 0.0f;
    CHECK(generate_rotation_gizmo(g).empty());

    RotationGizmoGeometry g2;
    g2.knob = false;
    CHECK(generate_rotation_gizmo(g2).size() == 3);
}

TEST_CASE("rotation: ring bounds")
{
    auto parts = generate_rotation_gizmo({});
    const float r = 1.0f, t = 0.035f;
    for (int i = 0; i < 3; ++i)
    {
        const auto& b = parts[i].mesh.bounds;
        CHECK(b.min.x >= -(r + t) - 1e-4f);
        CHECK(b.max.x <=  (r + t) + 1e-4f);
        CHECK(b.min.y >= -(r + t) - 1e-4f);
        CHECK(b.max.y <=  (r + t) + 1e-4f);
        CHECK(b.min.z >= -(r + t) - 1e-4f);
        CHECK(b.max.z <=  (r + t) + 1e-4f);
    }
}

// ── Bend gizmo ──────────────────────────────────────────────────────────────

TEST_CASE("bend: parts")
{
    auto parts = generate_bend_gizmo({});
    REQUIRE(parts.size() == 3);
    CHECK(parts[0].kind == GizmoPartKind::Shaft);
    CHECK(parts[1].kind == GizmoPartKind::Arc);
    CHECK(parts[2].kind == GizmoPartKind::ArrowHead);

    // u_auto for bendAxis {0,1,0} → u = {0,0,1}; the ring circle is centered at
    // -radius*u = (0,0,-1), matching the deform_mesh bend spine.
    const float radius = 1.0f;
    const exd::math::Vec3f center = {0.0f, 0.0f, -1.0f};
    for (const auto& v : parts[1].mesh.vertices)
    {
        float dist = (v.position - center).length();
        CHECK(std::abs(dist - radius) <= 0.06f);
    }

    // Arc angle extent <= sweep.
    const exd::math::Vec3f ax = {0, 1, 0};
    const exd::math::Vec3f u  = {0, 0, 1};
    const exd::math::Vec3f z1 = ax.cross(u); // {1,0,0}
    float minA = std::numeric_limits<float>::max();
    float maxA = -std::numeric_limits<float>::max();
    for (const auto& v : parts[1].mesh.vertices)
    {
        exd::math::Vec3f rel = v.position - center;
        float a = std::atan2(rel.dot(z1), rel.dot(u));
        minA = std::min(minA, a);
        maxA = std::max(maxA, a);
    }
    const float sweep = 1.5707963f;
    CHECK(maxA - minA <= sweep + 1e-3f);

    CHECK(normals_unit(parts));
}

TEST_CASE("bend: degenerate")
{
    BendGizmoGeometry g;
    g.radius = 0.0f;
    CHECK(generate_bend_gizmo(g).empty());
}

// ── Twist gizmo ─────────────────────────────────────────────────────────────

TEST_CASE("twist: parts")
{
    auto parts = generate_twist_gizmo({});
    REQUIRE(parts.size() == 4);
    CHECK(parts[0].kind == GizmoPartKind::Arc);
    CHECK(parts[1].kind == GizmoPartKind::ArrowHead);
    CHECK(parts[2].kind == GizmoPartKind::Arc);
    CHECK(parts[3].kind == GizmoPartKind::ArrowHead);

    // Both arcs sit on the circle of radius `radius` around twistAxis.
    const float radius = 1.0f, tubeRadius = 0.03f;
    const exd::math::Vec3f ax = {0, 1, 0};
    for (int i = 0; i < 4; i += 2)
    {
        for (const auto& v : parts[i].mesh.vertices)
        {
            exd::math::Vec3f radial = v.position - ax * v.position.dot(ax);
            CHECK(std::abs(radial.length() - radius) <= tubeRadius + 1e-3f);
        }
    }

    TwistGizmoGeometry g;
    g.radius = 0.0f;
    CHECK(generate_twist_gizmo(g).empty());
}

// ── Taper gizmo ─────────────────────────────────────────────────────────────

TEST_CASE("taper: parts")
{
    auto parts = generate_taper_gizmo({});
    REQUIRE(parts.size() == 3);
    CHECK(parts[0].kind == GizmoPartKind::Frame);
    CHECK(parts[1].kind == GizmoPartKind::Frame);
    CHECK(parts[2].kind == GizmoPartKind::Connector);

    const float frameThickness = 0.03f;
    const float startHalf = 0.3f * 1.0f * 0.5f;   // baseFrameSize*startScale/2 = 0.15
    const float endHalf   = 0.3f * 0.5f * 0.5f;   // baseFrameSize*endScale/2   = 0.075

    // Frames sit at y = ±0.5 (default taper axis Y).
    float maxDY0 = 0.0f, maxDY1 = 0.0f;
    for (const auto& v : parts[0].mesh.vertices)
        maxDY0 = std::max(maxDY0, std::abs(v.position.y - (-0.5f)));
    for (const auto& v : parts[1].mesh.vertices)
        maxDY1 = std::max(maxDY1, std::abs(v.position.y - 0.5f));
    CHECK(maxDY0 <= frameThickness + 1e-3f);
    CHECK(maxDY1 <= frameThickness + 1e-3f);

    // Corner radius = frame half-size: the square reaches half of
    // baseFrameSize*scale along each in-plane axis (frameThickness tube cross-section).
    auto hasAxisExtent = [&](const MeshData& m, float target)
    {
        for (const auto& v : m.vertices)
        {
            if (std::abs(std::abs(v.position.x) - target) <= frameThickness + 1e-3f)
                return true;
            if (std::abs(std::abs(v.position.z) - target) <= frameThickness + 1e-3f)
                return true;
        }
        return false;
    };
    CHECK(hasAxisExtent(parts[0].mesh, startHalf));
    CHECK(hasAxisExtent(parts[1].mesh, endHalf));

    CHECK(!parts[2].mesh.vertices.empty());
    CHECK(normals_unit(parts));
}

TEST_CASE("taper: degenerate")
{
    TaperGizmoGeometry g;
    g.length = 0.0f;
    CHECK(generate_taper_gizmo(g).empty());
}

// ── Lattice gizmo ───────────────────────────────────────────────────────────

TEST_CASE("lattice: default")
{
    auto parts = generate_lattice_gizmo({});
    REQUIRE(parts.size() == 28); // 27 control points + 1 connector

    int pointCount = 0, connectorCount = 0;
    for (const auto& p : parts)
    {
        if (p.kind == GizmoPartKind::ControlPoint) ++pointCount;
        if (p.kind == GizmoPartKind::Connector)    ++connectorCount;
    }
    CHECK(pointCount == 27);
    CHECK(connectorCount == 1);

    // Node positions: size*(i/(nx-1) - 0.5), X fastest; icosphere radius 0.045.
    for (int k = 0; k < 3; ++k)
        for (int j = 0; j < 3; ++j)
            for (int i = 0; i < 3; ++i)
            {
                uint32_t id = static_cast<uint32_t>(i + j * 3 + k * 9);
                const exd::math::Vec3f expected = {
                    static_cast<float>(i) / 2.0f - 0.5f,
                    static_cast<float>(j) / 2.0f - 0.5f,
                    static_cast<float>(k) / 2.0f - 0.5f
                };
                CHECK(!parts[id].mesh.vertices.empty());
                CHECK(min_dist_to_point(parts[id].mesh, expected) < 0.05f);
            }

    // Connector part is the last part with partId 0 and non-empty geometry.
    CHECK(parts[27].kind == GizmoPartKind::Connector);
    CHECK(parts[27].partId == 0);
    CHECK(!parts[27].mesh.vertices.empty());
}

TEST_CASE("lattice: degenerate")
{
    LatticeCageGeometry g;
    g.grid = {1, 3, 3};
    CHECK(generate_lattice_gizmo(g).empty());

    LatticeCageGeometry g2;
    g2.size = {0.0f, 1.0f, 1.0f};
    CHECK(generate_lattice_gizmo(g2).empty());
}

TEST_CASE("lattice: grid 2x2x2")
{
    LatticeCageGeometry g;
    g.grid = {2, 2, 2};
    auto parts = generate_lattice_gizmo(g);
    REQUIRE(parts.size() == 9); // 8 control points + 1 connector

    // Node formula puts grid-2 nodes at ±0.5*size along each axis.
    for (int k = 0; k < 2; ++k)
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i)
            {
                uint32_t id = static_cast<uint32_t>(i + j * 2 + k * 4);
                const exd::math::Vec3f expected = {
                    (i == 0 ? -0.5f : 0.5f) * g.size.x,
                    (j == 0 ? -0.5f : 0.5f) * g.size.y,
                    (k == 0 ? -0.5f : 0.5f) * g.size.z
                };
                CHECK(min_dist_to_point(parts[id].mesh, expected) < 0.05f);
            }
    CHECK(parts[8].kind == GizmoPartKind::Connector);
}

// ── Merge / filter ──────────────────────────────────────────────────────────

TEST_CASE("merge: merge_gizmo_parts")
{
    auto parts = generate_translation_gizmo({});
    size_t totalVerts = 0, totalIdx = 0;
    for (const auto& p : parts)
    {
        totalVerts += p.mesh.vertices.size();
        totalIdx   += p.mesh.indices.size();
    }

    auto merged = merge_gizmo_parts(parts);
    CHECK(merged.vertices.size() == totalVerts);
    CHECK(merged.indices.size() == totalIdx);
    CHECK(merged.topology == PrimitiveTopology::Triangles);
    CHECK(merged.bounds.min.x <= merged.bounds.max.x + 1e-6f);
    CHECK(merged.bounds.min.y <= merged.bounds.max.y + 1e-6f);
    CHECK(merged.bounds.min.z <= merged.bounds.max.z + 1e-6f);

    auto empty = merge_gizmo_parts({});
    CHECK(empty.vertices.empty());
    CHECK(empty.indices.empty());
}

TEST_CASE("filter: filter_gizmo_parts")
{
    auto parts = generate_translation_gizmo({});

    auto shafts = filter_gizmo_parts(parts, GizmoPartKind::Shaft);
    CHECK(shafts.size() == 3);

    auto hx = filter_gizmo_parts(parts, GizmoPartKind::Handle, GizmoAxis::X);
    REQUIRE(hx.size() == 1);
    CHECK(hx[0].partId == 3);
    CHECK(hx[0].axis == GizmoAxis::X);
}
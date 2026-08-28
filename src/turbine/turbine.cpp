#include <exd/geometry/turbine.hpp>

#include "turbine_internal.hpp"

#include <exd/geometry/extrusion.hpp>
#include <exd/geometry/mesh_ops.hpp>
#include <exd/geometry/part.hpp>
#include <exd/geometry/spline.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <cstdint>
#include <string>
#include <vector>

namespace exd::geometry
{
namespace
{

constexpr float kPi    = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

float deg2rad(float deg) { return deg * kPi / 180.0f; }

// ── Small mesh accumulator with per-vertex normal smoothing ──
struct Accumulator
{
    std::vector<math::Vec3f> pos;
    std::vector<uint32_t>    idx;

    uint32_t add(math::Vec3f p) { pos.push_back(p); return static_cast<uint32_t>(pos.size() - 1); }
    void tri(uint32_t a, uint32_t b, uint32_t c) { idx.push_back(a); idx.push_back(b); idx.push_back(c); }
    void quad(uint32_t a, uint32_t b, uint32_t c, uint32_t d) { tri(a, b, c); tri(a, c, d); }

    MeshData build() const
    {
        MeshData mesh;
        mesh.topology = PrimitiveTopology::Triangles;

        std::vector<math::Vec3f> norm(pos.size(), math::Vec3f{0.0f, 0.0f, 0.0f});
        for (std::size_t i = 0; i + 2 < idx.size(); i += 3) {
            const math::Vec3f e1 = pos[idx[i + 1]] - pos[idx[i]];
            const math::Vec3f e2 = pos[idx[i + 2]] - pos[idx[i]];
            const math::Vec3f n = e1.cross(e2);
            norm[idx[i]]     += n;
            norm[idx[i + 1]] += n;
            norm[idx[i + 2]] += n;
        }
        mesh.vertices.resize(pos.size());
        for (std::size_t i = 0; i < pos.size(); ++i) {
            mesh.vertices[i].position = pos[i];
            mesh.vertices[i].normal   = norm[i].normalized();
        }
        mesh.indices = idx;
        mesh.bounds  = compute_bounds(mesh.vertices);
        return mesh;
    }
};

MonotoneCubicSpline spline_of(const std::vector<math::Vec2f>& pts)
{
    std::vector<float> xs, ys;
    xs.reserve(pts.size());
    ys.reserve(pts.size());
    for (const auto& p : pts) { xs.push_back(p.x); ys.push_back(p.y); }
    return MonotoneCubicSpline(std::move(xs), std::move(ys));
}

math::Vec2f lerp(const math::Vec2f& a, const math::Vec2f& b, float t)
{
    return a + (b - a) * t;
}

/// Axial midpoint of the flow path z-range. The whole machine is centered on
/// the origin and its axis runs along -Z by default (matching a camera that
/// looks down -Z), so a turbine can be dropped at the origin directly.
float axial_center(const FlowPath& flow)
{
    float zmin =  std::numeric_limits<float>::max();
    float zmax = -std::numeric_limits<float>::max();
    for (const auto& p : flow.hub_points)    { zmin = std::min(zmin, p.x); zmax = std::max(zmax, p.x); }
    for (const auto& p : flow.shroud_points) { zmin = std::min(zmin, p.x); zmax = std::max(zmax, p.x); }
    if (zmin > zmax) return 0.0f;
    return (zmin + zmax) * 0.5f;
}

/// Cubic-Hermite camber line from inlet/exit metal angles (relative to chord).
struct Camber
{
    float y0 = 0.0f, y1 = 0.0f, s0 = 0.0f, s1 = 0.0f;   // y(0), y(1), y'(0), y'(1)

    float operator()(float x) const
    {
        const float x2 = x * x;
        const float x3 = x2 * x;
        const float h00 =  2.0f * x3 - 3.0f * x2 + 1.0f;
        const float h10 =        x3 - 2.0f * x2 + x;
        const float h01 = -2.0f * x3 + 3.0f * x2;
        const float h11 =        x3 -       x2;
        return h00 * y0 + h10 * s0 + h01 * y1 + h11 * s1;
    }
};

/// NACA-style closed-TE thickness distribution (0..1 in x/c), scaled to t/c.
float naca_thickness(float x, float t_over_c)
{
    if (x <= 0.0f || x >= 1.0f) return 0.0f;
    const float xh = std::sqrt(x);
    return 5.0f * t_over_c *
        (0.2969f * xh - 0.1260f * x - 0.3516f * x * x + 0.2843f * x * x * x - 0.1015f * x * x * x * x);
}

} // namespace

namespace detail {

std::vector<math::Vec3f> build_meridional_profile(const std::vector<math::Vec2f>& points)
{
    if (points.size() < 2) return {};

    const MonotoneCubicSpline spline = spline_of(points);
    const float zmin = spline.min_x();
    const float zmax = spline.max_x();
    if (zmax <= zmin) return {};

    // Meridional refinement matching generate_flow_path_mesh at the default
    // revolve_segments = 64 (steps = max(8, 64/2) = 32).
    const uint32_t steps = 32u;
    const float center = (zmin + zmax) * 0.5f;

    std::vector<math::Vec3f> profile;
    profile.reserve(steps + 1);
    for (uint32_t i = 0; i <= steps; ++i) {
        const float z = zmin + (zmax - zmin) * static_cast<float>(i) / static_cast<float>(steps);
        // x = r (revolve radius), y = axial, negated + centered on origin.
        profile.push_back({spline.evaluate(z), -(z - center), 0.0f});
    }
    return profile;
}

MeshData build_blade_row_impl(const BladeRow& row, const FlowPath& flow,
                              uint32_t revolve_segments,
                              BladeRowBuildInfo* info)
{
    if (info) {
        info->skinPerBlade      = 0;
        info->hubCapPerBlade    = 0;
        info->shroudCapPerBlade = 0;
        info->stridePerBlade    = 0;
    }

    (void)revolve_segments;

    std::vector<BladeSection> sections = row.sections;
    if (sections.size() < 2) {
        for (int i = 0; i <= 4; ++i)
            sections.push_back(BladeSection{static_cast<float>(i) / 4.0f});
    }

    const uint32_t n = std::max(8u, row.chordwise_points);
    const uint32_t Z = std::max(1u, static_cast<uint32_t>(row.blade_count.value));
    const float center = axial_center(flow);

    Accumulator acc;
    std::vector<std::vector<uint32_t>> loops;
    loops.reserve(sections.size());

    for (const BladeSection& sec : sections) {
        const float f = sec.span;
        const math::Vec2f le = lerp(row.leading_edge_hub,  row.leading_edge_shroud,  f);
        const math::Vec2f te = lerp(row.trailing_edge_hub, row.trailing_edge_shroud, f);
        const math::Vec2f chord = te - le;
        const float chord_len = chord.length();
        if (chord_len <= 1e-6f) return {};

        const math::Vec2f u = chord / chord_len;   // chord unit vector in (z, r)

        const std::vector<math::Vec2f> profile =
            generate_blade_section_profile(sec, chord_len, n);

        // Stagger rotates the section about its LE in the (axial, tangential)
        // plane. The stagger is a per-section parameter.
        const float sg = std::sin(deg2rad(sec.stagger.value));
        const float cg = std::cos(deg2rad(sec.stagger.value));

        std::vector<uint32_t> loop;
        loop.reserve(n);
        for (const math::Vec2f& p : profile) {
            const float p_ax  = p.x * cg - p.y * sg;   // along chord (meridional)
            const float p_tan = p.x * sg + p.y * cg;   // tangential (r * theta)

            const math::Vec2f M = le + u * p_ax;       // meridional (z, r)
            float r = M.y;
            if (f >= 1.0f && row.tip_feature == TipFeature::Clearance)
                r -= flow.tip_clearance.value;

            const float z = -((M.x + row.sweep.value * f) - center);
            const float theta = (p_tan + row.lean.value * f) / std::max(r, 1e-6f);

            const math::Vec3f pos{ r * std::cos(theta), r * std::sin(theta), z };
            loop.push_back(acc.add(pos));
        }
        loops.push_back(std::move(loop));
    }

    // Skin between consecutive section loops (identical point count).
    for (std::size_t i = 0; i + 1 < loops.size(); ++i) {
        for (uint32_t j = 0; j < n; ++j) {
            const uint32_t j1 = (j + 1) % n;
            acc.quad(loops[i][j], loops[i][j1], loops[i + 1][j1], loops[i + 1][j]);
            if (info) info->skinPerBlade += 2;
        }
    }

    // End caps (hub + shroud) via centroid triangle fan.
    std::size_t capLoopIdx = 0;
    for (const std::vector<uint32_t>& loop : {loops.front(), loops.back()}) {
        math::Vec3f centroid{0.0f, 0.0f, 0.0f};
        for (const uint32_t v : loop) centroid += acc.pos[v];
        centroid = centroid / static_cast<float>(loop.size());
        const uint32_t c = acc.add(centroid);
        for (uint32_t j = 0; j < n; ++j) {
            const uint32_t j1 = (j + 1) % n;
            acc.tri(c, loop[j1], loop[j]);
            if (capLoopIdx == 0) {
                if (info) ++(info->hubCapPerBlade);
            } else {
                if (info) ++(info->shroudCapPerBlade);
            }
        }
        ++capLoopIdx;
    }
    if (info)
        info->stridePerBlade = info->skinPerBlade + info->hubCapPerBlade + info->shroudCapPerBlade;

    MeshData blade = acc.build();

    std::vector<MeshData> blades;
    blades.reserve(Z);
    blades.push_back(std::move(blade));
    for (uint32_t k = 1; k < Z; ++k) {
        const float angle = kTwoPi * static_cast<float>(k) / static_cast<float>(Z);
        const math::Quat rot = math::Quat::from_axis_angle({0.0f, 0.0f, 1.0f}, angle);
        blades.push_back(transform_mesh(blades.front(),
                                        math::Mat4::trs({0.0f, 0.0f, 0.0f}, rot, {1.0f, 1.0f, 1.0f})));
    }
    return merge_meshes(blades);
}

const char* blade_row_role_name(BladeRowType type)
{
    static const char* names[] = {"stator", "rotor", "nozzle", "diffuser"};
    const int i = static_cast<int>(type);
    if (i < 0 || i >= static_cast<int>(sizeof(names) / sizeof(names[0]))) return "unknown";
    return names[i];
}

} // namespace detail

MeshData generate_flow_path_mesh(const FlowPath& flow, uint32_t revolve_segments)
{
    if (flow.hub_points.size() < 2 || flow.shroud_points.size() < 2)
        return {};

    // No sampleable overlap between the hub and shroud z-ranges → empty.
    const MonotoneCubicSpline hub_domain = spline_of(flow.hub_points);
    const MonotoneCubicSpline shroud_domain = spline_of(flow.shroud_points);
    const float zmin = std::max(hub_domain.min_x(), shroud_domain.min_x());
    const float zmax = std::min(hub_domain.max_x(), shroud_domain.max_x());
    if (zmax <= zmin) return {};

    const std::vector<math::Vec3f> hub_profile = detail::build_meridional_profile(flow.hub_points);
    const std::vector<math::Vec3f> shroud_profile = detail::build_meridional_profile(flow.shroud_points);
    if (hub_profile.size() < 2 || shroud_profile.size() < 2) return {};

    LatheGeometry hub_geom;
    hub_geom.profile  = hub_profile;
    hub_geom.axis     = LatheAxis::Z;
    hub_geom.segments = revolve_segments;
    hub_geom.capped   = false;

    LatheGeometry shroud_geom = hub_geom;
    shroud_geom.profile = shroud_profile;

    std::array<MeshData, 2> parts{generate_lathe_mesh(hub_geom), generate_lathe_mesh(shroud_geom)};
    return merge_meshes(parts);
}

std::vector<math::Vec2f> generate_blade_section_profile(
    const BladeSection& section, float chord_length, uint32_t points)
{
    const uint32_t n = std::max(12u, points);
    const uint32_t half = n / 2u;

    MonotoneCubicSpline camber_spline;
    if (section.camber_line.size() >= 2)
        camber_spline = spline_of(section.camber_line);

    const float inlet_rel = deg2rad(section.inlet_metal_angle.value - section.stagger.value);
    const float exit_rel  = deg2rad(section.exit_metal_angle.value - section.stagger.value);
    Camber camber;
    camber.s0 = std::tan(inlet_rel);
    camber.s1 = std::tan(exit_rel);

    const float t_over_c = section.max_thickness.value;

    auto camber_at = [&](float x) -> float {
        return camber_spline.valid() ? camber_spline.evaluate(x) : camber(x);
    };
    auto thick_at = [&](float x) -> float {
        if (section.thickness_distribution.size() >= 2)
            return spline_of(section.thickness_distribution).evaluate(x);
        return naca_thickness(x, t_over_c);
    };

    std::vector<math::Vec2f> loop;
    loop.reserve(2u * half);

    loop.push_back({1.0f, camber_at(1.0f)});                       // TE
    for (uint32_t k = 1; k < half; ++k) {
        const float x = 1.0f - static_cast<float>(k) / static_cast<float>(half);
        loop.push_back({x, camber_at(x) + 0.5f * thick_at(x)});   // upper
    }
    loop.push_back({0.0f, camber_at(0.0f)});                       // LE
    for (uint32_t k = 1; k < half; ++k) {
        const float x = static_cast<float>(k) / static_cast<float>(half);
        loop.push_back({x, camber_at(x) - 0.5f * thick_at(x)});   // lower
    }
    for (auto& p : loop) p = p * chord_length;
    return loop;
}

MeshData generate_blade_row_mesh(const BladeRow& row, const FlowPath& flow,
                                 uint32_t revolve_segments)
{
    return detail::build_blade_row_impl(row, flow, revolve_segments, nullptr);
}

MeshData generate_hub_mesh(const HubDefinition& hub_cfg, uint32_t revolve_segments)
{
    if (hub_cfg.shape == HubShape::None) return {};

    const float R  = std::max(hub_cfg.root_radius, 1e-4f);
    const float Lf = std::max(hub_cfg.front_length, 0.0f);
    const float La = std::max(hub_cfg.aft_length, 0.0f);
    if (Lf <= 0.0f && La <= 0.0f) return {};

    const uint32_t n = std::max(4u, hub_cfg.profile_points);

    // Meridional profile (x = r, y = z), ordered aft -> front so the lathe
    // puts the flat cap on the aft end (-Z) and the nose toward +Z.
    std::vector<math::Vec3f> profile;

    switch (hub_cfg.shape) {
    case HubShape::Cylinder: {
        profile.push_back({R, -La, 0.0f});
        profile.push_back({R,  Lf, 0.0f});
        break;
    }
    case HubShape::FlatDisk: {
        const float t = std::clamp(0.06f * R, 0.02f, 0.25f);
        profile.push_back({R, -t, 0.0f});
        profile.push_back({R,  t, 0.0f});
        break;
    }
    case HubShape::Spinner:
    case HubShape::Bullet: {
        // Body: straight cylinder from the aft end to the rotor plane.
        if (La > 0.0f) profile.push_back({R, -La, 0.0f});
        profile.push_back({R, 0.0f, 0.0f});
        // Nose: ellipsoid dome (Spinner) or power-law ogive (Bullet).
        if (Lf > 0.0f) {
            const float p = (hub_cfg.shape == HubShape::Bullet)
                                ? std::clamp(hub_cfg.nose_power, 0.2f, 1.0f)
                                : 2.0f;   // squared = half-ellipse for Spinner
            for (uint32_t i = 1; i <= n; ++i) {
                const float f = static_cast<float>(i) / static_cast<float>(n);
                const float z = Lf * f;
                const float r = (hub_cfg.shape == HubShape::Bullet)
                                    ? R * std::pow(1.0f - f, p)
                                    : R * std::sqrt(1.0f - f * f);
                profile.push_back({r, z, 0.0f});
            }
        }
        break;
    }
    case HubShape::Tapered: {
        // Aft taper from the trailing end (aft_radius, 0 = point) to the
        // rotor plane at root radius.
        if (La > 0.0f) {
            for (uint32_t i = 0; i < n; ++i) {
                const float f = static_cast<float>(i) / static_cast<float>(n);
                profile.push_back({R + (hub_cfg.aft_radius - R) * f, -La + La * f, 0.0f});
            }
        }
        profile.push_back({R, 0.0f, 0.0f});
        // Nose taper to a point at +Lf.
        if (Lf > 0.0f) {
            for (uint32_t i = 1; i <= n; ++i) {
                const float f = static_cast<float>(i) / static_cast<float>(n);
                profile.push_back({R * (1.0f - f), Lf * f, 0.0f});
            }
        }
        break;
    }
    case HubShape::None:
    default:
        return {};
    }

    LatheGeometry lathe;
    lathe.profile  = std::move(profile);
    lathe.axis     = LatheAxis::Z;
    lathe.segments = std::max(8u, revolve_segments);
    lathe.capped   = true;
    return generate_lathe_mesh(lathe);
}

MeshData generate_turbine_mesh(const TurbineDefinition& turbine)
{
    std::vector<MeshData> parts;
    if (turbine.hub.shape != HubShape::None) {
        MeshData hub = generate_hub_mesh(turbine.hub, turbine.revolve_segments);
        if (!hub.vertices.empty()) parts.push_back(std::move(hub));
    }
    MeshData flow_path = generate_flow_path_mesh(turbine.flow_path, turbine.revolve_segments);
    if (!flow_path.vertices.empty()) parts.push_back(std::move(flow_path));
    for (const BladeRow& row : turbine.blade_rows) {
        MeshData row_mesh = generate_blade_row_mesh(row, turbine.flow_path, turbine.revolve_segments);
        if (!row_mesh.vertices.empty()) parts.push_back(std::move(row_mesh));
    }
    if (parts.empty()) return {};
    return merge_meshes(parts);
}

Assembly generate_turbine_assembly(const TurbineDefinition& turbine)
{
    Assembly asm_;
    auto push_part = [&](std::string name, MeshData mesh) {
        if (mesh.vertices.empty()) return;
        Part p = as_part(std::move(name), std::move(mesh));
        if (!p.mesh.vertices.empty())
            p.patches.push_back(make_patch_range("surface", 0, uint32_t(p.mesh.indices.size() / 3)));
        asm_.parts.push_back(std::move(p));
    };

    push_part("hub", generate_hub_mesh(turbine.hub, turbine.revolve_segments));
    push_part("flow_path", generate_flow_path_mesh(turbine.flow_path, turbine.revolve_segments));

    for (size_t i = 0; i < turbine.blade_rows.size(); ++i) {
        const BladeRow& row = turbine.blade_rows[i];
        detail::BladeRowBuildInfo info;
        MeshData mesh = detail::build_blade_row_impl(row, turbine.flow_path,
                                                     turbine.revolve_segments, &info);
        if (mesh.vertices.empty()) continue;

        Part p = as_part(std::string(detail::blade_row_role_name(row.type)) + "_" + std::to_string(i),
                         std::move(mesh));

        // Each blade in the row contributes (skin + hub + shroud) triangles in
        // fixed sub-ranges: [skin), [hub), [shroud).
        const uint32_t stride = info.stridePerBlade;
        const uint32_t blades = uint32_t(p.mesh.indices.size() / 3) / stride;
        std::vector<uint32_t> skinFaces, hubFaces, shroudFaces;
        for (uint32_t k = 0; k < blades; ++k) {
            for (uint32_t f = 0; f < info.skinPerBlade;   ++f) skinFaces.push_back(k * stride + f);
            for (uint32_t f = 0; f < info.hubCapPerBlade; ++f) hubFaces.push_back(k * stride + info.skinPerBlade + f);
            for (uint32_t f = 0; f < info.shroudCapPerBlade; ++f) shroudFaces.push_back(k * stride + info.skinPerBlade + info.hubCapPerBlade + f);
        }
        p.patches.push_back({"blade_surface", std::move(skinFaces)});
        p.patches.push_back({"hub_cap", std::move(hubFaces)});
        p.patches.push_back({"shroud_cap", std::move(shroudFaces)});
        asm_.parts.push_back(std::move(p));
    }

    // Union of bounds over all part meshes (skip empty meshes; empty → Bounds{}).
    asm_.bounds = {};
    bool have = false;
    for (const Part& part : asm_.parts) {
        if (part.mesh.vertices.empty()) continue;
        const Bounds b = compute_bounds(part.mesh.vertices);
        if (!have) {
            asm_.bounds = b;
            have = true;
        } else {
            asm_.bounds.min.x = std::min(asm_.bounds.min.x, b.min.x);
            asm_.bounds.min.y = std::min(asm_.bounds.min.y, b.min.y);
            asm_.bounds.min.z = std::min(asm_.bounds.min.z, b.min.z);
            asm_.bounds.max.x = std::max(asm_.bounds.max.x, b.max.x);
            asm_.bounds.max.y = std::max(asm_.bounds.max.y, b.max.y);
            asm_.bounds.max.z = std::max(asm_.bounds.max.z, b.max.z);
        }
    }

    return asm_;
}

} // namespace exd::geometry

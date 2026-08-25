#include <exd/geometry/turbine.hpp>

#include <exd/geometry/extrusion.hpp>
#include <exd/geometry/mesh_ops.hpp>
#include <exd/geometry/spline.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <cstdint>
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

MeshData generate_flow_path_mesh(const FlowPath& flow, uint32_t revolve_segments)
{
    if (flow.hub_points.size() < 2 || flow.shroud_points.size() < 2)
        return {};

    const MonotoneCubicSpline hub = spline_of(flow.hub_points);
    const MonotoneCubicSpline shroud = spline_of(flow.shroud_points);

    const float zmin = std::max(hub.min_x(), shroud.min_x());
    const float zmax = std::min(hub.max_x(), shroud.max_x());
    if (zmax <= zmin) return {};

    const float center = axial_center(flow);
    const uint32_t steps = std::max(8u, revolve_segments / 2u);
    auto sample = [&](const MonotoneCubicSpline& s) {
        std::vector<math::Vec3f> profile;
        profile.reserve(steps + 1);
        for (uint32_t i = 0; i <= steps; ++i) {
            const float z = zmin + (zmax - zmin) * static_cast<float>(i) / static_cast<float>(steps);
            // x = r (revolve radius), y = axial, negated + centered on origin.
            profile.push_back({s.evaluate(z), -(z - center), 0.0f});
        }
        return profile;
    };

    LatheGeometry hub_geom;
    hub_geom.profile  = sample(hub);
    hub_geom.axis     = LatheAxis::Z;
    hub_geom.segments = revolve_segments;
    hub_geom.capped   = false;

    LatheGeometry shroud_geom = hub_geom;
    shroud_geom.profile = sample(shroud);

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
        }
    }

    // End caps (hub + shroud) via centroid triangle fan.
    for (const std::vector<uint32_t>& loop : {loops.front(), loops.back()}) {
        math::Vec3f centroid{0.0f, 0.0f, 0.0f};
        for (const uint32_t v : loop) centroid += acc.pos[v];
        centroid = centroid / static_cast<float>(loop.size());
        const uint32_t c = acc.add(centroid);
        for (uint32_t j = 0; j < n; ++j) {
            const uint32_t j1 = (j + 1) % n;
            acc.tri(c, loop[j1], loop[j]);
        }
    }

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

MeshData generate_turbine_mesh(const TurbineDefinition& turbine)
{
    std::vector<MeshData> parts;
    MeshData flow_path = generate_flow_path_mesh(turbine.flow_path, turbine.revolve_segments);
    if (!flow_path.vertices.empty()) parts.push_back(std::move(flow_path));
    for (const BladeRow& row : turbine.blade_rows) {
        MeshData row_mesh = generate_blade_row_mesh(row, turbine.flow_path, turbine.revolve_segments);
        if (!row_mesh.vertices.empty()) parts.push_back(std::move(row_mesh));
    }
    if (parts.empty()) return {};
    return merge_meshes(parts);
}

} // namespace exd::geometry

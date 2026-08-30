#include <exd/geometry/steam_engine.hpp>

#include <exd/geometry/extrusion.hpp>
#include <exd/geometry/mesh_builder.hpp>
#include <exd/geometry/mesh_ops.hpp>
#include <exd/geometry/primitives3d.hpp>

#include <exd/math/mat4.hpp>
#include <exd/math/quat.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace exd::geometry
{
namespace
{

constexpr float kPi      = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;

/// Revolve a profile (points (radius, axial)) about X, NO end caps. The
/// lathe's own caps are deliberately not used: for LatheAxis::Y the cap fans
/// traverse the join rings in the SAME direction as the side surface
/// (verified empirically at SEG=64 against the directed-edge parity check),
/// which makes capped Y-lathes orientation-inconsistent. The recipe builds
/// every closure with explicit winding-aware fans (axis_fan) instead.
MeshData lathe_x_open(std::vector<math::Vec3f> profile, uint32_t segments)
{
    LatheGeometry g;
    g.profile  = std::move(profile);
    g.axis     = LatheAxis::X;
    g.segments = std::max(3u, segments);
    g.capped   = false;
    return generate_lathe_mesh(g);
}

/// Revolve a profile (points (radius, axial)) about Y, NO end caps.
MeshData lathe_y_open(std::vector<math::Vec3f> profile, uint32_t segments)
{
    LatheGeometry g;
    g.profile  = std::move(profile);
    g.axis     = LatheAxis::Y;
    g.segments = std::max(3u, segments);
    g.capped   = false;
    return generate_lathe_mesh(g);
}

/// Revolve a profile (points (radius, axial)) about Z, NO end caps.
MeshData lathe_z_open(std::vector<math::Vec3f> profile, uint32_t segments)
{
    LatheGeometry g;
    g.profile  = std::move(profile);
    g.axis     = LatheAxis::Z;
    g.segments = std::max(3u, segments);
    g.capped   = false;
    return generate_lathe_mesh(g);
}

/// Axis-closing disc fan with a single centre vertex on the revolve axis.
///
/// Winding contract (empirical, verified against the directed-edge parity
/// check at SEG=64 for every join in this recipe): the fan that closes a
/// piece's ring must traverse that ring OPPOSITE to the piece's own
/// traversal — every shared edge must then appear exactly once in each
/// direction. `winding` is the fan's traversal (+1 = (C, P_i, P_{i+1}),
/// −1 = reversed); `normalSign` only authors the per-vertex axial normal
/// (outward ±axis): rendering uses normals, the watertight gate checks
/// winding only.
MeshData axis_fan(LatheAxis axis, float axial, float radius, float normalSign,
                  float winding, uint32_t segments)
{
    const math::Quat white{1.0f, 1.0f, 1.0f, 1.0f};
    MeshBuilder b;
    b.reserve(segments + 1, segments * 3);

    math::Vec3f axisN;
    if (axis == LatheAxis::X)      axisN = {normalSign, 0.0f, 0.0f};
    else if (axis == LatheAxis::Y) axisN = {0.0f, normalSign, 0.0f};
    else if (axis == LatheAxis::Z) axisN = {0.0f, 0.0f, normalSign};
    else return {};

    Vertex center;
    center.color  = white;
    center.normal = axisN;
    if (axis == LatheAxis::X)      center.position = {axial, 0.0f, 0.0f};
    else if (axis == LatheAxis::Y) center.position = {0.0f, axial, 0.0f};
    else                           center.position = {0.0f, 0.0f, axial};
    const uint32_t c = b.add_vertex(center);

    std::vector<uint32_t> ring;
    ring.reserve(segments);
    for (uint32_t i = 0; i < segments; ++i)
    {
        const float a = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
        Vertex v;
        v.color  = white;
        v.normal = axisN;
        if (axis == LatheAxis::X)
            v.position = {axial, radius * std::cos(a), radius * std::sin(a)};
        else if (axis == LatheAxis::Y)
            v.position = {radius * std::cos(a), axial, radius * std::sin(a)};
        else
            v.position = {radius * std::cos(a), radius * std::sin(a), axial};
        ring.push_back(b.add_vertex(v));
    }
    for (uint32_t i = 0; i < segments; ++i)
    {
        const uint32_t j = (i + 1) % segments;
        if (winding > 0.0f)
            b.add_triangle(c, ring[i], ring[j]);
        else
            b.add_triangle(c, ring[j], ring[i]);
    }
    MeshData m = b.build();
    m.bounds = compute_bounds(m.vertices);
    return m;
}

/// Reverse winding (swap two indices per triangle) and resmooth normals.
/// Used ONLY where a lathe piece must present reversed rings to its join
/// partners (the chamber bore wall: −r normals AND ring senses that oppose
/// the crank annulus and the chamber bottom).
MeshData flip_and_resmooth(const MeshData& mesh)
{
    MeshData out = mesh;
    for (size_t i = 0; i + 2 < out.indices.size(); i += 3)
        std::swap(out.indices[i], out.indices[i + 1]);
    return recompute_normals(out, NormalMode::Smooth);
}

/// Negate vertex normals WITHOUT touching winding: used for surfaces whose
/// winding is condition-correct but whose lathe-authored normal points the
/// wrong way (the piston step ring: profile decreasing radius → −x normal,
/// outward is +x).
MeshData negate_normals(MeshData mesh)
{
    for (Vertex& v : mesh.vertices)
        v.normal = -v.normal;
    return mesh;
}

Patch patch_range(std::string name, uint32_t firstFace, uint32_t faceCount)
{
    Patch p;
    p.name = std::move(name);
    p.faces.reserve(faceCount);
    for (uint32_t f = firstFace; f < firstFace + faceCount; ++f)
        p.faces.push_back(f);
    return p;
}

Patch patch_ranges(std::string name, std::vector<std::pair<uint32_t, uint32_t>> ranges)
{
    Patch p;
    p.name = std::move(name);
    for (const auto& [first, count] : ranges)
        for (uint32_t f = first; f < first + count; ++f)
            p.faces.push_back(f);
    return p;
}

std::vector<math::Vec3f> circle_outline(float radius, uint32_t points)
{
    std::vector<math::Vec3f> ring;
    ring.reserve(points);
    for (uint32_t i = 0; i < points; ++i)
    {
        const float a = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(points);
        ring.push_back({radius * std::cos(a), radius * std::sin(a), 0.0f});
    }
    return ring;
}

math::Mat4 translate_only(const math::Vec3f& t)
{
    return math::Mat4::trs(t, math::Quat{1.0f, 0.0f, 0.0f, 0.0f},
                           math::Vec3f{1.0f, 1.0f, 1.0f});
}

void accumulate_bounds(Assembly& a)
{
    a.bounds = {};
    bool have = false;
    for (const Part& part : a.parts)
    {
        if (part.mesh.vertices.empty()) continue;
        const Bounds b = compute_bounds(part.mesh.vertices);
        if (!have)
        {
            a.bounds = b;
            have = true;
        }
        else
        {
            a.bounds.min.x = std::min(a.bounds.min.x, b.min.x);
            a.bounds.min.y = std::min(a.bounds.min.y, b.min.y);
            a.bounds.min.z = std::min(a.bounds.min.z, b.min.z);
            a.bounds.max.x = std::max(a.bounds.max.x, b.max.x);
            a.bounds.max.y = std::max(a.bounds.max.y, b.max.y);
            a.bounds.max.z = std::max(a.bounds.max.z, b.max.z);
        }
    }
}

/// Tri count per 1-segment lathe piece (2 * SEG).
uint32_t side_faces(uint32_t seg) { return 2u * seg; }

/// Weld a set of pieces into one part (ordinal-preserving, patch-safe).
Part weld_part(std::string name, const std::vector<MeshData>& pieces)
{
    return as_part(std::move(name), weld_vertices(merge_meshes(pieces), 1e-4f));
}

} // namespace

Assembly generate_steam_engine_assembly(const SteamEngineDefinition& def)
{
    Assembly a;

    // ── Validation (empty result on impossible geometry, repo convention) ──
    const float rc = def.crank_radius;
    const bool finiteState = std::isfinite(def.crank_angle_deg) &&
                             std::isfinite(def.crank_radius) &&
                             std::isfinite(def.conrod_length) &&
                             std::isfinite(def.piston_rod_length);
    if (!finiteState) return a;
    if (!(rc > 0.0f)) return a;
    if (!(def.conrod_length > rc)) return a;
    if (!(def.cylinder_outer_radius > 0.0f)) return a;
    if (!(def.cylinder_bore_radius > 0.0f &&
          def.cylinder_bore_radius < def.cylinder_outer_radius)) return a;
    if (!(def.piston_radius > 0.0f && def.piston_radius < def.cylinder_bore_radius)) return a;
    if (!(def.piston_rod_length > def.piston_length)) return a;
    if (!(def.flywheel_rim_radius > 0.0f &&
          def.flywheel_groove_radius > 0.0f &&
          def.flywheel_groove_radius < def.flywheel_rim_radius)) return a;
    if (!(def.rod_plane_z > def.pin_z_start + def.pin_length)) return a;   // pin never pierces the rod

    const uint32_t SEG = std::max(3u, def.revolve_segments);
    const float    theta = def.crank_angle_deg * kDeg2Rad;
    const float    ct    = std::cos(theta);
    const float    st    = std::sin(theta);

    // ── Mechanism (inline crank-slider, zero offset, evaluated here) ──
    const float x_cc = def.crank_center_x;
    const float x_c  = x_cc + rc * ct + std::sqrt(def.conrod_length * def.conrod_length
                                                  - rc * rc * st * st);
    const float x_pk = x_c - def.piston_rod_length;
    const float x_ce = def.cylinder_crank_end_x;
    const float x_h  = x_ce - def.cylinder_length;
    const float z_rod = def.rod_plane_z;
    const float pin_x = x_cc + rc * ct;
    const float pin_y = rc * st;
    const float chamber_bottom = x_h + 0.02f;   // blind bore recess depth

    // ── cylinder ──────────────────────────────────────────────────────────
    // Closed manifold with a blind bore chamber; the chamber mouth (r < bore
    // at the crank end) is an EXTERIOR region (torus-hole style), so the
    // surface has no boundary loops. The piston rod exits through the mouth
    // as a separate, interpenetrating part — assembly placement, not CSG.
    {
        // Winding bookkeeping (validated empirically — every join below
        // passes the 1:1 opposing-edge check, see tests):
        //   outer wall:  [(outer,x_ce)→(outer,x_h)]  reversed profile order
        //   head disc:   fan winding +1, normal −x          (closes outer wall @ x_h)
        //   crank ann.:  [(bore,x_ce)→(outer,x_ce)] normal +x (closes outer wall @ x_ce
        //                on its outer ring; the bore wall is flipped to oppose it)
        //   bore wall:   [(bore,x_ce)→(bore,bottom)] flipped → −r normals (chamber wall)
        //   chamber bot: fan winding −1, normal +x          (closes bore wall @ bottom)
        std::vector<MeshData> pieces;
        pieces.push_back(lathe_x_open({{def.cylinder_outer_radius, x_ce},
                                       {def.cylinder_outer_radius, x_h}}, SEG));
        pieces.push_back(axis_fan(LatheAxis::X, x_h, def.cylinder_outer_radius,
                                  -1.0f, 1.0f, SEG));
        pieces.push_back(lathe_x_open({{def.cylinder_bore_radius, x_ce},
                                       {def.cylinder_outer_radius, x_ce}}, SEG));
        pieces.push_back(flip_and_resmooth(lathe_x_open({{def.cylinder_bore_radius, x_ce},
                                                         {def.cylinder_bore_radius, chamber_bottom}},
                                                        SEG)));
        pieces.push_back(axis_fan(LatheAxis::X, chamber_bottom, def.cylinder_bore_radius,
                                  +1.0f, -1.0f, SEG));
        Part p = weld_part("cylinder", pieces);
        // layout: wall n, head fan SEG, crank ann n, bore wall n, chamber fan SEG
        const uint32_t n = side_faces(SEG);
        p.patches.push_back(patch_range("wall",      0u,               n));
        p.patches.push_back(patch_range("cap_head",  n,                uint32_t(SEG)));
        p.patches.push_back(patch_range("cap_crank", n + SEG,          n));
        p.patches.push_back(patch_ranges("bore", {{2 * n + SEG, n}, {3 * n + SEG, uint32_t(SEG)}}));
        a.parts.push_back(transform_part(p, translate_only({0.0f, 0.0f, z_rod})));
    }

    // ── steam chest (box on the cylinder top) ─────────────────────────────
    {
        BoxGeometry chest;
        chest.size = {def.chest_width, def.chest_height, def.chest_depth};
        Part p = generate_box_part(chest);
        p.mesh = weld_vertices(p.mesh, 1e-4f);
        p.name = "steam_chest";
        const float chest_y = def.cylinder_outer_radius + 0.5f * def.chest_height;
        a.parts.push_back(transform_part(p, translate_only({def.chest_x_center, chest_y, z_rod})));
    }

    // ── steam inlet / exhaust stubs (closed; no lathe caps) ────────────────
    {
        const float y0 = def.cylinder_outer_radius + def.chest_height;
        const float y1 = y0 + def.port_height;
        const float xs[2]{def.chest_x_center - 0.5f * def.port_spacing,
                          def.chest_x_center + 0.5f * def.port_spacing};
        const char* names[2]{"steam_inlet", "steam_exhaust"};
        for (int i = 0; i < 2; ++i)
        {
            std::vector<MeshData> pieces;
            pieces.push_back(lathe_y_open({{def.port_radius, y0}, {def.port_radius, y1}}, SEG));
            pieces.push_back(axis_fan(LatheAxis::Y, y0, def.port_radius, -1.0f, -1.0f, SEG));
            pieces.push_back(axis_fan(LatheAxis::Y, y1, def.port_radius, +1.0f, +1.0f, SEG));
            Part p = weld_part(names[i], pieces);
            const uint32_t n = side_faces(SEG);
            p.patches.push_back(patch_range("surface", 0, n));
            p.patches.push_back(patch_range("cap_start", n, uint32_t(SEG)));
            p.patches.push_back(patch_range("cap_end", n + SEG, uint32_t(SEG)));
            a.parts.push_back(transform_part(p, translate_only({xs[i], 0.0f, z_rod})));
        }
    }

    // ── piston (crown + body + step ring + rod to the crosshead) ───────────
    {
        const float x_step = x_pk + def.piston_length;
        std::vector<MeshData> pieces;
        pieces.push_back(axis_fan(LatheAxis::X, x_pk, def.piston_radius, -1.0f, -1.0f, SEG));
        pieces.push_back(lathe_x_open({{def.piston_radius, x_pk},
                                       {def.piston_radius, x_step}}, SEG));
        pieces.push_back(negate_normals(lathe_x_open({{def.piston_radius, x_step},
                                                      {def.rod_radius, x_step}}, SEG)));
        pieces.push_back(lathe_x_open({{def.rod_radius, x_step},
                                       {def.rod_radius, x_c}}, SEG));
        pieces.push_back(axis_fan(LatheAxis::X, x_c, def.rod_radius, +1.0f, +1.0f, SEG));
        Part p = weld_part("piston", pieces);
        const uint32_t n = side_faces(SEG);
        p.patches.push_back(patch_range("crown", 0u, uint32_t(SEG)));
        p.patches.push_back(patch_range("wall",  uint32_t(SEG), 3u * n + uint32_t(SEG)));
        a.parts.push_back(transform_part(p, translate_only({0.0f, 0.0f, z_rod})));
    }

    // ── crosshead (sliding block at x_c) ─────────────────────────────────
    {
        BoxGeometry cross;
        cross.size = {def.crosshead_size, def.crosshead_size, def.crosshead_thickness};
        Part p = generate_box_part(cross);
        p.mesh = weld_vertices(p.mesh, 1e-4f);
        p.name = "crosshead";
        a.parts.push_back(transform_part(p, translate_only({x_c, 0.0f, z_rod})));
    }

    // ── conrod (crosshead → crank pin, posed by an orientation quaternion) ──
    {
        ExtrusionGeometry rod;
        rod.profile = circle_outline(def.conrod_radius, 24);
        rod.depth   = def.conrod_length;
        rod.capped  = false;
        std::vector<MeshData> rodPieces;
        rodPieces.push_back(generate_extrusion_mesh(rod));
        // extrusion wall rings sit at ±depth/2 and traverse −θ (front, z<0)
        // and +θ (back, z>0) — the OPPOSITE of the lathe convention, so the
        // closures must oppose those directions
        rodPieces.push_back(axis_fan(LatheAxis::Z, -0.5f * rod.depth, def.conrod_radius,
                                     -1.0f, +1.0f, 24));
        rodPieces.push_back(axis_fan(LatheAxis::Z, +0.5f * rod.depth, def.conrod_radius,
                                     +1.0f, -1.0f, 24));
        Part p = weld_part("conrod", rodPieces);
        p.patches.push_back(patch_range("wall",      0u,           2u * 24u));
        p.patches.push_back(patch_range("cap_start", 2u * 24u,     24u));
        p.patches.push_back(patch_range("cap_end",   3u * 24u,     24u));

        const float pin_z = def.pin_z_start + 0.5f * def.pin_length;
        const float dx = pin_x - x_c;
        const float dy = pin_y;
        const float dz = pin_z - z_rod;     // rod plane lifted clear of the pin
        const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len > 1e-9f)
        {
            // Rotate local +Z onto d̂ = (dx,dy,dz)/len: axis = ẑ×d̂, angle =
            // acos(d̂·ẑ) — valid for the skewed (out-of-plane) conrod.
            const float nx = -dy / len;
            const float ny =  dx / len;
            const float ang = std::acos(dz / len);
            const math::Quat q = math::Quat::from_axis_angle({nx, ny, 0.0f}, ang);
            // The extrusion spans local z ∈ [−L/2, +L/2]; translate so the
            // small end lands exactly at the crosshead and the big end at
            // the pin: T = crosshead + ½·(pin − crosshead).
            a.parts.push_back(transform_part(p, math::Mat4::trs(
                {x_c + 0.5f * dx, 0.5f * dy, z_rod + 0.5f * dz}, q,
                math::Vec3f{1.0f, 1.0f, 1.0f})));
        }
    }

    // ── flywheel (V-groove pulley disc at the crank centre, the power takeoff)
    {
        const float t = 0.5f * def.flywheel_thickness;
        const float g = def.flywheel_groove_half_width;
        std::vector<MeshData> pieces;
        pieces.push_back(lathe_z_open({{def.flywheel_rim_radius, -t},
                                       {def.flywheel_rim_radius, -g},
                                       {def.flywheel_groove_radius, 0.0f},
                                       {def.flywheel_rim_radius, g},
                                       {def.flywheel_rim_radius, t}}, SEG));
        pieces.push_back(axis_fan(LatheAxis::Z, -t, def.flywheel_rim_radius, -1.0f, -1.0f, SEG));
        pieces.push_back(axis_fan(LatheAxis::Z,  t, def.flywheel_rim_radius, +1.0f, +1.0f, SEG));
        Part p = weld_part("flywheel", pieces);
        const uint32_t n = side_faces(SEG);
        p.patches.push_back(patch_range("rim",        0u,                 4u * n));
        p.patches.push_back(patch_range("face_start", 4u * n,              uint32_t(SEG)));
        p.patches.push_back(patch_range("face_end",   4u * n + SEG,        uint32_t(SEG)));
        a.parts.push_back(transform_part(p, translate_only({x_cc, 0.0f, 0.0f})));
    }

    // ── crank pin (orbits in the rod plane, pierces the flywheel face) ────
    {
        std::vector<MeshData> pieces;
        pieces.push_back(lathe_z_open({{def.pin_radius, def.pin_z_start},
                                       {def.pin_radius, def.pin_z_start + def.pin_length}}, SEG));
        pieces.push_back(axis_fan(LatheAxis::Z, def.pin_z_start, def.pin_radius, -1.0f, -1.0f, SEG));
        pieces.push_back(axis_fan(LatheAxis::Z, def.pin_z_start + def.pin_length,
                                  def.pin_radius, +1.0f, +1.0f, SEG));
        Part p = weld_part("crank_pin", pieces);
        const uint32_t n = side_faces(SEG);
        p.patches.push_back(patch_range("surface", 0, n));
        p.patches.push_back(patch_range("cap_start", n, uint32_t(SEG)));
        p.patches.push_back(patch_range("cap_end", n + SEG, uint32_t(SEG)));
        a.parts.push_back(transform_part(p, translate_only({pin_x, pin_y, 0.0f})));
    }

    // ── crankshaft (capped journal through the flywheel hub) ──────────────
    {
        std::vector<MeshData> pieces;
        pieces.push_back(lathe_z_open({{def.shaft_radius, -def.shaft_half_length},
                                       {def.shaft_radius,  def.shaft_half_length}}, SEG));
        pieces.push_back(axis_fan(LatheAxis::Z, -def.shaft_half_length, def.shaft_radius,
                                  -1.0f, -1.0f, SEG));
        pieces.push_back(axis_fan(LatheAxis::Z, def.shaft_half_length, def.shaft_radius,
                                  +1.0f, +1.0f, SEG));
        Part p = weld_part("crankshaft", pieces);
        const uint32_t n = side_faces(SEG);
        p.patches.push_back(patch_range("journal", 0, n));
        p.patches.push_back(patch_range("cap_start", n, uint32_t(SEG)));
        p.patches.push_back(patch_range("cap_end", n + SEG, uint32_t(SEG)));
        a.parts.push_back(transform_part(p, translate_only({x_cc, 0.0f, 0.0f})));
    }

    // ── Orientation canonicalization ──
    // The lathe's winding chirality is profile-order dependent, so parts built
    // with different profile orders can come out globally inside-out while
    // still passing the directed-edge gate (the boolean gate silently
    // normalizes). Downstream consumers (mass properties, backface culling,
    // outward-patch semantics) need outward winding: flip any part whose
    // signed volume is negative and resmooth normals from the winding.
    for (Part& part : a.parts)
    {
        if (part.mesh.vertices.empty()) continue;
        double volume = 0.0;
        const auto& m = part.mesh;
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3)
        {
            const auto& p0 = m.vertices[m.indices[i + 0]].position;
            const auto& p1 = m.vertices[m.indices[i + 1]].position;
            const auto& p2 = m.vertices[m.indices[i + 2]].position;
            volume += static_cast<double>(p0.x) * (static_cast<double>(p1.y) * p2.z -
                                                   static_cast<double>(p1.z) * p2.y) +
                      static_cast<double>(p0.y) * (static_cast<double>(p1.z) * p2.x -
                                                   static_cast<double>(p1.x) * p2.z) +
                      static_cast<double>(p0.z) * (static_cast<double>(p1.x) * p2.y -
                                                   static_cast<double>(p1.y) * p2.x);
        }
        if (volume < 0.0)
            part.mesh = flip_and_resmooth(part.mesh);
    }

    accumulate_bounds(a);
    return a;
}

MeshData generate_steam_engine_mesh(const SteamEngineDefinition& engine)
{
    const Assembly a = generate_steam_engine_assembly(engine);
    if (a.parts.empty()) return {};
    return flatten(a).mesh;
}

} // namespace exd::geometry

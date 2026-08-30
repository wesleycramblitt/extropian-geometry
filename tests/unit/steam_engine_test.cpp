#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

using namespace exd::geometry;

namespace
{

constexpr float kPi = 3.14159265358979323846f;

SteamEngineDefinition default_engine()
{
    SteamEngineDefinition def;
    return def;
}

bool has_patch(const Part& part, const std::string& name)
{
    for (const Patch& patch : part.patches)
        if (patch.name == name) return true;
    return false;
}

/// Expected crosshead x for the inline crank-slider at angle θ.
float crosshead_x(const SteamEngineDefinition& def, float theta_deg)
{
    const float t  = theta_deg * kPi / 180.0f;
    const float r  = def.crank_radius;
    const float L  = def.conrod_length;
    return def.crank_center_x + r * std::cos(t) + std::sqrt(L * L - r * r * std::sin(t) * std::sin(t));
}

/// Number of boundary edges: canonical undirected edges (min,max) appearing
/// in exactly ONE triangle. 0 → the surface is topologically closed.
uint32_t boundary_edges(const MeshData& mesh)
{
    std::unordered_map<uint64_t, uint32_t> counts;
    auto key = [](uint32_t a, uint32_t b)
    {
        const uint64_t lo = std::min(a, b), hi = std::max(a, b);
        return (lo << 32) | hi;
    };
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const uint32_t ia = mesh.indices[i + 0];
        const uint32_t ib = mesh.indices[i + 1];
        const uint32_t ic = mesh.indices[i + 2];
        counts[key(ia, ib)]++;
        counts[key(ib, ic)]++;
        counts[key(ic, ia)]++;
    }
    uint32_t nBoundary = 0;
    for (const auto& [e, c] : counts)
        if (c == 1) nBoundary++;
    return nBoundary;
}

} // namespace

TEST_CASE("steam engine: default machine parts") {
    const SteamEngineDefinition def = default_engine();
    const Assembly a = generate_steam_engine_assembly(def);
    REQUIRE(a.parts.size() == 10);

    const std::vector<std::string> names{
        "cylinder", "steam_chest", "steam_inlet", "steam_exhaust", "piston",
        "crosshead", "conrod", "flywheel", "crank_pin", "crankshaft"};
    for (std::size_t i = 0; i < names.size(); ++i)
        CHECK(a.parts[i].name == names[i]);

    for (const Part& part : a.parts)
        REQUIRE_FALSE(part.mesh.vertices.empty());

    // ── patches ──
    const Part& cylinder = a.parts[0];
    REQUIRE(has_patch(cylinder, "wall"));
    REQUIRE(has_patch(cylinder, "bore"));
    REQUIRE(has_patch(cylinder, "cap_head"));
    REQUIRE(has_patch(cylinder, "cap_crank"));

    const Part& piston = a.parts[4];
    REQUIRE(has_patch(piston, "crown"));
    REQUIRE(has_patch(piston, "wall"));

    const Part& flywheel = a.parts[7];
    REQUIRE(has_patch(flywheel, "rim"));
    REQUIRE(has_patch(flywheel, "face_start"));
    REQUIRE(has_patch(flywheel, "face_end"));

    for (const Part& part : a.parts)
    {
        // every face of every part belongs to exactly one patch
        const uint32_t total = static_cast<uint32_t>(part.mesh.indices.size() / 3);
        std::vector<uint32_t> seen(total, 0);
        for (const Patch& patch : part.patches)
            for (const uint32_t f : patch.faces)
            {
                REQUIRE(f < total);
                seen[f]++;
            }
        for (const uint32_t s : seen)
            CHECK(s == 1);
    }
}

TEST_CASE("steam engine: crank-slider kinematics") {
    const SteamEngineDefinition def = default_engine();
    for (const float theta : {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 270.0f})
    {
        SteamEngineDefinition d = def;
        d.crank_angle_deg = theta;
        const Assembly a = generate_steam_engine_assembly(d);
        REQUIRE(a.parts.size() == 10);

        const Part& crosshead = a.parts[5];
        const Part& piston    = a.parts[4];
        const Part& pin       = a.parts[8];

        const Bounds ch = compute_bounds(crosshead.mesh.vertices);
        const Bounds pi = compute_bounds(piston.mesh.vertices);
        const Bounds pn = compute_bounds(pin.mesh.vertices);

        const float xc = crosshead_x(def, theta);
        const float half = 0.5f * def.crosshead_size;
        CHECK(ch.min.x == doctest::Approx(xc - half).epsilon(1e-3f));
        CHECK(ch.max.x == doctest::Approx(xc + half).epsilon(1e-3f));
        CHECK(ch.min.z == doctest::Approx(def.rod_plane_z - 0.5f * def.crosshead_thickness).epsilon(1e-3f));

        // piston crown face at x_pk = x_c − rod length
        CHECK(pi.min.x == doctest::Approx(xc - def.piston_rod_length).epsilon(1e-3f));

        // pin centre at (crank_x + rc·cosθ, rc·sinθ); a capped cylinder's
        // extent in x/y is its radius (the caps close the z ends)
        const float t = theta * kPi / 180.0f;
        const float px = def.crank_center_x + def.crank_radius * std::cos(t);
        const float py = def.crank_radius * std::sin(t);
        CHECK(pn.min.x == doctest::Approx(px - def.pin_radius).epsilon(1e-3f));
        CHECK(pn.max.x == doctest::Approx(px + def.pin_radius).epsilon(1e-3f));
        CHECK(pn.min.y == doctest::Approx(py - def.pin_radius).epsilon(1e-3f));
        CHECK(pn.max.y == doctest::Approx(py + def.pin_radius).epsilon(1e-3f));
    }
}

TEST_CASE("steam engine: stroke spans two crank radii") {
    SteamEngineDefinition d0 = default_engine();
    d0.crank_angle_deg = 0.0f;
    SteamEngineDefinition d1 = default_engine();
    d1.crank_angle_deg = 180.0f;

    const Bounds b0 = compute_bounds(generate_steam_engine_assembly(d0).parts[4].mesh.vertices);
    const Bounds b1 = compute_bounds(generate_steam_engine_assembly(d1).parts[4].mesh.vertices);
    // crown travels 2·r along x between TDC and BDC
    CHECK(b0.min.x - b1.min.x == doctest::Approx(2.0f * d0.crank_radius).epsilon(1e-3f));
}

TEST_CASE("steam engine: parts are closed watertight") {
    const SteamEngineDefinition def = default_engine();
    const Assembly a = generate_steam_engine_assembly(def);
    REQUIRE(a.parts.size() == 10);

    for (const Part& part : a.parts)
    {
        CAPTURE(part.name);
        // every part is a closed manifold: no boundary edges. The cylinder's
        // bore-chamber mouth at the crank end is an EXTERIOR region (a
        // torus-hole style concavity), so it contributes no boundary loop.
        CHECK(boundary_edges(part.mesh) == 0);
    }
}

/// Signed volume of a welded part (positive = outward winding).
double signed_volume(const MeshData& m)
{
    double vol = 0.0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3)
    {
        const auto& p0 = m.vertices[m.indices[i + 0]].position;
        const auto& p1 = m.vertices[m.indices[i + 1]].position;
        const auto& p2 = m.vertices[m.indices[i + 2]].position;
        vol += static_cast<double>(p0.x) * (static_cast<double>(p1.y) * p2.z -
                                            static_cast<double>(p1.z) * p2.y) +
               static_cast<double>(p0.y) * (static_cast<double>(p1.z) * p2.x -
                                            static_cast<double>(p1.x) * p2.z) +
               static_cast<double>(p0.z) * (static_cast<double>(p1.x) * p2.y -
                                            static_cast<double>(p1.y) * p2.x);
    }
    return vol;
}

TEST_CASE("steam engine: parts pass the boolean gate, outward, across the stroke") {
    // Recipe contract: solid parts must survive the library's own
    // closed-manifold QA gate AND be outward-oriented (positive signed
    // volume — downstream mass properties / backface semantics rely on it).
    BoxGeometry probe;
    probe.size = {0.01f, 0.01f, 0.01f};
    const MeshData box = generate_box_mesh(probe);
    REQUIRE_FALSE(box.vertices.empty());

    for (const float theta : {0.0f, 90.0f, 180.0f, 270.0f})
    {
        SteamEngineDefinition d = default_engine();
        d.crank_angle_deg = theta;
        const Assembly a = generate_steam_engine_assembly(d);
        REQUIRE(a.parts.size() == 10);
        for (const Part& part : a.parts)
        {
            CAPTURE(theta);
            CAPTURE(part.name);
            const MeshData result = boolean_mesh(part.mesh, box, BooleanOp::Union);
            REQUIRE_FALSE(result.vertices.empty());
            CHECK(signed_volume(part.mesh) > 0.0);
        }
    }
}

TEST_CASE("steam engine: conrod reaches crosshead and pin") {
    for (const float theta : {0.0f, 45.0f, 90.0f, 180.0f, 270.0f})
    {
        SteamEngineDefinition d = default_engine();
        d.crank_angle_deg = theta;
        const Assembly a = generate_steam_engine_assembly(d);
        const Part& conrod  = a.parts[6];
        const Part& crosshead = a.parts[5];
        const Part& pin     = a.parts[8];

        // cap_start = small end (local −Z), cap_end = big end (+Z): their
        // centroids must sit on the crosshead and the pin respectively.
        exd::math::Vec3f startC{0.0f, 0.0f, 0.0f}, endC{0.0f, 0.0f, 0.0f};
        uint32_t sN = 0, eN = 0;
        for (const Patch& patch : conrod.patches)
        {
            if (patch.name == "cap_start")
                for (const uint32_t f : patch.faces)
                {
                    for (int k = 0; k < 3; ++k)
                        startC = startC + conrod.mesh.vertices[conrod.mesh.indices[f * 3 + k]].position;
                    sN += 3;
                }
            if (patch.name == "cap_end")
                for (const uint32_t f : patch.faces)
                {
                    for (int k = 0; k < 3; ++k)
                        endC = endC + conrod.mesh.vertices[conrod.mesh.indices[f * 3 + k]].position;
                    eN += 3;
                }
        }
        const exd::math::Vec3f s = sN ? startC * (1.0f / static_cast<float>(sN)) : exd::math::Vec3f{};
        const exd::math::Vec3f e = eN ? endC * (1.0f / static_cast<float>(eN)) : exd::math::Vec3f{};

        const Bounds ch = compute_bounds(crosshead.mesh.vertices);
        const Bounds pi = compute_bounds(pin.mesh.vertices);
        const float t = theta * kPi / 180.0f;
        const float px = d.crank_center_x + d.crank_radius * std::cos(t);
        const float py = d.crank_radius * std::sin(t);
        const float pz = d.pin_z_start + 0.5f * d.pin_length;
        const float pinErr = std::sqrt((e.x - px) * (e.x - px) + (e.y - py) * (e.y - py) +
                                       (e.z - pz) * (e.z - pz));
        CAPTURE(theta);
        // small end within the crosshead block
        CHECK(s.x >= ch.min.x - 0.01f);
        CHECK(s.x <= ch.max.x + 0.01f);
        CHECK(s.z >= ch.min.z - 0.01f);
        CHECK(s.z <= ch.max.z + 0.01f);
        // big end on the pin centerline: the conrod is conrod_length while
        // |pin − crosshead| varies ≈ ±0.2% over the stroke, and the fan
        // centroid averages the ring — 5 mm absolute covers both
        CHECK(pinErr < 0.005f);
    }
}

TEST_CASE("steam engine: piston stays inside the chamber") {
    for (const float theta : {0.0f, 90.0f, 180.0f, 270.0f})
    {
        SteamEngineDefinition d = default_engine();
        d.crank_angle_deg = theta;
        const Assembly a = generate_steam_engine_assembly(d);
        const Part& piston = a.parts[4];
        // body vertices (radius > rod radius): the rod itself extends to the
        // crosshead by design, so filter it out
        float minBodyX = std::numeric_limits<float>::max();
        float maxBodyX = -std::numeric_limits<float>::max();
        for (const Vertex& v : piston.mesh.vertices)
        {
            // radial distance about the cylinder axis (the part sits at z = rod_plane_z)
            const float dz = v.position.z - d.rod_plane_z;
            const float rr = std::sqrt(v.position.y * v.position.y + dz * dz);
            if (rr > d.rod_radius + 1e-4f)
            {
                minBodyX = std::min(minBodyX, v.position.x);
                maxBodyX = std::max(maxBodyX, v.position.x);
            }
        }
        // crown never reaches the blind-bore bottom
        const float bottom = d.cylinder_crank_end_x - d.cylinder_length + 0.02f;
        CHECK(minBodyX > bottom + 0.005f);
        // piston back never exits the bore mouth (crank-end plane)
        CHECK(maxBodyX < d.cylinder_crank_end_x - 0.005f);
    }
}

TEST_CASE("steam engine: deterministic generation") {
    const SteamEngineDefinition def = default_engine();
    const MeshData m1 = generate_steam_engine_mesh(def);
    const MeshData m2 = generate_steam_engine_mesh(def);
    REQUIRE_FALSE(m1.vertices.empty());
    CHECK(m1.vertices.size() == m2.vertices.size());
    CHECK(m1.indices.size() == m2.indices.size());
    REQUIRE(m1.vertices.size() == m2.vertices.size());
    for (std::size_t i = 0; i < m1.vertices.size(); ++i)
    {
        const exd::math::Vec3f& p1 = m1.vertices[i].position;
        const exd::math::Vec3f& p2 = m2.vertices[i].position;
        CHECK(p1.x == p2.x);
        CHECK(p1.y == p2.y);
        CHECK(p1.z == p2.z);
    }
    REQUIRE(m1.indices.size() == m2.indices.size());
    for (std::size_t i = 0; i < m1.indices.size(); ++i)
        CHECK(m1.indices[i] == m2.indices[i]);
}

TEST_CASE("steam engine: validation returns empty") {
    SteamEngineDefinition bad = default_engine();
    bad.conrod_length = 0.05f;   // <= crank_radius → mechanism cannot assemble
    const Assembly a = generate_steam_engine_assembly(bad);
    CHECK(a.parts.empty());
    CHECK(generate_steam_engine_mesh(bad).vertices.empty());

    SteamEngineDefinition bad2 = default_engine();
    bad2.piston_radius = 0.08f;  // >= bore radius
    CHECK(generate_steam_engine_assembly(bad2).parts.empty());

    SteamEngineDefinition bad3 = default_engine();
    bad3.cylinder_bore_radius = 0.12f;  // >= outer radius
    CHECK(generate_steam_engine_assembly(bad3).parts.empty());

    SteamEngineDefinition bad4 = default_engine();
    bad4.crank_radius = 0.0f;   // non-positive
    CHECK(generate_steam_engine_assembly(bad4).parts.empty());

    SteamEngineDefinition bad5 = default_engine();
    bad5.crank_angle_deg = std::numeric_limits<float>::quiet_NaN();  // non-finite state
    CHECK(generate_steam_engine_assembly(bad5).parts.empty());

    SteamEngineDefinition bad6 = default_engine();
    bad6.rod_plane_z = 0.05f;   // pin z span [0.03, 0.09] would cross the rod plane
    CHECK(generate_steam_engine_assembly(bad6).parts.empty());
}

TEST_CASE("steam engine: mesh convenience equals flattened assembly") {
    const SteamEngineDefinition def = default_engine();
    const Assembly a = generate_steam_engine_assembly(def);
    const MeshData mesh = generate_steam_engine_mesh(def);
    REQUIRE_FALSE(mesh.vertices.empty());
    const Part flat = flatten(a);
    CHECK(mesh.vertices.size() == flat.mesh.vertices.size());
    CHECK(mesh.indices.size() == flat.mesh.indices.size());
}

TEST_CASE("steam engine: bounds sane and cover the mechanism") {
    const SteamEngineDefinition def = default_engine();
    const Assembly a = generate_steam_engine_assembly(def);
    const Bounds& b = a.bounds;
    CHECK(b.min.x <= b.max.x);
    CHECK(b.min.y <= b.max.y);
    CHECK(b.min.z <= b.max.z);
    // flywheel rim radius is the widest reach in y
    CHECK(b.max.y >= def.flywheel_rim_radius - 1e-3f);
    CHECK(b.min.y <= -def.flywheel_rim_radius + 1e-3f);
    // crosshead reach at TDC
    CHECK(b.max.x >= crosshead_x(def, 0.0f) - 1e-3f);
    // cylinder head plane
    CHECK(b.min.x <= def.cylinder_crank_end_x - def.cylinder_length + 1e-3f);
    // shaft extent along z
    CHECK(b.max.z >= def.shaft_half_length - 1e-3f);
    CHECK(std::isfinite(b.min.x));
    CHECK(std::isfinite(b.max.x));
    CHECK(std::isfinite(b.min.y));
    CHECK(std::isfinite(b.max.y));
    CHECK(std::isfinite(b.min.z));
    CHECK(std::isfinite(b.max.z));
}

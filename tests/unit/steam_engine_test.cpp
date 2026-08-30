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

        // Body-local convention: cap_start = big end at the pin (the local
        // origin), cap_end = small end at the crosshead (the loop point).
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
        const float t = theta * kPi / 180.0f;
        const float px = d.crank_center_x + d.crank_radius * std::cos(t);
        const float py = d.crank_radius * std::sin(t);
        const float pz = d.pin_z_start + 0.5f * d.pin_length;
        CAPTURE(theta);
        // big end on the pin centerline (5 mm covers the ½·|d| − L residual
        // and the fan-centroid averaging)
        const float pinErr = std::sqrt((s.x - px) * (s.x - px) + (s.y - py) * (s.y - py) +
                                       (s.z - pz) * (s.z - pz));
        CHECK(pinErr < 0.005f);
        // small end within the crosshead block
        CHECK(e.x >= ch.min.x - 0.01f);
        CHECK(e.x <= ch.max.x + 0.01f);
        CHECK(e.z >= ch.min.z - 0.01f);
        CHECK(e.z <= ch.max.z + 0.01f);
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

// ── Mechanism contract (Phase D migration) ──

TEST_CASE("steam engine: mechanism validates and matches the joint table") {
    const SteamEngineResult r = generate_steam_engine(default_engine());
    std::vector<std::string> errors;
    REQUIRE(validate_mechanism(r.mechanism, errors));

    REQUIRE(r.mechanism.joints.size() == 7);
    CHECK(r.mechanism.joints[0].name == "shaft");
    CHECK(r.mechanism.joints[0].kind == JointKind::Continuous);
    CHECK(r.mechanism.joints[0].child == "crankshaft");
    CHECK(r.mechanism.driver_joint == "shaft");

    // the conrod carries both ends: two incoming joints → the loop edge
    uint32_t conrodIncoming = 0;
    for (const Joint& j : r.mechanism.joints)
        if (j.child == "conrod") conrodIncoming++;
    CHECK(conrodIncoming == 2);   // conrod_pin (tree) + conrod_cs (loop)
}

TEST_CASE("steam engine: FK rest pose equals the recipe's θ=0 pose") {
    SteamEngineDefinition d = default_engine();
    d.crank_angle_deg = 0.0f;
    const SteamEngineResult r = generate_steam_engine(d);
    const std::map<std::string, exd::math::Mat4> rest = evaluate_poses(r.mechanism, 0.0f);

    // recipe world pose of each non-static part (translation column)
    std::map<std::string, exd::math::Vec3f> recipeTrans;
    for (const Part& p : r.assembly.parts)
        recipeTrans[p.name] = {0.0f, 0.0f, 0.0f};   // placeholder: derive below

    // The recipe pose map is not stored, so compare against the EXECUTED
    // assembly: each part's bounds-centre must coincide with the FK pose's
    // translation of the part's local origin. The local origin is inside the
    // mesh; use the assembly part's bounds centre minus the mesh's local
    // bounds centre (both at the same world pose).
    for (const Part& placed : r.assembly.parts)
    {
        const auto it = rest.find(placed.name);
        if (it == rest.end()) continue;   // statics resolve to identity
        // local bounds centre
        const Part* local = nullptr;
        for (const Part& b : r.body)
            if (b.name == placed.name) { local = &b; break; }
        REQUIRE(local != nullptr);
        const Bounds lo = compute_bounds(local->mesh.vertices);
        const exd::math::Vec3f loC{(lo.min.x + lo.max.x) / 2.0f, (lo.min.y + lo.max.y) / 2.0f,
                              (lo.min.z + lo.max.z) / 2.0f};
        const Bounds wo = compute_bounds(placed.mesh.vertices);
        const exd::math::Vec3f woC{(wo.min.x + wo.max.x) / 2.0f, (wo.min.y + wo.max.y) / 2.0f,
                              (wo.min.z + wo.max.z) / 2.0f};
        // world centre ≈ restPose · local centre (translation part only when
        // the rest pose is pure translation — true at θ=0 for this machine?
        // crankshaft/flywheel/pin rotate about z: their centres are ON the
        // rotation axis (x_cc, 0, z_c)… except the pin (centre offset rc):
        // at θ=0 the rotation angle is 0, so restPose·loC == loC + t.
        const exd::math::Vec3f t{it->second.m[12], it->second.m[13], it->second.m[14]};
        CAPTURE(placed.name);
        CHECK(woC.x == doctest::Approx(loC.x + t.x).epsilon(2e-3f));
        CHECK(woC.y == doctest::Approx(loC.y + t.y).epsilon(2e-3f));
        CHECK(woC.z == doctest::Approx(loC.z + t.z).epsilon(2e-3f));
    }
}

TEST_CASE("steam engine: mjcf export carries joints, motor, and the loop") {
    const SteamEngineResult r = generate_steam_engine(default_engine());
    const ExportBundle b = to_mjcf(r.mechanism, r.body);
    const std::string& x = b.xml;

    // static base: cylinder/chest/ports under worldbody; kin parts as bodies
    CHECK(x.find("<body name=\"cylinder\" pos=\"0 0 0\">") != std::string::npos);
    // the driver: motor on the shaft
    CHECK(x.find("<motor joint=\"shaft\" ctrlrange=\"-50 50\"/>") != std::string::npos);
    // crank world-anchored continuous hinge
    CHECK(x.find("<joint name=\"shaft\" type=\"hinge\"") != std::string::npos);
    CHECK(x.find("<body name=\"crankshaft\" pos=\"0.45 0 0\">") != std::string::npos);
    // prismatic piston with limits, at the rest crosshead anchor
    CHECK(x.find("<joint name=\"piston_sl\" type=\"slide\"") != std::string::npos);
    CHECK(x.find("<body name=\"piston\" pos=\"1 0 0.105\">") != std::string::npos);
    // the slider-crank loop: conrod's second incoming joint → connect weld
    CHECK(x.find("<connect body1=\"crosshead\" body2=\"conrod\" anchor=\"1 0 0.105\"/>") != std::string::npos);
    // contact parts only: flywheel + crankshaft carry collision groups
    CHECK(x.find("mesh=\"flywheel\" group=\"1\" contype=\"1\" conaffinity=\"1\"") != std::string::npos);
    CHECK(x.find("mesh=\"piston\" group=\"1\" contype=") == std::string::npos);
    // inertials present, mesh bundle complete
    CHECK(x.find("<inertial pos=") != std::string::npos);
    REQUIRE(b.meshes.count("flywheel") == 1);
    CHECK(b.meshes.at("flywheel").find("v ") != std::string::npos);
}

TEST_CASE("steam engine: export deterministic") {
    const SteamEngineDefinition d = default_engine();
    const auto r1 = generate_steam_engine(d);
    const auto r2 = generate_steam_engine(d);
    CHECK(to_mjcf(r1.mechanism, r1.body).xml == to_mjcf(r2.mechanism, r2.body).xml);
    CHECK(to_urdf(r1.mechanism, r1.body).xml == to_urdf(r2.mechanism, r2.body).xml);
    CHECK(r1.body.size() == r2.body.size());
}

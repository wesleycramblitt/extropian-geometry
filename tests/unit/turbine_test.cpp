#include <doctest/doctest.h>

#include <exd/geometry/turbine.hpp>

#include <exd/geometry/mesh_ops.hpp>

#include <cmath>

using namespace exd::geometry;

namespace {

FlowPath annulus() {
    FlowPath f;
    // Simple cylindrical annulus: hub r=0.4, shroud r=1.0 over z in [0, 2].
    f.hub_points    = {{0.0f, 0.4f}, {1.0f, 0.4f}, {2.0f, 0.4f}};
    f.shroud_points = {{0.0f, 1.0f}, {1.0f, 1.0f}, {2.0f, 1.0f}};
    f.tip_clearance = {0.01f, 0.0f, 0.02f, "m", false};
    return f;
}

BladeRow rotor() {
    BladeRow r;
    r.type = BladeRowType::Rotor;
    r.blade_count = {24, 1, 200, "", false};
    r.leading_edge_hub    = {0.5f, 0.4f};
    r.leading_edge_shroud = {0.5f, 1.0f};
    r.trailing_edge_hub   = {0.9f, 0.4f};
    r.trailing_edge_shroud= {0.9f, 1.0f};
    // One section suffices to exercise the profile builder.
    r.sections = {BladeSection{0.0f}, BladeSection{1.0f}};
    r.sections[0].chord = {0.4f, 0.001f, 0.5f, "m", false};
    r.sections[0].max_thickness = {0.12f, 0.001f, 0.9f, "t/c", false};
    r.sections[0].stagger = {20.0f, -90.0f, 90.0f, "deg", false};
    r.sections[0].inlet_metal_angle  = {40.0f, -90.0f, 90.0f, "deg", false};
    r.sections[0].exit_metal_angle   = {-30.0f, -90.0f, 90.0f, "deg", false};
    r.sections[1] = r.sections[0];
    r.sections[1].span = 1.0f;
    r.tip_feature = TipFeature::Clearance;
    return r;
}

BladeRow single_straight_blade(float stagger_deg)
{
    BladeRow r;
    r.type = BladeRowType::Rotor;
    r.blade_count = {1, 1, 200, "", false};
    r.leading_edge_hub    = {0.5f, 0.4f};
    r.leading_edge_shroud = {0.5f, 1.0f};
    r.trailing_edge_hub   = {0.9f, 0.4f};
    r.trailing_edge_shroud= {0.9f, 1.0f};
    r.chordwise_points = 20;
    r.sections = {BladeSection{0.25f}, BladeSection{0.5f}};
    r.sections[0].stagger = {stagger_deg, -360.0f, 360.0f, "deg", false};
    r.sections[1].stagger = {stagger_deg, -360.0f, 360.0f, "deg", false};
    return r;
}

} // namespace

TEST_CASE("blade section profile is a closed, chord-length polygon") {
    BladeSection s;
    s.max_thickness = {0.12f, 0.001f, 0.9f, "t/c", false};
    s.inlet_metal_angle  = {40.0f, -90.0f, 90.0f, "deg", false};
    s.exit_metal_angle   = {-30.0f, -90.0f, 90.0f, "deg", false};
    s.stagger = {20.0f, -90.0f, 90.0f, "deg", false};

    const auto loop = generate_blade_section_profile(s, 0.4f, 48);
    REQUIRE(loop.size() >= 12);
    // x spans [0, chord].
    float max_x = 0.0f;
    for (const auto& p : loop) max_x = std::max(max_x, p.x);
    CHECK(max_x == doctest::Approx(0.4f));
    // The profile has positive thickness (upper and lower surfaces differ).
    float max_y = 0.0f, min_y = 0.0f;
    for (const auto& p : loop) { max_y = std::max(max_y, p.y); min_y = std::min(min_y, p.y); }
    CHECK(max_y > 0.0f);
    CHECK(min_y < 0.0f);
}

TEST_CASE("flow path mesh revolves hub and shroud into surfaces") {
    const MeshData mesh = generate_flow_path_mesh(annulus(), 64);
    REQUIRE_FALSE(mesh.vertices.empty());
    CHECK(mesh.indices.size() % 3 == 0);
    // An annulus surface has plenty of vertices.
    CHECK(mesh.vertices.size() > 64);
}

TEST_CASE("blade row mesh lofts sections and Z-folds") {
    const MeshData mesh = generate_blade_row_mesh(rotor(), annulus(), 64);
    REQUIRE_FALSE(mesh.vertices.empty());
    CHECK(mesh.indices.size() % 3 == 0);
    // 24 blades, each a closed loft of 2 sections x N points.
    CHECK(mesh.vertices.size() > 24 * 48);
}

TEST_CASE("turbine mesh assembles flow path and rows") {
    TurbineDefinition t;
    t.flow_path = annulus();
    t.blade_rows = {rotor()};
    const MeshData mesh = generate_turbine_mesh(t);
    REQUIRE_FALSE(mesh.vertices.empty());
    const MeshData path_only = generate_flow_path_mesh(annulus(), 64);
    CHECK(mesh.vertices.size() > path_only.vertices.size());
}

TEST_CASE("turbine is centered on origin and runs along -Z") {
    const MeshData mesh = generate_flow_path_mesh(annulus(), 64);
    REQUIRE_FALSE(mesh.vertices.empty());
    Bounds b = compute_bounds(mesh.vertices);
    // Centered: |min.z| ~= |max.z| (annulus spans z in [0,2] -> world z in [-1,1]).
    CHECK((b.min.z + b.max.z) == doctest::Approx(0.0f).epsilon(0.05f));
    // Axial extent along Z (r <= 1.0 in XY), not along Y.
    CHECK(std::fabs(b.max.z) > 0.9f);
    CHECK(std::fabs(b.max.y) <= 1.0f);
}

TEST_CASE("empty flow path produces empty mesh") {
    FlowPath f;   // no hub/shroud points
    CHECK(generate_flow_path_mesh(f).vertices.empty());
}

// ── Hub ────────────────────────────────────────────────────────────────────

TEST_CASE("hub: none produces empty mesh") {
    HubDefinition h;
    h.shape = HubShape::None;
    CHECK(generate_hub_mesh(h).vertices.empty());
}

TEST_CASE("hub: cylinder is a capped, radius-sized body") {
    HubDefinition h;
    h.shape = HubShape::Cylinder;
    h.root_radius  = 0.5f;
    h.front_length = 0.3f;
    h.aft_length   = 0.2f;

    const MeshData mesh = generate_hub_mesh(h, 48);
    REQUIRE_FALSE(mesh.vertices.empty());
    CHECK(mesh.indices.size() % 3 == 0);

    const Bounds b = compute_bounds(mesh.vertices);
    CHECK(b.min.z == doctest::Approx(-0.2f).epsilon(1e-3f));
    CHECK(b.max.z == doctest::Approx(0.3f).epsilon(1e-3f));
    CHECK(std::fabs(b.max.x) == doctest::Approx(0.5f).epsilon(1e-3f));
    CHECK(std::fabs(b.max.y) == doctest::Approx(0.5f).epsilon(1e-3f));

    // Both ends are capped: cap fan hubs on the axis with axial normals.
    int cap_hubs = 0;
    for (const auto& v : mesh.vertices)
        if (std::fabs(v.normal.z) > 0.99f && v.position.x == 0.0f && v.position.y == 0.0f)
            ++cap_hubs;
    CHECK(cap_hubs == 2);
}

TEST_CASE("hub: spinner has a pointed nose and a capped aft end") {
    HubDefinition h;
    h.shape = HubShape::Spinner;
    h.root_radius  = 0.35f;
    h.front_length = 0.6f;
    h.aft_length   = 0.4f;

    const MeshData mesh = generate_hub_mesh(h, 48);
    REQUIRE_FALSE(mesh.vertices.empty());

    const Bounds b = compute_bounds(mesh.vertices);
    CHECK(b.min.z == doctest::Approx(-0.4f).epsilon(1e-3f));  // flat aft cap
    CHECK(b.max.z == doctest::Approx(0.6f).epsilon(1e-3f));   // nose tip
    CHECK(b.max.x == doctest::Approx(0.35f).epsilon(1e-3f));

    // Only the aft end carries a cap (the nose closes to a point).
    int aft_caps = 0;
    for (const auto& v : mesh.vertices)
        if (v.normal.z < -0.99f && v.position.x == 0.0f && v.position.y == 0.0f)
            ++aft_caps;
    CHECK(aft_caps == 1);
}

TEST_CASE("hub: flat disk is thin and follows the root radius") {
    HubDefinition h;
    h.shape = HubShape::FlatDisk;
    h.root_radius = 0.5f;

    const MeshData mesh = generate_hub_mesh(h, 32);
    REQUIRE_FALSE(mesh.vertices.empty());

    const Bounds b = compute_bounds(mesh.vertices);
    CHECK((b.max.z - b.min.z) < 0.1f);                       // thin disc
    CHECK(b.max.x == doctest::Approx(0.5f).epsilon(1e-3f));
}

TEST_CASE("turbine mesh assembles with optional hub") {
    TurbineDefinition with_hub;
    with_hub.flow_path = annulus();
    with_hub.blade_rows = {rotor()};
    with_hub.hub.shape = HubShape::Cylinder;
    with_hub.hub.root_radius  = 0.4f;
    with_hub.hub.front_length = 0.3f;
    with_hub.hub.aft_length   = 0.2f;

    TurbineDefinition without_hub = with_hub;
    without_hub.hub.shape = HubShape::None;

    const MeshData a = generate_turbine_mesh(with_hub);
    const MeshData b = generate_turbine_mesh(without_hub);
    REQUIRE_FALSE(a.vertices.empty());
    CHECK(a.vertices.size() > b.vertices.size());
}

TEST_CASE("blade row: convex side sits at +theta for |stagger| < 90 deg") {
    // The convention test: at mid-chord the upper-surface vertex must sit at
    // a larger tangential angle than the lower-surface vertex while
    // |stagger| < 90 (camber suction side toward +theta), and flip beyond.
    // Section f=0.5 is the second loop: verts 20..39; upper@x=0.5 -> 25,
    // lower@x=0.5 -> 35 (profile: TE idx0, upper k=1..9, LE idx10, lower
    // k=11..19).
    auto tangents = [](float stagger) {
        const MeshData mesh = generate_blade_row_mesh(single_straight_blade(stagger), annulus(), 48);
        REQUIRE_FALSE(mesh.vertices.empty());
        const auto& v = mesh.vertices;
        const float th_up = std::atan2(v[25].position.y, v[25].position.x);
        const float th_lo = std::atan2(v[35].position.y, v[35].position.x);
        return std::make_pair(th_up, th_lo);
    };

    const auto [up_pos, lo_pos] = tangents(15.0f);
    CHECK(up_pos > lo_pos);                       // convex side at +theta
    const auto [up_neg, lo_neg] = tangents(-15.0f);
    // Stagger sign only rotates the chord: for |stagger| < 90 deg the
    // convex side still faces +theta (cos(stagger) > 0).
    CHECK(up_neg > lo_neg);
    const auto [up180, lo180] = tangents(180.0f); // flipped past 90 deg
    CHECK(up180 < lo180);
}

TEST_CASE("blade row: arbitrary stagger angles produce valid geometry") {
    for (const float s : {-360.0f, -270.0f, -180.0f, -90.0f, 45.0f, 90.0f, 180.0f, 270.0f, 360.0f}) {
        CAPTURE(s);
        const MeshData mesh = generate_blade_row_mesh(single_straight_blade(s), annulus(), 48);
        REQUIRE_FALSE(mesh.vertices.empty());
        CHECK(mesh.indices.size() % 3 == 0);
        // Two 20-point sections + two cap centroids, one blade.
        CHECK(mesh.vertices.size() == 2 * 20 + 2);
        const Bounds b = compute_bounds(mesh.vertices);
        // Blade radius must stay within the shroud radius + profile slack.
        CHECK(b.max.x < 1.3f);
        CHECK(b.max.y < 1.3f);
        CHECK(std::isfinite(b.min.z));
        CHECK(std::isfinite(b.max.z));
    }
}

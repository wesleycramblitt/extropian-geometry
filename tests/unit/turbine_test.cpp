#include <doctest/doctest.h>

#include <exd/geometry/turbine.hpp>

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

TEST_CASE("empty flow path produces empty mesh") {
    FlowPath f;   // no hub/shroud points
    CHECK(generate_flow_path_mesh(f).vertices.empty());
}

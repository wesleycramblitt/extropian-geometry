#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <cmath>
#include <limits>
#include <string>

using namespace exd::geometry;

namespace {

FlowPath compressor_flow()
{
    FlowPath f;
    // Simple cylindrical annulus: hub r=0.4, shroud r=1.0 over z in [0, 2].
    f.hub_points    = {{0.0f, 0.4f}, {1.0f, 0.4f}, {2.0f, 0.4f}};
    f.shroud_points = {{0.0f, 1.0f}, {1.0f, 1.0f}, {2.0f, 1.0f}};
    f.tip_clearance = {0.01f, 0.0f, 0.02f, "m", false};
    return f;
}

/// A 2-section blade row on the (z, r) LE/TE stations with the compressor fan
/// sense (rotor stagger negative so the suction side faces -theta).
BladeRow make_row(BladeRowType type, float le_z, float te_z, float stagger_deg)
{
    BladeRow r;
    r.type = type;
    r.blade_count = {12, 1, 200, "", false};
    r.leading_edge_hub    = {le_z, 0.45f};
    r.leading_edge_shroud = {le_z, 0.95f};
    r.trailing_edge_hub   = {te_z, 0.45f};
    r.trailing_edge_shroud= {te_z, 0.95f};
    r.sections = {BladeSection{0.0f}, BladeSection{1.0f}};
    r.sections[0].chord = {0.30f, 0.001f, 0.5f, "m", false};
    r.sections[0].max_thickness = {0.10f, 0.001f, 0.9f, "t/c", false};
    r.sections[0].stagger = {stagger_deg, -360.0f, 360.0f, "deg", false};
    r.sections[0].inlet_metal_angle  = {30.0f, -90.0f, 90.0f, "deg", false};
    r.sections[0].exit_metal_angle   = {-20.0f, -90.0f, 90.0f, "deg", false};
    r.sections[1] = r.sections[0];
    r.sections[1].span = 1.0f;
    r.tip_feature = TipFeature::Clearance;
    return r;
}

CompressorDefinition default_machine()
{
    CompressorDefinition def;
    def.flow_path = compressor_flow();

    CompressorStage stage;
    stage.rotor  = make_row(BladeRowType::Rotor,  0.5f, 0.9f, -20.0f);
    stage.stator = make_row(BladeRowType::Stator, 1.1f, 1.5f,  10.0f);
    def.stages = {stage};

    def.spinner.shape        = HubShape::Spinner;
    def.spinner.root_radius  = 0.4f;
    def.spinner.front_length = 0.25f;
    def.spinner.aft_length   = 0.15f;
    return def;
}

bool has_patch(const Part& part, const std::string& name)
{
    for (const Patch& patch : part.patches)
        if (patch.name == name) return true;
    return false;
}

uint32_t patch_faces(const Part& part, const std::string& name)
{
    for (const Patch& patch : part.patches)
        if (patch.name == name) return static_cast<uint32_t>(patch.faces.size());
    return 0;
}

} // namespace

TEST_CASE("compressor: default machine parts") {
    const CompressorDefinition def = default_machine();
    const Assembly a = generate_compressor_assembly(def);
    REQUIRE(a.parts.size() == 4);
    CHECK(a.parts[0].name == "spinner");
    CHECK(a.parts[1].name == "casing");
    CHECK(a.parts[2].name == "rotor_0");
    CHECK(a.parts[3].name == "stator_0");

    for (const Part& part : a.parts)
        CHECK_FALSE(part.mesh.vertices.empty());

    // spinner: single "surface" patch covering every face.
    const Part& spinner = a.parts[0];
    REQUIRE(has_patch(spinner, "surface"));
    CHECK(patch_faces(spinner, "surface") == spinner.mesh.indices.size() / 3);

    // casing: "surface" plus (off-axis ends) "cap_start"/"cap_end".
    const Part& casing = a.parts[1];
    REQUIRE(has_patch(casing, "surface"));
    REQUIRE(has_patch(casing, "cap_start"));
    REQUIRE(has_patch(casing, "cap_end"));
    CHECK(patch_faces(casing, "surface") + patch_faces(casing, "cap_start") +
              patch_faces(casing, "cap_end") == casing.mesh.indices.size() / 3);

    // rotor_0 / stator_0: blade rows carry the three blade patches.
    const Part& rotor = a.parts[2];
    REQUIRE(has_patch(rotor, "blade_surface"));
    REQUIRE(has_patch(rotor, "hub_cap"));
    REQUIRE(has_patch(rotor, "shroud_cap"));
    CHECK(patch_faces(rotor, "blade_surface") + patch_faces(rotor, "hub_cap") +
              patch_faces(rotor, "shroud_cap") == rotor.mesh.indices.size() / 3);

    const Part& stator = a.parts[3];
    REQUIRE(has_patch(stator, "blade_surface"));
    REQUIRE(has_patch(stator, "hub_cap"));
    REQUIRE(has_patch(stator, "shroud_cap"));
    CHECK(patch_faces(stator, "blade_surface") + patch_faces(stator, "hub_cap") +
              patch_faces(stator, "shroud_cap") == stator.mesh.indices.size() / 3);
}

TEST_CASE("compressor: igv and stages order") {
    CompressorDefinition def = default_machine();
    def.has_igv = true;
    def.igv = make_row(BladeRowType::Stator, 0.15f, 0.35f, 0.0f);

    CompressorStage second;
    second.rotor  = make_row(BladeRowType::Rotor,  1.6f, 1.95f, -25.0f);
    second.stator = make_row(BladeRowType::Stator, 1.6f, 1.95f,   5.0f);
    def.stages.push_back(second);

    const Assembly a = generate_compressor_assembly(def);
    REQUIRE(a.parts.size() == 7);
    CHECK(a.parts[0].name == "spinner");
    CHECK(a.parts[1].name == "casing");
    CHECK(a.parts[2].name == "igv");
    CHECK(a.parts[3].name == "rotor_0");
    CHECK(a.parts[4].name == "stator_0");
    CHECK(a.parts[5].name == "rotor_1");
    CHECK(a.parts[6].name == "stator_1");

    const Part& igv = a.parts[2];
    REQUIRE(has_patch(igv, "blade_surface"));
    REQUIRE(has_patch(igv, "hub_cap"));
    REQUIRE(has_patch(igv, "shroud_cap"));
    CHECK(patch_faces(igv, "blade_surface") + patch_faces(igv, "hub_cap") +
              patch_faces(igv, "shroud_cap") == igv.mesh.indices.size() / 3);

    for (std::size_t i = 3; i < a.parts.size(); ++i) {
        const Part& p = a.parts[i];
        REQUIRE(has_patch(p, "blade_surface"));
        REQUIRE(has_patch(p, "hub_cap"));
        REQUIRE(has_patch(p, "shroud_cap"));
    }
}

TEST_CASE("compressor: casing matches shroud flow path") {
    CompressorDefinition def;
    // 3-point symmetric shroud profile only (no hub, no spinner, no rows).
    def.flow_path.shroud_points = {{-0.5f, 0.8f}, {0.0f, 1.2f}, {0.5f, 0.8f}};
    def.spinner.shape = HubShape::None;

    const Assembly a = generate_compressor_assembly(def);
    REQUIRE(a.parts.size() == 1);
    REQUIRE(a.parts[0].name == "casing");

    const Part& casing = a.parts[0];
    REQUIRE_FALSE(casing.mesh.vertices.empty());
    REQUIRE(has_patch(casing, "surface"));

    float max_r = 0.0f;
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    for (const Vertex& v : casing.mesh.vertices) {
        const float r = std::sqrt(v.position.x * v.position.x + v.position.y * v.position.y);
        max_r = std::max(max_r, r);
        min_z = std::min(min_z, v.position.z);
        max_z = std::max(max_z, v.position.z);
    }

    // The casing is the shroud surface revolved: max radius == max shroud r.
    const float max_shroud_r = 1.2f;
    CHECK(max_r == doctest::Approx(max_shroud_r).epsilon(0.02f));

    // Axial extent matches the shroud control-point span (casing centered on
    // the origin, following the machine orientation convention).
    const float first_z = def.flow_path.shroud_points.front().x;   // -0.5
    const float last_z  = def.flow_path.shroud_points.back().x;    // +0.5
    const float span    = last_z - first_z;
    CHECK(std::abs(min_z - first_z) <= 0.05f * span);
    CHECK(std::abs(max_z - last_z)  <= 0.05f * span);
}

TEST_CASE("compressor: no stages still gives flow machine") {
    // (a) shroud-only flow path → a single casing part with surface patch.
    CompressorDefinition def;
    def.flow_path.shroud_points = {{-0.5f, 0.9f}, {0.0f, 1.0f}, {0.5f, 0.9f}};
    def.spinner.shape = HubShape::None;
    const Assembly a = generate_compressor_assembly(def);
    REQUIRE(a.parts.size() == 1);
    CHECK(a.parts[0].name == "casing");
    REQUIRE(has_patch(a.parts[0], "surface"));

    // (b) fully empty machine → empty assembly.
    CompressorDefinition empty;
    empty.spinner.shape = HubShape::None;
    const Assembly e = generate_compressor_assembly(empty);
    CHECK(e.parts.empty());
    CHECK(generate_compressor_mesh(empty).vertices.empty());
}

TEST_CASE("compressor: mesh convenience equals flattened assembly") {
    const CompressorDefinition def = default_machine();
    const Assembly a = generate_compressor_assembly(def);
    const MeshData mesh = generate_compressor_mesh(def);
    REQUIRE_FALSE(mesh.vertices.empty());
    const Part flat = flatten(a);
    CHECK(mesh.vertices.size() == flat.mesh.vertices.size());
    CHECK(mesh.indices.size()  == flat.mesh.indices.size());
}

TEST_CASE("compressor: bounds sane") {
    const CompressorDefinition def = default_machine();
    const Assembly a = generate_compressor_assembly(def);
    const Bounds& b = a.bounds;
    CHECK(b.min.x <= b.max.x);
    CHECK(b.min.y <= b.max.y);
    CHECK(b.min.z <= b.max.z);
    // Bounds cover the casing radius (shroud r = 1.0).
    CHECK(b.max.x >= 0.9f);
    CHECK(b.max.y >= 0.9f);
    CHECK(std::isfinite(b.min.z));
    CHECK(std::isfinite(b.max.z));
    CHECK(b.min.z < b.max.z);
}
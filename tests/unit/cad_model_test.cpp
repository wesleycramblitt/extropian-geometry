#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <string>
#include <vector>

using namespace exd::geometry;

namespace
{

/// A small deterministic machine: a toothed-belt pump casing + rotor disc,
/// with multiphysics topology (solid + fluid regions, MRF interface),
/// physics-tagged BCs, body loads, cyclic sector, and a rotation frame.
Part casing_box()
{
    Part p = generate_box_part(BoxGeometry{{0.3f, 0.2f, 0.2f}}); // name comes out as "box"
    p.name = "casing";
    p.meta.material = "aluminum-6061";
    p.meta.contact = true;
    return p;
}

Part rotor_cyl()
{
    Part p = generate_cylinder_part(CylinderGeometry{0.06f, 0.12f}); // name comes out as "cylinder"
    p.name = "rotor";
    p.meta.material = "steel-1045";
    p.meta.motion = PartMotion::Rotating;
    // periodic sector boundary faces: pin the wall patch as the sector gate
    p.patches.push_back(make_patch_range("sector", 0, 64));
    return p;
}

Mechanism simple_mech()
{
    Mechanism m;
    m.joints.push_back({"shaft", JointKind::Continuous, "", "rotor",
                        {0, 0, 0}, {0, 0, 1}, -1e30f, 1e30f, 10.0f, 20.0f});
    m.driver_joint = "shaft";
    return m;
}

CADModel pump_model()
{
    std::vector<Part> parts{casing_box(), rotor_cyl()};
    CADModel m = make_cad_model("belt_pump", parts, simple_mech());
    m.materials = MaterialDB::defaults();

    // regions: solid casing + fluid rotor-gap, coupled at the rotor wall
    Region solid;
    solid.name = "casing_solid";
    solid.boundary = {{"casing", "+x"}, {"casing", "-x"}, {"casing", "+y"},
                      {"casing", "-y"}, {"casing", "+z"}, {"casing", "-z"}};
    solid.active = {Physics::Structural, Physics::Thermal};
    solid.material[Physics::Structural] = "aluminum-6061";
    solid.material[Physics::Thermal]    = "aluminum-6061";
    m.regions.push_back(solid);

    Region fluid;
    fluid.name = "rotor_gap_fluid";
    fluid.boundary = {{"rotor", "wall"}};
    fluid.active = {Physics::Fluid, Physics::Thermal};
    fluid.material[Physics::Fluid]  = "water";
    fluid.material[Physics::Thermal] = "water";
    m.regions.push_back(fluid);

    m.domains = {{"structure", {"casing_solid"}}, {"flow", {"rotor_gap_fluid"}}};

    m.interfaces.push_back({"rotor_mrf", InterfaceKind::MRFStage,
                            "flow", "structure", {"rotor", "wall"}, {"rotor", "wall"}});

    // two BCs on the same patch across different physics — allowed by design
    BoundaryCondition fixed;
    fixed.name = "casing_fix";
    fixed.physics = Physics::Structural;
    fixed.kind = BCKind::Fixed;
    fixed.part = "casing";
    fixed.patch = "-y";
    m.bcs.push_back(fixed);

    BoundaryCondition htc;
    htc.name = "casing_conv";
    htc.physics = Physics::Thermal;
    htc.kind = BCKind::Convection;
    htc.part = "casing";
    htc.patch = "+x";
    htc.magnitude = 10.0f;   // W/(m²·K) with ambient encoded in value.x
    htc.value = {293.0f, 0.0f, 0.0f};
    m.bcs.push_back(htc);

    BodyLoad g;
    g.name = "gravity";
    g.region = "casing_solid";
    g.kind = BodyLoadKind::Gravity;
    g.vector = {0.0f, -9.81f, 0.0f};
    m.body_loads.push_back(g);

    m.cyclic_sectors.push_back({"rotor_sector", "rotor", "sector", 9, 40.0f, {0, 0, 1}});
    m.frames.push_back({"rotor_frame", "rotor", 3000.0f, {0, 0, 1}});
    return m;
}

} // namespace

TEST_CASE("cad_model: patch semantics are additive; legacy defaults preserved") {
    const Part p = generate_box_part(BoxGeometry{{1, 1, 1}});
    REQUIRE(p.patches.size() == 6);
    const Patch& first = p.patches.front();
    CHECK(first.semantic == PatchSemantic::Unspecified);
    CHECK(first.mesh_size == doctest::Approx(0.0f));
    CHECK(first.material.empty());
    // face ordinals still resolve to triangles of the part mesh
    CHECK((first.faces.empty() || first.faces.back() < p.mesh.indices.size() / 3));
}

TEST_CASE("cad_model: datatable interpolation (clamped ends)") {
    DataTable t;
    t.x = {0.0f, 10.0f, 20.0f};
    t.y = {0.0f, 100.0f, 200.0f};
    CHECK(t.empty() == false);
    CHECK(t.sample(5.0f) == doctest::Approx(50.0f));
    CHECK(t.sample(-5.0f) == doctest::Approx(0.0f));    // clamp low
    CHECK(t.sample(25.0f) == doctest::Approx(200.0f));  // clamp high
    CHECK(DataTable{}.empty());
}

TEST_CASE("cad_model: material db defaults and registration") {
    const MaterialDB db = MaterialDB::defaults();
    CHECK(db.find("steel-1045") != nullptr);
    CHECK(db.find("aluminum-6061") != nullptr);
    CHECK(db.find("water") != nullptr);
    CHECK(db.find("no-such-material") == nullptr);
    CHECK(db.names().size() >= 7);

    const Material* steel = db.find("steel-1045");
    CHECK(steel->density == doctest::Approx(7850.0f));
    CHECK(steel->youngs_modulus == doctest::Approx(205.0e9f));
    CHECK(steel->poisson == doctest::Approx(0.29f));

    MaterialDB editable;
    REQUIRE(editable.register_(*steel));
    CHECK(editable.size() == 1);
    CHECK(editable.register_(*steel) == false);       // duplicate rejected
    Material unnamed;                                  // empty name rejected
    CHECK(editable.register_(unnamed) == false);
}

TEST_CASE("cad_model: make_cad_model computes bounds and resolves materials") {
    std::vector<Part> parts{casing_box(), rotor_cyl()};
    CADModel m = make_cad_model("pump", parts, simple_mech());
    m.materials = MaterialDB::defaults();

    CHECK(m.name == "pump");
    CHECK(m.units == UnitSystem::SI);
    CHECK(m.parts.size() == 2);
    // bounds cover the union of the two parts
    CHECK(m.bounds.min.x == doctest::Approx(-0.15f));
    CHECK(m.bounds.max.x == doctest::Approx(0.15f));
    CHECK(m.bounds.max.z == doctest::Approx(0.1f));   // box (±0.1) dominates rotor (±0.06)

    const Part& casing = m.parts[0];
    CHECK(m.material_for(casing) == m.materials.find("aluminum-6061"));
    CHECK(m.material_for({"casing", "-y"}) == m.materials.find("aluminum-6061"));
    // unknown material does not resolve
    Part dangling = rotor_cyl();
    dangling.meta.material = "unobtanium";
    CHECK(m.material_for(dangling) == nullptr);
}

TEST_CASE("cad_model: multiphysics model validates") {
    CADModel m = pump_model();
    std::vector<std::string> errors;
    REQUIRE(m.validate(errors));
    CHECK(errors.empty());
}

TEST_CASE("cad_model: validation reports reference and material failures deterministically") {
    CADModel m = pump_model();
    m.materials = MaterialDB::defaults();   // keep table: pump_model set it

    std::vector<std::string> errors;

    // BC referencing a missing part
    m.bcs.push_back({"bad_bc", Physics::Thermal, BCKind::SpecifiedTemperature,
                     "no_such_part", "wall", {0, 0, 0}, 300.0f, {}, ""});
    CHECK(m.validate(errors) == false);
    const std::string joined = errors[0] + "\n";
    CHECK(joined.find("bad_bc") != std::string::npos);

    // unresolved part material
    errors.clear();
    m.bcs.pop_back();
    m.parts[0].meta.material = "unobtanium";
    CHECK(m.validate(errors) == false);
    CHECK(errors[0].find("unobtanium") != std::string::npos);

    // region bound to a missing face
    errors.clear();
    m.parts[0].meta.material = "aluminum-6061";
    Region ghost;
    ghost.name = "ghost";
    ghost.boundary = {{"rotor", "side_panel"}};   // patch does not exist
    ghost.active = {Physics::Structural};
    ghost.material[Physics::Structural] = "steel-1045";
    m.regions.push_back(ghost);
    CHECK(m.validate(errors) == false);
    CHECK(errors[0].find("side_panel") != std::string::npos);
}

TEST_CASE("cad_model: valid model stays valid after corrections") {
    CADModel m = pump_model();
    Part dangling = rotor_cyl();
    dangling.name = "rotor";                     // restore name (copy kept patches)
    dangling.meta.material = "steel-1045";
    m.parts[1] = dangling;
    std::vector<std::string> errors;
    REQUIRE(m.validate(errors));
    CHECK(errors.empty());
}

TEST_CASE("cad_model: mechanisms travel with the model and validate") {
    CADModel m = pump_model();
    std::vector<std::string> errors;
    REQUIRE(m.validate(errors));
    CHECK(m.mechanism.joints.size() == 1);
    CHECK(m.mechanism.couplings.empty());
    // FK still works from the carried mechanism
    const auto poses = evaluate_poses(m.mechanism, 0.0f);
    CHECK(poses.count("rotor") == 1);
}

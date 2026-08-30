#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <cmath>
#include <string>
#include <vector>

using namespace exd::geometry;

namespace
{

/// Two disc parts: a hub axle (thin cylinder along Z at origin) and a wheel
/// (torus/cylinder) built in its own local frame, ready for joints.
Part make_shaft(const std::string& name, float radius = 0.04f, float halfLen = 0.1f)
{
    CylinderGeometry cyl;
    cyl.radius = radius;
    cyl.height = 2.0f * halfLen;
    Part p = generate_cylinder_part(cyl);
    p.name = name;
    return p;
}

float pose_angle_z(const exd::math::Mat4& m)
{
    // rotation angle about Z from the matrix (column vectors: m*e_x)
    const float cx = m.m[0], sx = m.m[1];
    return std::atan2(sx, cx);
}

} // namespace

TEST_CASE("mechanism: wheel on axle (continuous) rotates with the driver") {
    Part shaft = make_shaft("shaft");
    CylinderGeometry wheelCyl;
    wheelCyl.radius = 0.2f;
    wheelCyl.height = 0.02f;
    Part wheel = as_part("wheel", generate_cylinder_mesh(wheelCyl));

    Mechanism m;
    m.joints.push_back({"axle", JointKind::Continuous, "", "wheel", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 1e30f, 1e30f});
    m.driver_joint = "axle";

    const auto p0 = evaluate_poses(m, 0.5f);
    REQUIRE(p0.count("wheel") == 1);
    CHECK(pose_angle_z(p0.at("wheel")) == doctest::Approx(0.5f).epsilon(1e-5f));
    const auto p1 = evaluate_poses(m, -0.25f);
    CHECK(pose_angle_z(p1.at("wheel")) == doctest::Approx(-0.25f).epsilon(1e-5f));
    // shaft is static: no entry (identity when applied)
    CHECK(p0.count("shaft") == 0);

    // apply_poses: patch ordinals survive (cylinder part patches)
    std::vector<Part> parts{shaft, wheel};
    const Assembly a = apply_poses(m, parts, p0);
    REQUIRE(a.parts.size() == 2);
    CHECK(a.parts[1].mesh.vertices.size() == wheel.mesh.vertices.size());
    for (const Patch& patch : a.parts[1].patches)
        for (const uint32_t f : patch.faces)
            CHECK(f < a.parts[1].mesh.indices.size() / 3);
}

TEST_CASE("mechanism: gear pair couples coordinates at the ratio") {
    Part a = make_shaft("gear_a");
    Part b = make_shaft("gear_b");

    Mechanism m;
    m.joints.push_back({"j_a", JointKind::Continuous, "", "gear_a", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 1e30f, 1e30f});
    m.joints.push_back({"j_b", JointKind::Continuous, "", "gear_b", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 1e30f, 1e30f});
    m.couplings.push_back({"gear_ab", CouplingKind::Gear, "j_a", "j_b", -2.0f});  // counter-rotating 1:2
    m.driver_joint = "j_a";

    const auto poses = evaluate_poses(m, 3.0f);
    REQUIRE(poses.count("gear_a") == 1);
    REQUIRE(poses.count("gear_b") == 1);
    CHECK(pose_angle_z(poses.at("gear_a")) == doctest::Approx(3.0f).epsilon(1e-5f));
    // q_b = ratio·q_a = −6 → angle of gear_b is −6 (full turns fold)
    CHECK(pose_angle_z(poses.at("gear_b")) == doctest::Approx(-6.0f + 2.0f * 3.14159265f).epsilon(1e-4f));

    // reverse drive: q_a = q_b / ratio
    m.driver_joint = "j_b";
    const auto rev = evaluate_poses(m, 1.0f);
    CHECK(pose_angle_z(rev.at("gear_a")) == doctest::Approx(-0.5f).epsilon(1e-5f));
    CHECK(pose_angle_z(rev.at("gear_b")) == doctest::Approx(1.0f).epsilon(1e-5f));
}

TEST_CASE("mechanism: prismatic slider moves along the axis with limits") {
    Part rail = make_shaft("rail");
    Part slider = as_part("slider", generate_box_mesh(BoxGeometry{{0.05f, 0.05f, 0.05f}}));

    Mechanism m;
    m.joints.push_back({"slide", JointKind::Prismatic, "", "slider", {0,0,0}, {1,0,0}, 0.0f, 2.0f, 100.0f, 1.0f});
    m.driver_joint = "slide";

    const auto p = evaluate_poses(m, 1.5f);
    REQUIRE(p.count("slider") == 1);
    // 3D: translation along +X: element m[3] = translation.x (column-major-ish check via TRS)
    CHECK(p.at("slider").m[12] == doctest::Approx(1.5f).epsilon(1e-5f));

    // limits clamp the state
    const auto clamped = evaluate_poses(m, 5.0f);
    CHECK(clamped.at("slider").m[12] == doctest::Approx(2.0f).epsilon(1e-5f));
}

TEST_CASE("mechanism: revolute about an anchor rotates in place") {
    Part base = make_shaft("base");
    Part arm = as_part("arm", generate_box_mesh(BoxGeometry{{0.5f, 0.05f, 0.05f}}));
    Mechanism m;
    m.joints.push_back({"hinge", JointKind::Revolute, "", "arm", {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, -1.5708f, 1.5708f, 10.0f, 1.0f});
    m.driver_joint = "hinge";
    const auto p = evaluate_poses(m, 1.5708f);
    REQUIRE(p.count("arm") == 1);
    CHECK(pose_angle_z(p.at("arm")) == doctest::Approx(1.5708f).epsilon(1e-4f));
    // anchor stays fixed: world position of the anchor point = anchor
    // (rotation about anchor): position of local (1,0,0) ≈ (1,0,0)
    // via m*(1,0,0): x = m.m[0]*1 + m.m[12]
    const float x = p.at("arm").m[0] * 1.0f + p.at("arm").m[12];
    const float y = p.at("arm").m[1] * 1.0f + p.at("arm").m[13];
    CHECK(std::sqrt((x - 1.0f) * (x - 1.0f) + y * y) == doctest::Approx(0.0f).epsilon(1e-4f));
}

TEST_CASE("mechanism: validation catches graph errors") {
    Mechanism m;
    std::vector<std::string> junk;
    REQUIRE_FALSE(validate_mechanism(m, junk));   // no driver
    m.driver_joint = "axle";
    m.joints.push_back({"axle", JointKind::Continuous, "", "wheel", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 1e30f, 1e30f});
    std::vector<std::string> errors;
    CHECK(validate_mechanism(m, errors));
    // duplicate name
    m.joints.push_back({"axle", JointKind::Fixed, "", "other", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 1e30f, 1e30f});
    errors.clear();
    CHECK_FALSE(validate_mechanism(m, errors));
    CHECK(errors.size() >= 1);
    // degenerate axis
    m.joints.pop_back();
    m.joints.push_back({"bad", JointKind::Revolute, "", "other", {0,0,0}, {0,0,0}, -1, 1, 1, 1});
    errors.clear();
    CHECK_FALSE(validate_mechanism(m, errors));
    // unknown coupling target
    m.joints.pop_back();
    m.couplings.push_back({"c", CouplingKind::Gear, "axle", "ghost", 2.0f});
    errors.clear();
    CHECK_FALSE(validate_mechanism(m, errors));
    // zero ratio
    m.couplings.clear();
    m.couplings.push_back({"c", CouplingKind::Gear, "axle", "axle2", 0.0f});
    m.joints.push_back({"axle2", JointKind::Continuous, "", "w2", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 1e30f, 1e30f});
    errors.clear();
    CHECK_FALSE(validate_mechanism(m, errors));
}

TEST_CASE("mechanism: deterministic across repeated evaluation") {
    Part a = make_shaft("a");
    Part b = make_shaft("b");
    Mechanism m;
    m.joints.push_back({"j1", JointKind::Continuous, "", "a", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 1e30f, 1e30f});
    m.joints.push_back({"j2", JointKind::Continuous, "", "b", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 1e30f, 1e30f});
    m.couplings.push_back({"g", CouplingKind::Gear, "j1", "j2", -1.5f});
    m.driver_joint = "j1";
    const auto p1 = evaluate_poses(m, 0.7f);
    const auto p2 = evaluate_poses(m, 0.7f);
    REQUIRE(p1.size() == p2.size());
    for (const auto& [name, mat] : p1)
    {
        REQUIRE(p2.count(name) == 1);
        for (int k = 0; k < 16; ++k)
            CHECK(mat.m[k] == p2.at(name).m[k]);
    }
}

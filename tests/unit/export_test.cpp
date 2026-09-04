#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <cmath>
#include <string>

using namespace exd::geometry;

namespace
{

/// Two disc parts in body-local frames: gear_a (driver) and gear_b coupled
/// at ratio −2, plus a static housing.
void gearbox(Part& housing, Part& gearA, Part& gearB, Mechanism& mech,
             std::vector<Part>& parts)
{
    housing = as_part("housing", generate_box_mesh(BoxGeometry{{0.4f, 0.4f, 0.4f}}));
    housing.meta.contact = true;
    gearA = as_part("gear_a", generate_cylinder_mesh(CylinderGeometry{0.05f, 0.1f}));
    gearB = as_part("gear_b", generate_cylinder_mesh(CylinderGeometry{0.04f, 0.1f}));
    mech.joints.push_back({"j_a", JointKind::Continuous, "", "gear_a",
                           {0, 0, 0}, {0, 0, 1}, -1e30f, 1e30f, 10.0f, 20.0f});
    mech.joints.push_back({"j_b", JointKind::Continuous, "", "gear_b",
                           {0.2f, 0.0f, 0.0f}, {0, 0, 1}, -1e30f, 1e30f, 10.0f, 20.0f});
    mech.couplings.push_back({"g", CouplingKind::Gear, "j_a", "j_b", -2.0f});
    mech.driver_joint = "j_a";
    parts = {housing, gearA, gearB};
}

} // namespace

TEST_CASE("export: mjcf emits bodies, joints, equality, actuator") {
    Part housing, gearA, gearB;
    Mechanism m;
    std::vector<Part> parts;
    gearbox(housing, gearA, gearB, m, parts);

    const ExportBundle b = to_mjcf(m, parts);
    const std::string& x = b.xml;

    CHECK(x.find("<mujoco model=\"model\">") != std::string::npos);
    // static housing under worldbody with a mesh geom + inertial
    CHECK(x.find("<body name=\"housing\" pos=\"0 0 0\">") != std::string::npos);
    CHECK(x.find("<mesh name=\"housing\" file=\"housing.obj\"/>") != std::string::npos);
    // gear_b body at its joint anchor, continuous hinge on +Z
    CHECK(x.find("<body name=\"gear_b\" pos=\"0.2 0 0\">") != std::string::npos);
    CHECK(x.find("<joint name=\"j_b\" type=\"hinge\" pos=\"0 0 0\" axis=\"0 0 1\"") != std::string::npos);
    // the gear coupling as a constraint: q_b − (−2)·q_a = 0 → polycoef [0, 2, 1]
    CHECK(x.find("<joint joint1=\"j_a\" joint2=\"j_b\" polycoef=\"0 2 1\"/>") != std::string::npos);
    // driver actuator with the effort limit
    CHECK(x.find("<motor joint=\"j_a\" ctrlrange=\"-10 10\"/>") != std::string::npos);
    // inertials (default water density): gear_a cyl r=0.05 h=0.1 → V=πr²h≈7.85e-4
    CHECK(x.find("<inertial pos=") != std::string::npos);
    CHECK(x.find("mass=\"0.78") != std::string::npos);   // faceted cylinder ≈ analytic
    // contact part carries collision group; non-contact does not
    CHECK(x.find("<geom type=\"mesh\" mesh=\"housing\" group=\"1\" contype=\"1\" conaffinity=\"1\"") != std::string::npos);
    CHECK(x.find("mesh=\"gear_a\" group=\"1\" density=") != std::string::npos);
    CHECK(x.find("mesh=\"gear_a\" group=\"1\" contype=") == std::string::npos);
    // OBJ bundle: one mesh per part with matching vertex counts
    REQUIRE(b.meshes.count("gear_a") == 1);
    const std::string& obj = b.meshes.at("gear_a");
    CHECK(obj.find("v ") != std::string::npos);
    CHECK(obj.find("f ") != std::string::npos);
}

TEST_CASE("export: urdf links, joints, mimic, inertials") {
    Part housing, gearA, gearB;
    Mechanism m;
    std::vector<Part> parts;
    gearbox(housing, gearA, gearB, m, parts);

    const ExportBundle b = to_urdf(m, parts);
    const std::string& x = b.xml;

    CHECK(x.find("<robot name=\"model\">") != std::string::npos);
    CHECK(x.find("<link name=\"gear_a\">") != std::string::npos);
    CHECK(x.find("<joint name=\"j_b\" type=\"continuous\">") != std::string::npos);
    CHECK(x.find("<parent link=\"world\"/>") != std::string::npos);
    CHECK(x.find("<origin xyz=\"0.2 0 0\" rpy=\"0 0 0\"/>") != std::string::npos);
    CHECK(x.find("<limit lower=\"-1e+30\" upper=\"1e+30\" effort=\"10\" velocity=\"20\"/>") != std::string::npos);
    // gear → mimic on the driven joint
    CHECK(x.find("<mimic joint=\"j_a\" multiplier=\"-2\" offset=\"0\"/>") != std::string::npos);
    // inertia tensor row (six values, ours from Phase A)
    CHECK(x.find("<inertia ixx=") != std::string::npos);
    // contact part also emits a <collision> block; non-contact does not
    CHECK(x.find("<collision>") != std::string::npos);
    const std::vector<Part> noHousing{gearA, gearB};
    const std::string xNoContact = to_urdf(m, noHousing).xml;
    CHECK(xNoContact.find("<collision>") == std::string::npos);
}

TEST_CASE("export: mjcf emits loop welds for loop-carrier joints") {
    Part a = as_part("a", generate_box_mesh(BoxGeometry{{0.1f, 0.1f, 0.1f}}));
    Part b = as_part("b", generate_box_mesh(BoxGeometry{{0.1f, 0.1f, 0.1f}}));
    Part c = as_part("c", generate_box_mesh(BoxGeometry{{0.1f, 0.1f, 0.1f}}));
    Mechanism m;   // a→b tree edge; c→b loop edge (second incoming on b)
    m.joints.push_back({"ab", JointKind::Fixed, "", "b", {0, 0, 0}, {0, 0, 1}, 0, 0, 0, 0});
    m.joints.push_back({"cb", JointKind::Revolute, "c", "b", {1, 0, 0}, {0, 0, 1}, -3.14f, 3.14f, 5, 5});
    m.driver_joint = "ab";
    std::vector<Part> parts{a, b, c};
    const std::string x = to_mjcf(m, parts).xml;
    CHECK(x.find("<connect body1=\"c\" body2=\"b\" anchor=\"1 0 0\"/>") != std::string::npos);
}

TEST_CASE("export: compliance fields carry to both formats") {
    Part a = as_part("a", generate_cylinder_mesh(CylinderGeometry{0.02f, 0.05f}));
    Part b = as_part("b", generate_cylinder_mesh(CylinderGeometry{0.02f, 0.05f}));
    Mechanism m;
    Joint spring{"spring_j", JointKind::Prismatic, "", "b", {0, 0, 0}, {1, 0, 0},
                 0.0f, 0.1f, 20.0f, 1.0f};
    spring.stiffness = 500.0f;
    spring.damping = 4.0f;
    spring.armature = 0.02f;
    spring.frictionloss = 1.0f;
    m.joints.push_back(spring);
    m.driver_joint = "spring_j";
    std::vector<Part> parts{a, b};

    const std::string mj = to_mjcf(m, parts).xml;
    CHECK(mj.find("<joint name=\"spring_j\" type=\"slide\"") != std::string::npos);
    CHECK(mj.find("stiffness=\"500\"") != std::string::npos);
    CHECK(mj.find("damping=\"4\"") != std::string::npos);
    CHECK(mj.find("armature=\"0.02\"") != std::string::npos);
    CHECK(mj.find("frictionloss=\"1\"") != std::string::npos);

    const std::string ur = to_urdf(m, parts).xml;
    CHECK(ur.find("<dynamics damping=\"4\" friction=\"1\" stiffness=\"500\"/>") != std::string::npos);
}

TEST_CASE("export: deterministic and stable") {
    Part housing, gearA, gearB;
    Mechanism m;
    std::vector<Part> parts;
    gearbox(housing, gearA, gearB, m, parts);

    const ExportBundle b1 = to_mjcf(m, parts);
    const ExportBundle b2 = to_mjcf(m, parts);
    CHECK(b1.xml == b2.xml);
    CHECK(b1.meshes.at("gear_a") == b2.meshes.at("gear_a"));
    const ExportBundle u1 = to_urdf(m, parts);
    const ExportBundle u2 = to_urdf(m, parts);
    CHECK(u1.xml == u2.xml);
}

// ── Gated MuJoCo round-trip (runs only where python3 + mujoco exist) ──

#include <cstdio>
#include <fstream>
#include <cstdlib>
#include <filesystem>

namespace {

bool python_has_mujoco()
{
    const int rc = std::system("python3 -c \"import mujoco\" >/dev/null 2>&1");
    return rc == 0;
}

} // namespace

TEST_CASE("export: MJCF loads in MuJoCo (gated)") {
    if (!python_has_mujoco())
    {
        MESSAGE("skipped: python3 mujoco module not available");
        return;
    }
    Part a = as_part("gear_a", generate_cylinder_mesh(CylinderGeometry{0.05f, 0.1f}));
    Part b = as_part("gear_b", generate_cylinder_mesh(CylinderGeometry{0.04f, 0.1f}));
    Mechanism m;
    m.joints.push_back({"j_a", JointKind::Continuous, "", "gear_a", {0,0,0}, {0,0,1}, -1e30f, 1e30f, 10.0f, 20.0f});
    m.joints.push_back({"j_b", JointKind::Continuous, "", "gear_b", {0.2f,0,0}, {0,0,1}, -1e30f, 1e30f, 10.0f, 20.0f});
    m.couplings.push_back({"g", CouplingKind::Gear, "j_a", "j_b", -2.0f});
    m.driver_joint = "j_a";
    const std::vector<Part> parts{a, b};

    // also round-trip the full steam engine when it's cheap
    const SteamEngineResult se = generate_steam_engine(SteamEngineDefinition{});

    std::filesystem::path dir = std::filesystem::temp_directory_path() / "exd_mjcf_roundtrip";
    std::filesystem::create_directories(dir);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    int checked = 0;
    const auto check_bundle = [&](const char* label, const ExportBundle& bundle) {
        const auto xmlPath = dir / (std::string(label) + ".xml");
        std::ofstream xml(xmlPath);
        xml << bundle.xml;
        xml.close();
        for (const auto& [name, obj] : bundle.meshes)
        {
            std::ofstream o(dir / (name + ".obj"));
            o << obj;
            o.close();
        }
        const std::string cmd = "python3 -c \"import mujoco; " 
            "mj = mujoco.MjModel.from_xml_path('" + xmlPath.string() + "'); "
            "print('loaded', mj.nbody, 'bodies')\"";
        const int rc = std::system(cmd.c_str());
        CAPTURE(label);
        CHECK(rc == 0);
        checked++;
    };
    check_bundle("gearbox", to_mjcf(m, parts));
    check_bundle("steam", to_mjcf(se.mechanism, se.body));
    CHECK(checked == 2);
}

TEST_CASE("export: CADModel overload resolves material density") {
    Part box = generate_box_part(BoxGeometry{{0.1f, 0.1f, 0.1f}});
    box.name = "block";
    box.meta.material = "steel-1045";
    CADModel m = make_cad_model("matmodel", std::vector<Part>{box});
    m.materials = MaterialDB::defaults();

    // resolved through MaterialDB (steel 7850) → geom density attribute
    const std::string mjcf = to_mjcf(m).xml;
    CHECK(mjcf.find("density=\"7850\"") != std::string::npos);
    const std::string urdf = to_urdf(m).xml;
    // steel inertia vs. water default differs by ~7.85×
    const std::string water = to_urdf(m, ExportOptions{}).xml;
    CHECK(urdf.find("<mass value=") != std::string::npos);
    CHECK(water.find("<mass value=") != std::string::npos);
}

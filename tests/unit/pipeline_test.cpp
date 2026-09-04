#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <string>
#include <vector>

using namespace exd::geometry;

namespace
{

// ── A real machine through the unified pipeline ─────────────────────────────
// Steam engine recipe → CADModel (IR) → every adapter → reimport (MSH/STL).
CADModel steam_model()
{
    const SteamEngineResult engine = generate_steam_engine(SteamEngineDefinition{});
    CADModel m = make_cad_model("steam_engine", engine.body, engine.mechanism);
    m.materials = MaterialDB::defaults();
    for (Part& p : m.parts)
        p.meta.material = "steel-1045";
    return m;
}

size_t count_substr(const std::string& haystack, const std::string& needle)
{
    size_t n = 0, p = 0;
    while ((p = haystack.find(needle, p)) != std::string::npos)
    {
        ++n;
        p += needle.size();
    }
    return n;
}

} // namespace

TEST_CASE("pipeline: steam engine recipe → CADModel validates") {
    const CADModel m = steam_model();
    std::vector<std::string> errs;
    REQUIRE(m.validate(errs));
    CHECK(m.parts.size() >= 10);
    CHECK(m.mechanism.joints.size() == 7);        // steam engine contract
    CHECK(m.material_for(m.parts[0]) != nullptr); // steel-1045 resolves
}

TEST_CASE("pipeline: steam engine exports to every adapter") {
    const CADModel m = steam_model();

    const std::string obj = to_obj(m);
    CHECK(obj.find("o ") != std::string::npos);
    CHECK(count_substr(obj, "f ") >= 100);

    const std::string stl = to_stl_binary(m);
    CHECK(stl.size() > 1000u);

    const std::string msh = to_msh(m);
    CHECK(msh.find("$MeshFormat\n2.2") != std::string::npos);
    CHECK(count_substr(msh, "Physical") >= 1);

    const std::string vtk = to_vtk(m);
    CHECK(vtk.find("DATASET UNSTRUCTURED_GRID") != std::string::npos);
    const std::string vtu = to_vtu(m);
    CHECK(vtu.find("<VTKFile type=\"UnstructuredGrid\"") != std::string::npos);

    // every part is a closed watertight solid → one MANIFOLD_SOLID_BREP each
    const std::string step = to_step_faceted(m);
    CHECK(count_substr(step, "MANIFOLD_SOLID_BREP") == m.parts.size());
    CHECK(count_substr(step, "PRODUCT('steel") == 0);   // no bogus products
}

TEST_CASE("pipeline: gmsh round-trip preserves machine topology") {
    const CADModel m = steam_model();
    const CADModel back = import_msh(to_msh(m));

    std::vector<std::string> errs;
    REQUIRE(back.validate(errs));
    REQUIRE(back.parts.size() == m.parts.size());

    // aggregate triangle counts match across the whole machine
    size_t tri = 0, backTri = 0;
    for (const Part& p : m.parts)
        tri += p.mesh.indices.size() / 3;
    for (const Part& p : back.parts)
        backTri += p.mesh.indices.size() / 3;
    CHECK(tri == backTri);

    // a known patched part name survived (e.g. cylinder.wall → wall)
    bool foundCylinderWall = false;
    for (const Part& p : back.parts)
        for (const Patch& pa : p.patches)
            if (p.name == "cylinder" && pa.name == "wall")
                foundCylinderWall = true;
    CHECK(foundCylinderWall);
}

TEST_CASE("pipeline: stl round-trip carries the whole machine surface") {
    const CADModel m = steam_model();
    const CADModel back = import_stl(to_stl_binary(m));
    size_t tri = 0;
    for (const Part& p : m.parts)
        tri += p.mesh.indices.size() / 3;
    REQUIRE(back.parts.size() == 1);
    CHECK(back.parts[0].mesh.indices.size() == tri * 3);
}

#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <string>
#include <vector>

using namespace exd::geometry;

namespace
{

MeshData box_mesh()
{
    return generate_box_mesh(BoxGeometry{{0.3f, 0.2f, 0.2f}});   // 24 verts, 12 tris
}

CADModel cae_model()
{
    Part box = generate_box_part(BoxGeometry{{0.3f, 0.2f, 0.2f}});  // "box", patches +x..-z
    box.name = "casing";
    box.meta.material = "aluminum-6061";
    Part cyl = generate_cylinder_part(CylinderGeometry{0.06f, 0.12f});
    cyl.name = "rotor";
    cyl.meta.material = "steel-1045";
    CADModel m = make_cad_model("pump", std::vector<Part>{box, cyl});
    m.materials = MaterialDB::defaults();
    return m;
}

const Part* find_part(const CADModel& m, const std::string& name)
{
    for (const Part& p : m.parts)
        if (p.name == name)
            return &p;
    return nullptr;
}

} // namespace

TEST_CASE("import: obj round-trip preserves topology and positions") {
    const MeshData src = box_mesh();
    const MeshData m = parse_obj(to_obj(src));
    CHECK(m.vertices.size() == 24);
    CHECK(m.indices.size() == 36);
    CHECK(m.bounds.max.x == doctest::Approx(0.15f));
    CHECK(m.bounds.min.y == doctest::Approx(-0.1f));
    // vertex indexing round-trips: first vertex of the +x face
    CHECK(m.vertices[0].position.x == doctest::Approx(0.15f));
}

TEST_CASE("import: stl ascii round-trip (triangle soup, no welding)") {
    const MeshData src = box_mesh();
    const MeshData m = parse_stl(to_stl_ascii(src));
    CHECK(m.vertices.size() == 36);     // 12 triangles × 3
    CHECK(m.indices.size() == 36);
    // first facet is the box +x face: all three of its vertices sit at x=+0.15
    CHECK(m.vertices[0].position.x == doctest::Approx(0.15f));
    CHECK(m.vertices[1].position.x == doctest::Approx(0.15f));
    CHECK(m.vertices[2].position.x == doctest::Approx(0.15f));
}

TEST_CASE("import: stl binary round-trip via auto-detect") {
    const MeshData src = box_mesh();
    const std::string bin = to_stl_binary(src);
    const MeshData m = parse_stl(bin);
    CHECK(m.vertices.size() == 36);
    CHECK(m.indices.size() == 36);
    // identical to ascii parse
    const MeshData m2 = parse_stl(to_stl_ascii(src));
    CHECK(m.indices == m2.indices);
    for (size_t i = 0; i < m.vertices.size(); ++i)
        CHECK(m.vertices[i].position.z == doctest::Approx(m2.vertices[i].position.z));
}

TEST_CASE("import: stl → CADModel") {
    const CADModel imp = import_stl(to_stl_binary(cae_model()));
    REQUIRE(imp.parts.size() == 1);
    REQUIRE(imp.parts[0].mesh.indices.size() == 3u * (12u + 256u));
    std::vector<std::string> errs;
    CHECK(imp.validate(errs));
}

TEST_CASE("import: gmsh msh 2.2 round-trips patches and parts") {
    const CADModel src = cae_model();
    const std::string msh = to_msh(src);
    const CADModel imp = import_msh(msh);

    REQUIRE(imp.parts.size() == 2);
    const Part* casing = find_part(imp, "casing");
    const Part* rotor  = find_part(imp, "rotor");
    REQUIRE(casing != nullptr);
    REQUIRE(rotor != nullptr);

    // casing: 12 triangles, six two-face patches, names restored
    CHECK(casing->mesh.indices.size() == 36);
    REQUIRE(casing->patches.size() == 6);
    bool sawY = false;
    for (const Patch& pa : casing->patches)
    {
        if (pa.name == "-y")
        {
            sawY = true;
            REQUIRE(pa.faces.size() == 2);
        }
    }
    CHECK(sawY);

    // rotor: 256 triangles across wall/cap_start/cap_end
    CHECK(rotor->mesh.indices.size() == 3u * 256u);
    REQUIRE(rotor->patches.size() == 3);
    const Patch* wall = nullptr;
    for (const Patch& pa : rotor->patches)
        if (pa.name == "wall")
            wall = &pa;
    REQUIRE(wall != nullptr);
    CHECK(wall->faces.size() == 128);

    // round-tripped model stays mesh-sane and validates
    std::vector<std::string> errs;
    CHECK(imp.validate(errs));
}

TEST_CASE("import: gmsh round-trip preserves geometry extents") {
    const CADModel src = cae_model();
    const CADModel imp = import_msh(to_msh(src));
    CHECK(imp.bounds.max.x == doctest::Approx(src.bounds.max.x));
    CHECK(imp.bounds.max.z == doctest::Approx(src.bounds.max.z));
    // determinism
    CHECK(import_msh(to_msh(src)).name == imp.name);
}

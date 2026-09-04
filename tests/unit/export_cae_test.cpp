#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace exd::geometry;

namespace
{

/// Number on the first line that follows a "$Section\n" marker.
size_t section_count(const std::string& text, const std::string& section)
{
    const std::string marker = "$" + section + "\n";
    const size_t p = text.find(marker);
    if (p == std::string::npos)
        return 0;
    const size_t lineEnd = text.find('\n', p + marker.size());
    if (lineEnd == std::string::npos)
        return 0;
    return std::strtoull(text.c_str() + p + marker.size(), nullptr, 10);
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

CADModel box_model()
{
    Part box = generate_box_part(BoxGeometry{{0.3f, 0.2f, 0.2f}}); // 24 verts, 12 tris, 6 patches
    box.name = "casing";
    box.patches[0].semantic = PatchSemantic::Inlet;
    box.patches[0].mesh_size = 0.01f;
    return make_cad_model("box", std::vector<Part>{box});
}

CADModel cae_model()
{
    Part box = generate_box_part(BoxGeometry{{0.3f, 0.2f, 0.2f}});
    box.name = "casing";
    box.meta.material = "aluminum-6061";
    Part cyl = generate_cylinder_part(CylinderGeometry{0.06f, 0.12f}); // 256 tris, 3 patches
    cyl.name = "rotor";
    cyl.meta.material = "steel-1045";
    CADModel m = make_cad_model("pump", std::vector<Part>{box, cyl});
    m.materials = MaterialDB::defaults();
    return m;
}

} // namespace

TEST_CASE("cae_export: obj(CADModel) concatenates parts with object markers") {
    const std::string obj = to_obj(cae_model());
    CHECK(obj.find("o casing") != std::string::npos);
    CHECK(obj.find("o rotor") != std::string::npos);
    CHECK(count_substr(obj, "v ") >= 24);
    CHECK(count_substr(obj, "f ") == 12 + 256);
}

TEST_CASE("cae_export: stl ascii has one solid per part and correct facet count") {
    const CADModel m = box_model();
    const std::string s = to_stl_ascii(m);
    CHECK(s.find("solid casing") != std::string::npos);
    CHECK(count_substr(s, "facet normal") == 12);
    CHECK(s.find("endsolid casing") != std::string::npos);
    // single-part overload
    const std::string single = to_stl_ascii(m.parts[0].mesh);
    CHECK(single.find("solid mesh") != std::string::npos);
    CHECK(count_substr(single, "facet normal") == 12);
    // determinism
    CHECK(to_stl_ascii(m) == s);
}

TEST_CASE("cae_export: stl binary is parseable and byte-count exact") {
    const CADModel m = box_model();
    const std::string b = to_stl_binary(m);
    REQUIRE(b.size() == 80u + 4u + 12u * 50u);                 // header + count + 12×50 bytes

    uint32_t count = 0;
    std::memcpy(&count, b.data() + 80, 4);
    CHECK(count == 12);

    // first triangle of the box:+x face has normal (1,0,0)
    float n[3];
    std::memcpy(n, b.data() + 84, sizeof(n));
    CHECK(n[0] == doctest::Approx(1.0f));
    CHECK(n[1] == doctest::Approx(0.0f));
    CHECK(n[2] == doctest::Approx(0.0f));

    // single-part overload: 12 triangles * 50 bytes; model has 2 parts
    const CADModel two = cae_model();
    REQUIRE(to_stl_binary(two).size() == 2u * (80u + 4u) + (12u + 256u) * 50u);
    REQUIRE(to_stl_binary(two) != b);                          // rotor is present
}

TEST_CASE("cae_export: gmsh msh 2.2 maps patches to physical surfaces") {
    const CADModel m = box_model();
    const std::string msh = to_msh(m);

    CHECK(msh.find("$MeshFormat\n2.2 0 8") != std::string::npos);
    CHECK(section_count(msh, "Nodes") == 24);        // box vertices
    CHECK(section_count(msh, "Elements") == 12);     // box triangles

    // patch groups: six physical surfaces, all "casing.<name>" names present
    CHECK(msh.find("\"casing.+x\"") != std::string::npos);
    CHECK(msh.find("\"casing.-y\"") != std::string::npos);
    CHECK(msh.find("\"casing.+z\"") != std::string::npos);
    // every face of a fully-patched part still lands in a Physical group
    CHECK(count_substr(msh, "$EndElements") == 1);
    const size_t physCount = section_count(msh, "PhysicalNames");
    CHECK(physCount == 6);

    // two-part model: totals and cross-part group naming
    const std::string m2 = to_msh(cae_model());
    CHECK(section_count(m2, "Elements") == 12 + 256);
    CHECK(m2.find("\"casing.+x\"") != std::string::npos);
    CHECK(m2.find("\"rotor.wall\"") != std::string::npos);
    CHECK(m2.find("\"rotor.cap_start\"") != std::string::npos);
    // determinism
    CHECK(to_msh(m) == msh);
}

TEST_CASE("cae_export: vtk legacy emits grid, types and cell scalars") {
    const CADModel m = box_model();
    const std::string v = to_vtk(m);

    CHECK(v.find("# vtk DataFile Version 3.0") != std::string::npos);
    CHECK(v.find("DATASET UNSTRUCTURED_GRID") != std::string::npos);
    CHECK(v.find("POINTS 24 float") != std::string::npos);
    CHECK(v.find("CELLS 12 48") != std::string::npos);
    CHECK(v.find("CELL_TYPES 12") != std::string::npos);
    CHECK(v.find("SCALARS PartID int 1") != std::string::npos);
    CHECK(v.find("SCALARS PatchID int 1") != std::string::npos);

    // PatchID block carries the 1..6 patch ids (one per box face pair)
    const size_t pidPos = v.find("SCALARS PatchID");
    REQUIRE(pidPos != std::string::npos);
    const size_t lut = v.find("LOOKUP_TABLE default", pidPos);
    REQUIRE(lut != std::string::npos);
    std::vector<int> patchIds;
    {
        std::istringstream iss(v.substr(lut + std::string("LOOKUP_TABLE default\n").size()));
        int val;
        while (iss >> val) patchIds.push_back(val);
    }
    REQUIRE(patchIds.size() == 12);
    CHECK(patchIds.front() == 1);
    int maxPatch = *std::max_element(patchIds.begin(), patchIds.end());
    CHECK(maxPatch == 6);

    // unpatched part → PatchID stays 0 for every cell
    CADModel plain = make_cad_model("plain", std::vector<Part>{as_part("plain", generate_box_mesh(BoxGeometry{{0.1f, 0.1f, 0.1f}}))});
    const std::string v2 = to_vtk(plain);
    const size_t pid2 = v2.find("SCALARS PatchID");
    REQUIRE(pid2 != std::string::npos);
    const size_t lut2 = v2.find("LOOKUP_TABLE default", pid2);
    REQUIRE(lut2 != std::string::npos);
    std::vector<int> zeros;
    {
        std::istringstream iss(v2.substr(lut2 + std::string("LOOKUP_TABLE default\n").size()));
        int val;
        while (iss >> val) zeros.push_back(val);
    }
    REQUIRE(zeros.size() == 12);
    CHECK(std::all_of(zeros.begin(), zeros.end(), [](int x) { return x == 0; }));
}

bool step_refs_consistent(const std::string& s, size_t& nDefined, size_t& nUsed)
{
    std::set<int> defined, used;
    nDefined = nUsed = 0;
    size_t i = 0;
    while ((i = s.find('#', i)) != std::string::npos)
    {
        size_t j = i + 1;
        int v = 0;
        bool ok = false;
        while (j < s.size() && std::isdigit(static_cast<unsigned char>(s[j])))
        {
            v = v * 10 + (s[j] - '0');
            ok = true;
            ++j;
        }
        if (!ok || v <= 0)
        {
            ++i;
            continue;
        }
        if (j < s.size() && s[j] == '=')
        {
            defined.insert(v);
            ++nDefined;
        }
        else
        {
            used.insert(v);
        }
        i = j;
    }
    nUsed = used.size();
    for (const int r : used)
        if (!defined.count(r))
            return false;
    return true;
}

size_t char_count(const std::string& s, char c)
{
    return static_cast<size_t>(std::count(s.begin(), s.end(), c));
}

TEST_CASE("cae_export: step faceted brep emits closed solid per watertight part") {
    const CADModel m = box_model();
    const std::string st = to_step_faceted(m);

    CHECK(st.find("ISO-10303-21;") != std::string::npos);
    CHECK(st.find("FILE_SCHEMA(('AUTOMOTIVE_DESIGN") != std::string::npos);
    // one closed solid: box has 12 triangles → 12 ADVANCED_FACEs
    CHECK(count_substr(st, "MANIFOLD_SOLID_BREP") == 1);
    CHECK(count_substr(st, "CLOSED_SHELL") == 1);
    CHECK(count_substr(st, "ADVANCED_FACE") == 12);
    CHECK(st.rfind("END-ISO-10303-21;") != std::string::npos);
    CHECK(st.find("ENDSEC;\nEND-ISO-10303-21;") != std::string::npos);

    // entity references are internally consistent + parens balanced
    size_t nDef = 0, nUse = 0;
    CHECK(step_refs_consistent(st, nDef, nUse));
    REQUIRE(nDef > 0);
    CHECK(char_count(st, '(') == char_count(st, ')'));
}

TEST_CASE("cae_export: step skips non-watertight parts") {
    CADModel m = make_cad_model("open", std::vector<Part>{
        as_part("open_plane", generate_plane_mesh(PlaneGeometry{})),   // 2 triangles, open
        generate_box_part(BoxGeometry{{1.0f, 1.0f, 1.0f}})});
    const std::string st = to_step_faceted(m);
    CHECK(count_substr(st, "MANIFOLD_SOLID_BREP") == 1);   // only the box
    CHECK(st.find("open_plane") == std::string::npos);
    // determinism
    CHECK(to_step_faceted(m) == st);
}

TEST_CASE("cae_export: step two-part model has two named solids") {
    const std::string st = to_step_faceted(cae_model());
    CHECK(count_substr(st, "MANIFOLD_SOLID_BREP") == 2);
    CHECK(st.find("PRODUCT('casing'") != std::string::npos);
    CHECK(st.find("PRODUCT('rotor'") != std::string::npos);
    size_t nDef = 0, nUse = 0;
    CHECK(step_refs_consistent(st, nDef, nUse));
    CHECK(char_count(st, '(') == char_count(st, ')'));
}

// ── Gated STEP round-trip (runs only where gmsh + OCC exist) ──
bool gmsh_available()
{
    const int rc = std::system("command -v gmsh >/dev/null 2>&1");
    return rc == 0;
}

TEST_CASE("cae_export: step loads in gmsh (gated)") {
    if (!gmsh_available())
    {
        MESSAGE("skipped: gmsh executable not available");
        return;
    }
    const std::filesystem::path dir = std::filesystem::temp_directory_path();
    const auto path = dir / "exd_step_gate.step";
    {
        std::ofstream f(path);
        f << to_step_faceted(cae_model());
    }
    const std::string cmd = "gmsh " + path.string() + " -o /dev/null 2>/dev/null";
    CHECK(std::system(cmd.c_str()) == 0);
    std::filesystem::remove(path);
}

TEST_CASE("cae_export: vtu xml is structurally consistent") {
    const CADModel m = box_model();
    const std::string v = to_vtu(m);

    CHECK(v.find("<VTKFile type=\"UnstructuredGrid\"") != std::string::npos);
    CHECK(v.find("<Piece NumberOfPoints=\"24\" NumberOfCells=\"12\">") != std::string::npos);
    CHECK(v.find("Name=\"connectivity\"") != std::string::npos);
    CHECK(v.find("Name=\"offsets\"") != std::string::npos);
    CHECK(v.find("Name=\"types\"") != std::string::npos);
    CHECK(v.find("Name=\"PartID\"") != std::string::npos);
    CHECK(v.find("Name=\"PatchID\"") != std::string::npos);

    // connectivity block must contain exactly 36 tokens (12 tris × 3)
    const size_t con = v.find("Name=\"connectivity\"");
    REQUIRE(con != std::string::npos);
    const size_t gt = v.find('>', con);
    const size_t close = v.find("</DataArray>", gt);
    REQUIRE(close != std::string::npos);
    std::istringstream iss(v.substr(gt + 1, close - gt - 1));
    int tokens = 0, val;
    while (iss >> val) ++tokens;
    CHECK(tokens == 36);

    CHECK(v.find("</VTKFile>") != std::string::npos);
    CHECK(to_vtu(m) == v);   // determinism
}

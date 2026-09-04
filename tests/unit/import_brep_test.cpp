#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <filesystem>
#include <string>
#include <vector>

using namespace exd::geometry;

TEST_CASE("brep: backend availability is consistent") {
    CHECK(occt_available() == (EXD_GEOMETRY_HAS_OCCT == 1));
    if (occt_available())
        CHECK(occt_version().empty() == false);
    else
        CHECK(occt_version() == "none (stub build)");
}

TEST_CASE("brep: analytic STEP round-trip (gated on OCCT)") {
    if (!occt_available())
    {
        MESSAGE("skipped: OpenCASCADE not present (rebuild with -DEXD_GEOMETRY_ENABLE_OCCT=ON)");
        return;
    }

    Part box = generate_box_part(BoxGeometry{{0.2f, 0.3f, 0.1f}});
    box.name = "box";
    Part cyl = generate_cylinder_part(CylinderGeometry{0.05f, 0.12f});
    cyl.name = "cyl";
    const CADModel src = make_cad_model("roundtrip", std::vector<Part>{box, cyl});

    const std::filesystem::path dir = std::filesystem::temp_directory_path();
    const auto stepPath = dir / "exd_geometry_brep_roundtrip.step";

    std::string err;
    REQUIRE(export_brep_step(src, stepPath.string(), err));

    const BrepImportResult r = import_brep_file(stepPath.string());
    REQUIRE(r.ok);
    REQUIRE_FALSE(r.model.parts.empty());
    REQUIRE_FALSE(r.model.parts.front().mesh.vertices.empty());

    std::vector<std::string> errs;
    CHECK(r.model.validate(errs));

    std::filesystem::remove(stepPath);
}

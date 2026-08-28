#include <doctest/doctest.h>

#include <exd/geometry/geometry.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace exd::geometry;

// ── Construction helpers ────────────────────────────────────────────────────

TEST_CASE("part: as_part wraps mesh and keeps name")
{
    Part p = as_part("x", generate_box_mesh({}));
    CHECK(p.name == "x");
    CHECK(p.patches.empty());
    CHECK_FALSE(p.mesh.vertices.empty());
    // MeshData was moved in: box has 24 verts / 36 indices.
    CHECK(p.mesh.vertices.size() == 24);
    CHECK(p.mesh.indices.size() == 36);
}

TEST_CASE("part: make_patch_range")
{
    Patch p = make_patch_range("side", 10, 4);
    CHECK(p.name == "side");
    CHECK(p.faces == std::vector<uint32_t>({10, 11, 12, 13}));

    Patch empty = make_patch_range("none", 7, 0);
    CHECK(empty.faces.empty());
}

TEST_CASE("part: tag_faces collects ordinals")
{
    Part part = as_part("cyl", generate_cylinder_mesh({})); // S=64, capped
    const uint32_t nFaces = static_cast<uint32_t>(part.mesh.indices.size() / 3);

    tag_faces(part, "even", [](uint32_t f) { return f % 2 == 0; });
    REQUIRE(part.patches.size() == 1);
    const auto& even = part.patches[0];
    REQUIRE(even.faces.size() == nFaces / 2);
    CHECK(even.faces[0] == 0);
    CHECK(even.faces[1] == 2);
    CHECK(even.faces.back() == nFaces - 2);

    // Tagging again with the same name appends.
    tag_faces(part, "even", [](uint32_t f) { return f % 2 == 0; });
    REQUIRE(part.patches.size() == 1);
    CHECK(part.patches[0].faces.size() == nFaces);
    CHECK(part.patches[0].faces[nFaces / 2] == 0);
}

// ── Patch-preserving operations ─────────────────────────────────────────────

TEST_CASE("part: transform_part preserves patch ordinals")
{
    Part cyl = generate_cylinder_part({});   // S=64, capped
    const auto originalIndices = cyl.mesh.indices;
    const auto originalPatches = cyl.patches;
    const auto originalVerts   = cyl.mesh.vertices;

    exd::math::Mat4 translate = exd::math::Mat4::identity();
    translate.m[12] = 1.0f;

    Part moved = transform_part(cyl, translate);
    CHECK(moved.name == "cylinder");
    // Positions shifted by +X.
    REQUIRE(moved.mesh.vertices.size() == originalVerts.size());
    CHECK(moved.mesh.vertices[0].position.x == doctest::Approx(originalVerts[0].position.x + 1.0f));
    // Indices byte-identical.
    CHECK(moved.mesh.indices == originalIndices);
    // Patches identical.
    REQUIRE(moved.patches.size() == originalPatches.size());
    for (size_t i = 0; i < originalPatches.size(); ++i) {
        CHECK(moved.patches[i].name == originalPatches[i].name);
        CHECK(moved.patches[i].faces == originalPatches[i].faces);
    }

    // The cap_start patch still addresses the (moved) cap center vertex 130.
    const uint32_t capCenter = 2u * (64u + 1u); // 2*(S+1) = 130
    const Patch* cap_start = nullptr;
    for (const auto& p : moved.patches)
        if (p.name == "cap_start") cap_start = &p;
    REQUIRE(cap_start != nullptr);
    for (uint32_t f : cap_start->faces) {
        bool hasCenter = (moved.mesh.indices[3 * f + 0] == capCenter) ||
                         (moved.mesh.indices[3 * f + 1] == capCenter) ||
                         (moved.mesh.indices[3 * f + 2] == capCenter);
        CHECK(hasCenter);
    }
}

TEST_CASE("part: merge_parts unions bounds, keeps local patches")
{
    auto translate_x = [](float x) {
        exd::math::Mat4 t = exd::math::Mat4::identity();
        t.m[12] = x;
        return t;
    };
    Part a = transform_part(generate_box_part({}), translate_x(-1.0f));
    Part b = transform_part(generate_box_part({}), translate_x(1.0f));

    auto aPatches = a.patches;
    auto bPatches = b.patches;

    std::vector<Part> parts = {a, b};
    Assembly asm_ = merge_parts(parts);

    REQUIRE(asm_.parts.size() == 2);
    // Bounds cover both boxes (default size 1, half 0.5).
    CHECK(asm_.bounds.min.x == doctest::Approx(-1.5f));
    CHECK(asm_.bounds.max.x == doctest::Approx(1.5f));
    CHECK(asm_.bounds.min.y == doctest::Approx(-0.5f));
    CHECK(asm_.bounds.max.y == doctest::Approx(0.5f));

    // Local patches untouched.
    CHECK(asm_.parts[0].name == "box");
    CHECK(asm_.parts[0].patches.size() == 6);
    CHECK(asm_.parts[1].patches.size() == 6);
    for (size_t i = 0; i < 6; ++i) {
        CHECK(asm_.parts[0].patches[i].name == aPatches[i].name);
        CHECK(asm_.parts[0].patches[i].faces == aPatches[i].faces);
        CHECK(asm_.parts[1].patches[i].name == bPatches[i].name);
        CHECK(asm_.parts[1].patches[i].faces == bPatches[i].faces);
    }
}

TEST_CASE("part: flatten remaps and prefixes")
{
    const uint32_t S = 64;
    Part cyl = generate_cylinder_part({});
    cyl.name = "cyl";                          // native name is "cylinder"
    Part box = generate_box_part({});

    Assembly asm_;
    asm_.parts.push_back(cyl);
    asm_.parts.push_back(box);

    Part flat = flatten(asm_);
    CHECK(flat.name.empty());               // multi-part flatten has no name

    // 3 cylinder patches + 6 box patches = 9 total.
    REQUIRE(flat.patches.size() == 9);
    CHECK(flat.patches[0].name == "cyl.wall");
    CHECK(flat.patches[1].name == "cyl.cap_start");
    CHECK(flat.patches[2].name == "cyl.cap_end");
    CHECK(flat.patches[3].name == "box.+x");
    CHECK(flat.patches[4].name == "box.-x");
    CHECK(flat.patches[5].name == "box.+y");
    CHECK(flat.patches[6].name == "box.-y");
    CHECK(flat.patches[7].name == "box.+z");
    CHECK(flat.patches[8].name == "box.-z");

    // Merged face count = cylinder (2S + S + S) + box (12).
    const uint32_t cylFaces = 2 * S + S + S;
    CHECK(flat.mesh.indices.size() / 3 == cylFaces + 12);

    // cyl.wall face 0 still references the cylinder's first triangle,
    // which is (vertex 0, S+1, 1) in the source mesh (offset 0 after merge).
    CHECK(flat.patches[0].faces.front() == 0);
    CHECK(flat.mesh.indices[0] == 0);
    CHECK(flat.mesh.indices[1] == S + 1);
    CHECK(flat.mesh.indices[2] == 1);

    // box.+x triangles are remapped by the cylinder's triangle offset.
    CHECK(flat.patches[3].faces == std::vector<uint32_t>({cylFaces, cylFaces + 1}));

    // Cylinder cap bounds keep face ordinals in the merged mesh: cap_start
    // face 0 = triangle 2S, which references the cap center 130.
    CHECK(flat.patches[1].faces.front() == 2 * S);
    const uint32_t capCenter = 2u * (S + 1u);              // 130
    const uint32_t f0 = flat.patches[1].faces.front();
    bool hasCenter = (flat.mesh.indices[3 * f0] == capCenter) ||
                     (flat.mesh.indices[3 * f0 + 1] == capCenter) ||
                     (flat.mesh.indices[3 * f0 + 2] == capCenter);
    CHECK(hasCenter);
}

TEST_CASE("part: flatten single part returns unchanged")
{
    Part solo = as_part("solo", generate_box_mesh({}));
    solo.patches.push_back(make_patch_range("surface", 0, 12));

    Assembly asm_;
    asm_.parts.push_back(solo);

    Part flat = flatten(asm_);
    CHECK(flat.name == "solo");
    REQUIRE(flat.patches.size() == 1);
    CHECK(flat.patches[0].name == "surface");    // unprefixed
    CHECK(flat.patches[0].faces.size() == 12);
    CHECK(flat.mesh.vertices.size() == solo.mesh.vertices.size());
}

TEST_CASE("part: flatten empty assembly returns empty part")
{
    Assembly asm_;
    Part flat = flatten(asm_);
    CHECK(flat.name.empty());
    CHECK(flat.mesh.vertices.empty());
    CHECK(flat.patches.empty());
}

// ── Native patches ──────────────────────────────────────────────────────────

TEST_CASE("cylinder part: patches match layout")
{
    {   // capped default (S=64)
        Part part = generate_cylinder_part({});
        REQUIRE(part.name == "cylinder");
        REQUIRE(part.patches.size() == 3);

        const Patch& wall      = part.patches[0];
        const Patch& cap_start = part.patches[1];
        const Patch& cap_end   = part.patches[2];
        CHECK(wall.name == "wall");
        CHECK(wall.faces.size() == 128);
        CHECK(cap_start.name == "cap_start");
        CHECK(cap_start.faces.size() == 64);
        CHECK(cap_end.name == "cap_end");
        CHECK(cap_end.faces.size() == 64);

        // Partition of all 192... 256 faces, no overlap.
        const uint32_t total = static_cast<uint32_t>(part.mesh.indices.size() / 3);
        CHECK(total == 256);
        REQUIRE_FALSE(wall.faces.empty());
        REQUIRE_FALSE(cap_start.faces.empty());
        REQUIRE_FALSE(cap_end.faces.empty());
        CHECK(wall.faces.back() == 127);
        CHECK(cap_start.faces.front() == 128);
        CHECK(cap_end.faces.front() == 192);
        CHECK(cap_end.faces.back() == 255);
        for (const auto& w : wall.faces)      CHECK(w < total);
        for (const auto& c : cap_start.faces) CHECK(c < total);
        for (const auto& c : cap_end.faces)   CHECK(c < total);

        // Cap fans reference their cap center vertices: 2*(S+1) and 2*(S+1)+1.
        const uint32_t bot = 2u * (64u + 1u);        // 130
        const uint32_t top = 2u * (64u + 1u) + 1u;   // 131
        for (uint32_t f : cap_start.faces) {
            bool has = (part.mesh.indices[3 * f] == bot) ||
                       (part.mesh.indices[3 * f + 1] == bot) ||
                       (part.mesh.indices[3 * f + 2] == bot);
            CHECK(has);
        }
        for (uint32_t f : cap_end.faces) {
            bool has = (part.mesh.indices[3 * f] == top) ||
                       (part.mesh.indices[3 * f + 1] == top) ||
                       (part.mesh.indices[3 * f + 2] == top);
            CHECK(has);
        }
    }

    {   // uncapped
        CylinderGeometry geom;
        geom.capped = false;
        Part part = generate_cylinder_part(geom);
        REQUIRE(part.patches.size() == 1);
        CHECK(part.patches[0].name == "wall");
        CHECK(part.patches[0].faces.size() == 128);
    }
}

TEST_CASE("cone part: patches match layout")
{
    {   // capped default (S=64)
        Part part = generate_cone_part({});
        REQUIRE(part.name == "cone");
        REQUIRE(part.patches.size() == 2);
        const Patch& wall      = part.patches[0];
        const Patch& cap_start = part.patches[1];
        CHECK(wall.name == "wall");
        CHECK(wall.faces.size() == 64);
        CHECK(cap_start.name == "cap_start");
        CHECK(cap_start.faces.size() == 64);
        CHECK(wall.faces.back() == 63);

        // cap_start triangles contain the cap center vertex S+2 = 66.
        const uint32_t center = 64u + 2u;
        for (uint32_t f : cap_start.faces) {
            bool has = (part.mesh.indices[3 * f] == center) ||
                       (part.mesh.indices[3 * f + 1] == center) ||
                       (part.mesh.indices[3 * f + 2] == center);
            CHECK(has);
        }
    }

    {   // uncapped
        ConeGeometry geom;
        geom.capped = false;
        Part part = generate_cone_part(geom);
        REQUIRE(part.patches.size() == 1);
        CHECK(part.patches[0].name == "wall");
        CHECK(part.patches[0].faces.size() == 64);
    }
}

TEST_CASE("box part: six named faces")
{
    Part part = generate_box_part({});
    REQUIRE(part.name == "box");
    REQUIRE(part.patches.size() == 6);

    const std::string names[] = {"+x", "-x", "+y", "-y", "+z", "-z"};
    for (uint32_t i = 0; i < 6; ++i) {
        CHECK(part.patches[i].name == names[i]);
        CHECK(part.patches[i].faces == std::vector<uint32_t>({2 * i, 2 * i + 1}));
    }

    uint32_t totalFaces = 0;
    for (const auto& p : part.patches) totalFaces += static_cast<uint32_t>(p.faces.size());
    CHECK(totalFaces == static_cast<uint32_t>(part.mesh.indices.size() / 3));
    CHECK(totalFaces == 12);
}

TEST_CASE("extrusion part: patches match layout")
{
    ExtrusionGeometry geom;
    geom.profile = {{-0.5f, -0.5f, 0.0f}, {0.5f, -0.5f, 0.0f},
                    {0.5f,  0.5f, 0.0f}, {-0.5f,  0.5f, 0.0f}};
    geom.depth = 1.0f;

    {   // capped
        Part part = generate_extrusion_part(geom);
        REQUIRE(part.name == "extrusion");
        REQUIRE(part.patches.size() == 3);
        const Patch& wall      = part.patches[0];
        const Patch& cap_start = part.patches[1];
        const Patch& cap_end   = part.patches[2];
        CHECK(wall.name == "wall");
        CHECK(wall.faces.size() == 8);
        CHECK(cap_start.name == "cap_start");
        CHECK(cap_start.faces.size() == 4);
        CHECK(cap_end.name == "cap_end");
        CHECK(cap_end.faces.size() == 4);

        // First wall face = original triangle 0: front ring indices (0, 4, 1).
        CHECK(wall.faces.front() == 0);
        CHECK(part.mesh.indices[0] == 0);
        CHECK(part.mesh.indices[1] == 4);
        CHECK(part.mesh.indices[2] == 1);
    }

    {   // uncapped → wall only
        ExtrusionGeometry uncapped = geom;
        uncapped.capped = false;
        Part part = generate_extrusion_part(uncapped);
        REQUIRE(part.patches.size() == 1);
        CHECK(part.patches[0].name == "wall");
        CHECK(part.patches[0].faces.size() == 8);
    }
}

TEST_CASE("lathe part: patches match layout")
{
    LatheGeometry geom;
    geom.segments = 16;
    geom.profile = {{0.5f, 0.0f, 0.0f}, {0.6f, 1.0f, 0.0f}, {0.5f, 2.0f, 0.0f}};
    // SEG=16, nProf=3 → sideCount = 2*16*2 = 64; both ends off-axis → both caps.
    {
        Part part = generate_lathe_part(geom);
        REQUIRE(part.name == "lathe");
        REQUIRE(part.patches.size() == 3);
        CHECK(part.patches[0].name == "surface");
        CHECK(part.patches[0].faces.size() == 64);
        CHECK(part.patches[1].name == "cap_start");
        CHECK(part.patches[1].faces.size() == 16);
        CHECK(part.patches[2].name == "cap_end");
        CHECK(part.patches[2].faces.size() == 16);
        CHECK(part.patches[1].faces.front() == 64);
        CHECK(part.patches[2].faces.front() == 80);
    }

    // Pointed first profile point (x = 0.0f → |x| < 1e-5) → no cap_start.
    {
        LatheGeometry pointed = geom;
        pointed.profile[0] = {0.0f, 0.0f, 0.0f};
        Part part = generate_lathe_part(pointed);
        REQUIRE(part.patches.size() == 2);
        CHECK(part.patches[0].name == "surface");
        CHECK(part.patches[0].faces.size() == 64);
        CHECK(part.patches[1].name == "cap_end");
        CHECK(part.patches[1].faces.size() == 16);
        CHECK(part.patches[1].faces.front() == 64);
    }
}

// ── Turbine assembly ────────────────────────────────────────────────────────

namespace {

FlowPath part_test_flow_path()
{
    FlowPath f;
    f.hub_points    = {{0.0f, 0.4f}, {1.0f, 0.4f}, {2.0f, 0.4f}};
    f.shroud_points = {{0.0f, 1.0f}, {1.0f, 1.0f}, {2.0f, 1.0f}};
    f.tip_clearance = {0.01f, 0.0f, 0.02f, "m", false};
    return f;
}

BladeRow part_test_rotor()
{
    BladeRow r;
    r.type = BladeRowType::Rotor;
    r.blade_count = {8, 1, 200, "", false};
    r.leading_edge_hub     = {0.5f, 0.4f};
    r.leading_edge_shroud  = {0.5f, 1.0f};
    r.trailing_edge_hub    = {0.9f, 0.4f};
    r.trailing_edge_shroud = {0.9f, 1.0f};
    return r;
}

TurbineDefinition part_test_turbine()
{
    TurbineDefinition t;
    t.flow_path  = part_test_flow_path();
    t.blade_rows = {part_test_rotor()};
    return t;
}

} // namespace

TEST_CASE("turbine assembly: patched parts per row")
{
    Assembly asm_ = generate_turbine_assembly(part_test_turbine());

    REQUIRE_FALSE(asm_.parts.empty());

    bool has_hub = false, has_flow = false;
    const Part* rotor = nullptr;
    for (const auto& p : asm_.parts) {
        if (p.name == "hub")       has_hub = true;
        if (p.name == "flow_path") has_flow = true;
        if (p.name == "rotor_0")   rotor = &p;
    }
    // Default HubDefinition is a Spinner, so the hub mesh is present.
    CHECK(has_hub);
    CHECK(has_flow);
    REQUIRE(rotor != nullptr);

    const Patch* blade_surface = nullptr;
    const Patch* hub_cap       = nullptr;
    const Patch* shroud_cap    = nullptr;
    for (const auto& p : rotor->patches) {
        if (p.name == "blade_surface") blade_surface = &p;
        if (p.name == "hub_cap")       hub_cap       = &p;
        if (p.name == "shroud_cap")    shroud_cap    = &p;
    }
    REQUIRE(blade_surface != nullptr);
    REQUIRE(hub_cap != nullptr);
    REQUIRE(shroud_cap != nullptr);

    const uint32_t totalFaces = static_cast<uint32_t>(rotor->mesh.indices.size() / 3);
    const uint32_t patchFaces = static_cast<uint32_t>(blade_surface->faces.size() +
                                                      hub_cap->faces.size() +
                                                      shroud_cap->faces.size());
    CHECK(patchFaces == totalFaces);
    CHECK(blade_surface->faces.size() < totalFaces);
    CHECK(hub_cap->faces.size() > 0);
    CHECK(shroud_cap->faces.size() > 0);

    // Non-empty names all carry a single "surface" patch.
    for (const auto& p : asm_.parts) {
        if (p.mesh.vertices.empty()) continue;
        REQUIRE_FALSE(p.patches.empty());
    }
    CHECK(asm_.bounds.min.x <= asm_.bounds.max.x);
    CHECK(asm_.bounds.min.y <= asm_.bounds.max.y);
    CHECK(asm_.bounds.min.z <= asm_.bounds.max.z);
}

TEST_CASE("turbine assembly: mesh identical to generate_turbine_mesh")
{
    const TurbineDefinition t = part_test_turbine();
    Assembly asm_ = generate_turbine_assembly(t);
    REQUIRE(asm_.parts.size() >= 3);

    Part flat = flatten(asm_);
    MeshData whole = generate_turbine_mesh(t);

    CHECK(flat.mesh.vertices.size() == whole.vertices.size());
    CHECK(flat.mesh.indices.size() == whole.indices.size());
    CHECK(flat.mesh.topology == whole.topology);
}
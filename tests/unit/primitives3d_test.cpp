#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>

#include <cmath>

using namespace exd::geometry;

// ── Sphere ─────────────────────────────────────────────────────────────────

TEST_CASE("sphere: default UV sphere")
{
    SphereGeometry geom; // r=0.5, lat=16, lon=32
    auto mesh = generate_sphere_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // (16+1) * (32+1) = 17 * 33 = 561 vertices
    CHECK(mesh.vertices.size() == 561);
    // 16 * 32 * 6 = 3072 indices
    CHECK(mesh.indices.size() == 3072);

    // All vertices at distance radius from origin
    for (const auto& v : mesh.vertices)
    {
        float dist = std::sqrt(v.position.x * v.position.x +
                               v.position.y * v.position.y +
                               v.position.z * v.position.z);
        CHECK(dist == doctest::Approx(0.5f).epsilon(0.001f));
    }

    // Normals should be unit length
    for (const auto& v : mesh.vertices)
    {
        float nlen = std::sqrt(v.normal.x * v.normal.x +
                               v.normal.y * v.normal.y +
                               v.normal.z * v.normal.z);
        CHECK(nlen == doctest::Approx(1.0f).epsilon(0.001f));
    }
}

TEST_CASE("sphere: custom radius scales vertices")
{
    SphereGeometry geom;
    geom.radius = 2.0f;
    geom.latitudeSegments = 8;
    geom.longitudeSegments = 16;
    auto mesh = generate_sphere_mesh(geom);

    CHECK(mesh.vertices.size() == 9 * 17); // 153
    for (const auto& v : mesh.vertices)
    {
        float dist = std::sqrt(v.position.x * v.position.x +
                               v.position.y * v.position.y +
                               v.position.z * v.position.z);
        CHECK(dist == doctest::Approx(2.0f).epsilon(0.001f));
    }
}

TEST_CASE("sphere: poles are at correct Y positions")
{
    SphereGeometry geom;
    geom.radius = 1.0f;
    auto mesh = generate_sphere_mesh(geom);

    // First vertex (theta=0) should be at north pole: (0, 1, 0)
    CHECK(mesh.vertices[0].position.y == doctest::Approx(1.0f));
    CHECK(mesh.vertices[0].position.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].position.z == doctest::Approx(0.0f));

    // Last ring (theta=pi) should be at south pole: (0, -1, 0)
    // The last vertex in the grid
    size_t lastIdx = mesh.vertices.size() - 1;
    CHECK(mesh.vertices[lastIdx].position.y == doctest::Approx(-1.0f));
}

TEST_CASE("sphere: texcoords in [0,1] range when enabled")
{
    SphereGeometry geom;
    geom.generateTexcoords = true;
    auto mesh = generate_sphere_mesh(geom);

    for (const auto& v : mesh.vertices)
    {
        CHECK(v.uv.x >= doctest::Approx(0.0f));
        CHECK(v.uv.x <= doctest::Approx(1.0f));
        CHECK(v.uv.y >= doctest::Approx(0.0f));
        CHECK(v.uv.y <= doctest::Approx(1.0f));
    }
}

TEST_CASE("sphere: icosphere construction mode")
{
    SphereGeometry geom;
    geom.construction = SphereConstruction::Icosphere;
    geom.radius = 1.0f;
    geom.latitudeSegments = 2; // subdivisions
    auto mesh = generate_sphere_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    // All vertices should be on the unit sphere
    for (const auto& v : mesh.vertices)
    {
        float dist = std::sqrt(v.position.x * v.position.x +
                               v.position.y * v.position.y +
                               v.position.z * v.position.z);
        CHECK(dist == doctest::Approx(1.0f).epsilon(0.001f));
    }
}

TEST_CASE("sphere: icosphere with zero subdivisions matches base icosahedron")
{
    SphereGeometry geom;
    geom.construction = SphereConstruction::Icosphere;
    geom.latitudeSegments = 0;
    auto mesh = generate_sphere_mesh(geom);

    CHECK(mesh.vertices.size() == 12);
    CHECK(mesh.indices.size() == 60);
}

// ── Ellipsoid ───────────────────────────────────────────────────────────────

TEST_CASE("ellipsoid: default prolate ellipsoid")
{
    EllipsoidGeometry geom; // radii=(0.5, 1.0, 0.5), lat=16, lon=32
    auto mesh = generate_ellipsoid_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // (16+1) * (32+1) = 561 vertices, same layout as UV sphere
    CHECK(mesh.vertices.size() == 561);
    CHECK(mesh.indices.size() == 3072);

    // All vertices should satisfy ellipsoid equation within tolerance
    for (const auto& v : mesh.vertices)
    {
        float ex = v.position.x / 0.5f;
        float ey = v.position.y / 1.0f;
        float ez = v.position.z / 0.5f;
        CHECK(ex * ex + ey * ey + ez * ez == doctest::Approx(1.0f).epsilon(0.001f));
    }

    // Bounds should match radii
    CHECK(mesh.bounds.min.x == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.x == doctest::Approx(0.5f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.y == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.z == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.z == doctest::Approx(0.5f));
}

TEST_CASE("ellipsoid: custom three-axis radii")
{
    EllipsoidGeometry geom;
    geom.radii = {2.0f, 0.5f, 1.0f};
    geom.latitudeSegments = 8;
    geom.longitudeSegments = 16;
    auto mesh = generate_ellipsoid_mesh(geom);

    CHECK(mesh.vertices.size() == 9 * 17); // (8+1)*(16+1) = 153

    for (const auto& v : mesh.vertices)
    {
        float ex = v.position.x / 2.0f;
        float ey = v.position.y / 0.5f;
        float ez = v.position.z / 1.0f;
        CHECK(ex * ex + ey * ey + ez * ez == doctest::Approx(1.0f).epsilon(0.005f));
    }

    CHECK(mesh.bounds.min.x == doctest::Approx(-2.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(2.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.5f));
    CHECK(mesh.bounds.min.z == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.z == doctest::Approx(1.0f));
}

TEST_CASE("ellipsoid: zero radius returns empty")
{
    EllipsoidGeometry geom;
    geom.radii = {0.0f, 0.5f, 0.5f};
    auto mesh = generate_ellipsoid_mesh(geom);
    CHECK(mesh.vertices.empty());
}

// ── Box ────────────────────────────────────────────────────────────────────

TEST_CASE("box: default unit box")
{
    BoxGeometry geom; // size=(1,1,1)
    auto mesh = generate_box_mesh(geom);

    CHECK(mesh.vertices.size() == 24); // 6 faces × 4
    CHECK(mesh.indices.size() == 36);  // 6 faces × 6
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("box: non-cubic box")
{
    BoxGeometry geom;
    geom.size = {2.0f, 1.0f, 0.5f};
    auto mesh = generate_box_mesh(geom);

    CHECK(mesh.vertices.size() == 24);
    // Note: box generator does not compute bounds; verify via vertex positions instead
    float minX = 1e30f, maxX = -1e30f;
    float minY = 1e30f, maxY = -1e30f;
    float minZ = 1e30f, maxZ = -1e30f;
    for (const auto& v : mesh.vertices)
    {
        minX = std::min(minX, v.position.x);
        maxX = std::max(maxX, v.position.x);
        minY = std::min(minY, v.position.y);
        maxY = std::max(maxY, v.position.y);
        minZ = std::min(minZ, v.position.z);
        maxZ = std::max(maxZ, v.position.z);
    }
    CHECK(minX == doctest::Approx(-1.0f));
    CHECK(maxX == doctest::Approx(1.0f));
    CHECK(minY == doctest::Approx(-0.5f));
    CHECK(maxY == doctest::Approx(0.5f));
    CHECK(minZ == doctest::Approx(-0.25f));
    CHECK(maxZ == doctest::Approx(0.25f));
}

TEST_CASE("box: each face has correct normal")
{
    BoxGeometry geom;
    geom.size = {1.0f, 1.0f, 1.0f};
    auto mesh = generate_box_mesh(geom);

    // 6 faces, 4 vertices each, each face has a unique normal
    // Count unique normals
    int normCount[6] = {0}; // +X, -X, +Y, -Y, +Z, -Z
    for (const auto& v : mesh.vertices)
    {
        if (v.normal.x == doctest::Approx(1.0f))  normCount[0]++;
        if (v.normal.x == doctest::Approx(-1.0f)) normCount[1]++;
        if (v.normal.y == doctest::Approx(1.0f))  normCount[2]++;
        if (v.normal.y == doctest::Approx(-1.0f)) normCount[3]++;
        if (v.normal.z == doctest::Approx(1.0f))  normCount[4]++;
        if (v.normal.z == doctest::Approx(-1.0f)) normCount[5]++;
    }
    // Each face has 4 vertices with the same normal
    CHECK(normCount[0] == 4);
    CHECK(normCount[1] == 4);
    CHECK(normCount[2] == 4);
    CHECK(normCount[3] == 4);
    CHECK(normCount[4] == 4);
    CHECK(normCount[5] == 4);
}

// ── Cylinder ───────────────────────────────────────────────────────────────

TEST_CASE("cylinder: default capped cylinder")
{
    CylinderGeometry geom; // r=0.5, h=1.0, slices=32, capped=true
    auto mesh = generate_cylinder_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());

    // Side: 2*(32+1) = 66, Caps: 2 centers = 68 total
    CHECK(mesh.vertices.size() == 68);

    // Side indices: 32*6 = 192, Cap indices: 32*3*2 = 192, total = 384
    CHECK(mesh.indices.size() == 384);

    // Side vertices at correct Y positions
    for (const auto& v : mesh.vertices)
    {
        // Side vertices should be at Y = ±0.5 or cap centers at Y = ±0.5
        CHECK((v.position.y == doctest::Approx(-0.5f) ||
               v.position.y == doctest::Approx(0.5f)));
    }
}

TEST_CASE("cylinder: zero dimensions return empty")
{
    CylinderGeometry geom;
    geom.radius = 0.0f;
    auto mesh = generate_cylinder_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("cylinder: zero height returns empty")
{
    CylinderGeometry geom;
    geom.height = 0.0f;
    auto mesh = generate_cylinder_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("cylinder: uncapped cylinder has no cap vertices")
{
    CylinderGeometry geom;
    geom.capped = false;
    geom.radius = 0.5f;
    geom.height = 1.0f;
    geom.slices = 32;
    auto mesh = generate_cylinder_mesh(geom);

    // Only side vertices: 2*(32+1) = 66
    CHECK(mesh.vertices.size() == 66);
    // Only side indices: 32*6 = 192
    CHECK(mesh.indices.size() == 192);
}

TEST_CASE("cylinder: bounds are correct")
{
    CylinderGeometry geom;
    geom.radius = 0.5f;
    geom.height = 2.0f;
    auto mesh = generate_cylinder_mesh(geom);

    CHECK(mesh.bounds.min.x == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.x == doctest::Approx(0.5f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.y == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.z == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.z == doctest::Approx(0.5f));
}

// ── Plane ──────────────────────────────────────────────────────────────────

TEST_CASE("plane: default 1x1 segment plane")
{
    PlaneGeometry geom; // size=(1,0,1), segW=1, segD=1
    auto mesh = generate_plane_mesh(geom);

    CHECK(mesh.vertices.size() == 4);  // 2×2 grid
    CHECK(mesh.indices.size() == 6);   // 1 quad × 6
    CHECK(mesh.topology == PrimitiveTopology::Triangles);

    // All vertices at Y=0
    for (const auto& v : mesh.vertices)
        CHECK(v.position.y == doctest::Approx(0.0f));

    // All normals point +Y
    for (const auto& v : mesh.vertices)
        CHECK(v.normal.y == doctest::Approx(1.0f));
}

TEST_CASE("plane: segmented plane")
{
    PlaneGeometry geom;
    geom.segmentsW = 3;
    geom.segmentsD = 2;
    auto mesh = generate_plane_mesh(geom);

    CHECK(mesh.vertices.size() == (3 + 1) * (2 + 1)); // 4*3=12
    CHECK(mesh.indices.size() == 3 * 2 * 6);          // 36
}

TEST_CASE("plane: zero area returns empty")
{
    PlaneGeometry geom;
    geom.size = {0.0f, 0.0f, 0.0f};
    auto mesh = generate_plane_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("plane: zero X dimension returns empty")
{
    PlaneGeometry geom;
    geom.size = {0.0f, 0.0f, 1.0f};
    auto mesh = generate_plane_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("plane: bounds are correct")
{
    PlaneGeometry geom;
    geom.size = {4.0f, 0.0f, 2.0f};
    auto mesh = generate_plane_mesh(geom);

    CHECK(mesh.bounds.min.x == doctest::Approx(-2.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(2.0f));
    CHECK(mesh.bounds.min.z == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.z == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(0.0f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.0f));
}

// ── Capsule ────────────────────────────────────────────────────────────────

TEST_CASE("capsule: default capsule")
{
    CapsuleGeometry geom; // r=0.25, h=1.0, slices=32, stacks=8
    auto mesh = generate_capsule_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());

    // Bounds should span roughly [-0.25, -0.75, -0.25] to [0.25, 0.75, 0.25]
    // minY = -(h/2 + r) = -(0.5 + 0.25) = -0.75
    // maxY = +(h/2 + r) = 0.75
    CHECK(mesh.bounds.min.y == doctest::Approx(-0.75f).epsilon(0.01f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.75f).epsilon(0.01f));
}

TEST_CASE("capsule: zero dimensions return empty")
{
    CapsuleGeometry geom;
    geom.radius = 0.0f;
    geom.height = 0.0f;
    auto mesh = generate_capsule_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("capsule: vertex count matches expected layout")
{
    CapsuleGeometry geom;
    geom.slices = 32;
    geom.stacks = 8;
    auto mesh = generate_capsule_mesh(geom);

    // Top hemisphere: (8+1)*(32+1) = 9*33 = 297
    // Bottom hemisphere: same = 297
    // Total = 594
    CHECK(mesh.vertices.size() == 594);
}

// ── Icosahedron ────────────────────────────────────────────────────────────

TEST_CASE("icosahedron: base icosahedron (subdiv=0)")
{
    auto mesh = generate_icosahedron_mesh(1.0f, 0);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(mesh.vertices.size() == 12); // 12 base vertices
    CHECK(mesh.indices.size() == 60);  // 20 triangles × 3

    // All vertices on unit sphere
    for (const auto& v : mesh.vertices)
    {
        float dist = std::sqrt(v.position.x * v.position.x +
                               v.position.y * v.position.y +
                               v.position.z * v.position.z);
        CHECK(dist == doctest::Approx(1.0f).epsilon(0.001f));
    }
}

TEST_CASE("icosahedron: subdivision level 1")
{
    auto mesh = generate_icosahedron_mesh(1.0f, 1);

    CHECK(mesh.vertices.size() == 42);  // 12 + 30 edge midpoints
    CHECK(mesh.indices.size() == 240);  // 80 triangles × 3
}

TEST_CASE("icosahedron: subdivision level 2")
{
    auto mesh = generate_icosahedron_mesh(1.0f, 2);

    CHECK(mesh.vertices.size() == 162);
    CHECK(mesh.indices.size() == 960);  // 320 triangles × 3
}

TEST_CASE("icosahedron: zero radius returns empty")
{
    auto mesh = generate_icosahedron_mesh(0.0f, 0);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("icosahedron: negative subdivisions clamped to 0")
{
    auto mesh = generate_icosahedron_mesh(1.0f, -1);
    // Should behave like subdiv=0
    CHECK(mesh.vertices.size() == 12);
    CHECK(mesh.indices.size() == 60);
}

TEST_CASE("icosahedron: custom radius scales vertices")
{
    auto mesh = generate_icosahedron_mesh(2.5f, 0);

    for (const auto& v : mesh.vertices)
    {
        float dist = std::sqrt(v.position.x * v.position.x +
                               v.position.y * v.position.y +
                               v.position.z * v.position.z);
        CHECK(dist == doctest::Approx(2.5f).epsilon(0.001f));
    }
}

// ── Torus ──────────────────────────────────────────────────────────────────

TEST_CASE("torus: default torus produces valid mesh")
{
    TorusGeometry geom; // majorR=1.0, minorR=0.3, majorSegs=32, minorSegs=16
    auto mesh = generate_torus_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(mesh.vertices.size() == (32 + 1) * (16 + 1)); // 33*17 = 561
    CHECK(mesh.indices.size() == 32 * 16 * 6);          // 3072
}

TEST_CASE("torus: all vertices at expected distance from tube center")
{
    TorusGeometry geom;
    auto mesh = generate_torus_mesh(geom);

    // Each vertex should be within [majorR-minorR, majorR+minorR] radially
    // and within [-minorR, +minorR] in Y
    for (const auto& v : mesh.vertices)
    {
        float radialDist = std::sqrt(v.position.x * v.position.x + v.position.z * v.position.z);
        CHECK(radialDist >= doctest::Approx(0.7f).epsilon(0.01f));  // majorR - minorR
        CHECK(radialDist <= doctest::Approx(1.3f).epsilon(0.01f));  // majorR + minorR
        CHECK(v.position.y >= doctest::Approx(-0.3f).epsilon(0.01f));
        CHECK(v.position.y <= doctest::Approx(0.3f).epsilon(0.01f));
    }
}

TEST_CASE("torus: bounds are correct")
{
    TorusGeometry geom;
    auto mesh = generate_torus_mesh(geom);

    CHECK(mesh.bounds.min.x == doctest::Approx(-1.3f));  // -(majorR + minorR)
    CHECK(mesh.bounds.max.x == doctest::Approx(1.3f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-0.3f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.3f));
}

TEST_CASE("torus: zero major radius returns empty")
{
    TorusGeometry geom;
    geom.majorRadius = 0.0f;
    auto mesh = generate_torus_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("torus: zero minor radius returns empty")
{
    TorusGeometry geom;
    geom.minorRadius = 0.0f;
    auto mesh = generate_torus_mesh(geom);
    CHECK(mesh.vertices.empty());
}

// ── Cone ───────────────────────────────────────────────────────────────────

TEST_CASE("cone: default capped cone")
{
    ConeGeometry geom; // r=0.5, h=1.0, slices=32, capped=true
    auto mesh = generate_cone_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("cone: tip is at correct position")
{
    ConeGeometry geom; // height=1.0
    auto mesh = generate_cone_mesh(geom);

    // Find the tip vertex (should be at y=0.5)
    bool foundTip = false;
    for (const auto& v : mesh.vertices)
    {
        if (v.position.y == doctest::Approx(0.5f))
        {
            CHECK(v.position.x == doctest::Approx(0.0f));
            CHECK(v.position.z == doctest::Approx(0.0f));
            foundTip = true;
            break;
        }
    }
    CHECK(foundTip);
}

TEST_CASE("cone: base vertices at correct Y")
{
    ConeGeometry geom;
    auto mesh = generate_cone_mesh(geom);

    // All base ring vertices should be at y = -height/2 = -0.5
    // (cap center is also at y=-0.5 but at r=0, so skip that)
    int baseCount = 0;
    for (const auto& v : mesh.vertices)
    {
        if (v.position.y == doctest::Approx(-0.5f))
        {
            float r = std::sqrt(v.position.x * v.position.x + v.position.z * v.position.z);
            // Skip cap center (r ≈ 0)
            if (r > 0.01f)
            {
                CHECK(r == doctest::Approx(0.5f));
                baseCount++;
            }
        }
    }
    CHECK(baseCount > 0);
}

TEST_CASE("cone: zero radius returns empty")
{
    ConeGeometry geom;
    geom.radius = 0.0f;
    auto mesh = generate_cone_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("cone: zero height returns empty")
{
    ConeGeometry geom;
    geom.height = 0.0f;
    auto mesh = generate_cone_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("cone: bounds cover expected volume")
{
    ConeGeometry geom; // r=0.5, h=1.0
    auto mesh = generate_cone_mesh(geom);

    CHECK(mesh.bounds.min.x == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.x == doctest::Approx(0.5f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-0.5f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.5f));
}

// ── Disk ───────────────────────────────────────────────────────────────────

TEST_CASE("disk: filled disk (innerRadius=0)")
{
    DiskGeometry geom; // outerR=1.0, innerR=0.0, segs=32
    auto mesh = generate_disk_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // Triangle fan: 1 center + 33 perimeter = 34 vertices
    CHECK(mesh.vertices.size() >= 3);
    CHECK(!mesh.indices.empty());

    // All vertices at Y=0
    for (const auto& v : mesh.vertices)
        CHECK(v.position.y == doctest::Approx(0.0f));

    // All normals +Y
    for (const auto& v : mesh.vertices)
        CHECK(v.normal.y == doctest::Approx(1.0f));
}

TEST_CASE("disk: annulus (innerRadius>0)")
{
    DiskGeometry geom;
    geom.innerRadius = 0.3f;
    auto mesh = generate_disk_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    // Should have inner and outer rings
    CHECK(mesh.vertices.size() > 3);
    CHECK(!mesh.indices.empty());
}

TEST_CASE("disk: inner >= outer handled")
{
    DiskGeometry geom;
    geom.innerRadius = 1.5f;
    geom.outerRadius = 1.0f;
    auto mesh = generate_disk_mesh(geom);

    // innerR >= outerR is clamped to 0, producing a filled disk
    CHECK(!mesh.vertices.empty());
}

TEST_CASE("disk: zero outer radius returns empty")
{
    DiskGeometry geom;
    geom.outerRadius = 0.0f;
    auto mesh = generate_disk_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("disk: bounds correct")
{
    DiskGeometry geom; // outerR=1.0
    auto mesh = generate_disk_mesh(geom);

    CHECK(mesh.bounds.min.x == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.z == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.z == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(0.0f));
    CHECK(mesh.bounds.max.y == doctest::Approx(0.0f));
}

// ── Billboard ──────────────────────────────────────────────────────────────

TEST_CASE("billboard: default quad")
{
    BillboardGeometry geom; // size=(1,1,0)
    auto mesh = generate_billboard_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);

    // All on XY plane
    for (const auto& v : mesh.vertices)
        CHECK(v.position.z == doctest::Approx(0.0f));

    // All normals +Z
    for (const auto& v : mesh.vertices)
        CHECK(v.normal.z == doctest::Approx(1.0f));
}

TEST_CASE("billboard: custom size")
{
    BillboardGeometry geom;
    geom.size = {2.0f, 3.0f, 0.0f};
    auto mesh = generate_billboard_mesh(geom);

    CHECK(mesh.bounds.min.x == doctest::Approx(-1.0f));
    CHECK(mesh.bounds.max.x == doctest::Approx(1.0f));
    CHECK(mesh.bounds.min.y == doctest::Approx(-1.5f));
    CHECK(mesh.bounds.max.y == doctest::Approx(1.5f));
}

TEST_CASE("billboard: zero area returns empty")
{
    BillboardGeometry geom;
    geom.size = {0.0f, 0.0f, 0.0f};
    auto mesh = generate_billboard_mesh(geom);
    CHECK(mesh.vertices.empty());
}

// ── Tube ───────────────────────────────────────────────────────────────────

TEST_CASE("tube: simple straight tube")
{
    TubeGeometry geom;
    geom.path = {{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}}; // 2 points, straight up
    geom.radius = 0.2f;
    geom.radialSegments = 16;
    geom.capped = true;
    auto mesh = generate_tube_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("tube: path with multiple points")
{
    TubeGeometry geom;
    geom.path = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    geom.radius = 0.1f;
    auto mesh = generate_tube_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(mesh.vertices.size() > 0);
    CHECK(mesh.indices.size() > 0);
}

TEST_CASE("tube: fewer than 2 path points returns empty")
{
    TubeGeometry geom;
    geom.path = {{0, 0, 0}};
    auto mesh = generate_tube_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("tube: zero radius returns empty")
{
    TubeGeometry geom;
    geom.path = {{0, 0, 0}, {1, 0, 0}};
    geom.radius = 0.0f;
    auto mesh = generate_tube_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("tube: vertex count is correct")
{
    TubeGeometry geom;
    geom.path = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}}; // 3 points
    geom.radialSegments = 8;
    geom.capped = false;
    auto mesh = generate_tube_mesh(geom);

    // 3 points * (8+1) ring vertices = 27 vertices (no caps)
    CHECK(mesh.vertices.size() == 3 * (8 + 1));
}

// ── Arrow3D ────────────────────────────────────────────────────────────────

TEST_CASE("arrow3d: default arrow")
{
    Arrow3DGeometry geom; // start=(0,0,0), end=(0,1,0), headR=0.15, headL=0.3, shaftR=0.05, slices=16
    auto mesh = generate_arrow3d_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("arrow3d: zero-length returns empty")
{
    Arrow3DGeometry geom;
    geom.end = geom.start;
    auto mesh = generate_arrow3d_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("arrow3d: head longer than total length")
{
    Arrow3DGeometry geom;
    geom.headLength = 2.0f; // longer than default length of 1.0
    auto mesh = generate_arrow3d_mesh(geom);

    // headLength is clamped to totalLen, so should still produce a cone
    CHECK(!mesh.vertices.empty());
}

TEST_CASE("arrow3d: direction is preserved")
{
    Arrow3DGeometry geom;
    geom.start = {0.0f, 0.0f, 0.0f};
    geom.end   = {2.0f, 0.0f, 0.0f}; // pointing along X
    auto mesh  = generate_arrow3d_mesh(geom);

    // All vertices should be near the X axis (Y and Z within head radius bounds)
    for (const auto& v : mesh.vertices)
    {
        CHECK(std::abs(v.position.y) <= doctest::Approx(0.15f).epsilon(0.01f));
        CHECK(std::abs(v.position.z) <= doctest::Approx(0.15f).epsilon(0.01f));
    }
}

// ── Axes ───────────────────────────────────────────────────────────────────

TEST_CASE("axes: default axes")
{
    AxesGeometry geom; // length=1.0, shaftR=0.02, headR=0.06, headL=0.15
    auto mesh = generate_axes_mesh(geom);

    CHECK(mesh.topology == PrimitiveTopology::Triangles);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());
}

TEST_CASE("axes: three colored axes produce mesh")
{
    AxesGeometry geom;
    auto mesh = generate_axes_mesh(geom);

    // Should have vertices for all three axes (X, Y, Z)
    // Per axis: 10 slices → 11 verts/ring, shaft=2*11=22, head=11+1=12, total=34
    // 3 axes = 102 vertices
    CHECK(mesh.vertices.size() > 20);
    CHECK(mesh.indices.size() > 30);
}

TEST_CASE("axes: zero length returns empty")
{
    AxesGeometry geom;
    geom.length = 0.0f;
    auto mesh   = generate_axes_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("axes: vertices have per-axis colors")
{
    AxesGeometry geom;
    auto mesh   = generate_axes_mesh(geom);

    // Color stored as Quat: {R, G, B, A}
    // X axis: {1, 0, 0, 1}, Y axis: {0, 1, 0, 1}, Z axis: {0, 0, 1, 1}
    bool hasRed   = false;
    bool hasGreen = false;
    bool hasBlue  = false;

    for (const auto& v : mesh.vertices)
    {
        // Check for red (X axis): color.w ≈ 1.0 (R channel)
        if (v.color.w == doctest::Approx(1.0f).epsilon(0.01f) &&
            v.color.x == doctest::Approx(0.0f).epsilon(0.01f))
            hasRed = true;

        // Check for green (Y axis): color.x ≈ 1.0 (G channel)
        if (v.color.x == doctest::Approx(1.0f).epsilon(0.01f) &&
            v.color.w == doctest::Approx(0.0f).epsilon(0.01f))
            hasGreen = true;

        // Check for blue (Z axis): color.y ≈ 1.0 (B channel)
        if (v.color.y == doctest::Approx(1.0f).epsilon(0.01f) &&
            v.color.w == doctest::Approx(0.0f).epsilon(0.01f))
            hasBlue = true;
    }

    CHECK(hasRed);
    CHECK(hasGreen);
    CHECK(hasBlue);
}

// ── Sphere: normals / texcoords flags ───────────────────────────────────────

TEST_CASE("sphere: generateNormals flag controls normal output")
{
    SphereGeometry geom;
    geom.generateNormals = false;
    geom.generateTexcoords = true;
    auto mesh = generate_sphere_mesh(geom);

    CHECK(!mesh.vertices.empty());

    // With generateNormals = false, normals should be default (0, 1, 0)
    for (const auto& v : mesh.vertices)
    {
        CHECK(v.normal.x == doctest::Approx(0.0f));
        CHECK(v.normal.y == doctest::Approx(1.0f));
        CHECK(v.normal.z == doctest::Approx(0.0f));
    }
}

TEST_CASE("sphere: generateTexcoords flag controls UV output")
{
    SphereGeometry geom;
    geom.generateNormals = true;
    geom.generateTexcoords = false;
    auto mesh = generate_sphere_mesh(geom);

    CHECK(!mesh.vertices.empty());

    // With generateTexcoords = false, UVs should be default (0, 0, 0)
    for (const auto& v : mesh.vertices)
    {
        CHECK(v.uv.x == doctest::Approx(0.0f));
        CHECK(v.uv.y == doctest::Approx(0.0f));
        CHECK(v.uv.z == doctest::Approx(0.0f));
    }
}

TEST_CASE("sphere: both flags false still produces mesh")
{
    SphereGeometry geom;
    geom.generateNormals = false;
    geom.generateTexcoords = false;
    auto mesh = generate_sphere_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(mesh.vertices.size() == 561); // same vertex count
}

// ── Cone: uncapped ──────────────────────────────────────────────────────────

TEST_CASE("cone: uncapped cone has no cap vertices")
{
    ConeGeometry geom;
    geom.capped = false;
    auto mesh = generate_cone_mesh(geom);

    CHECK(!mesh.vertices.empty());

    // Uncapped: only side vertices (slices+1 ring + 1 tip) = 34
    // (slices clamped to 3 min, but default is 32)
    CHECK(mesh.vertices.size() == 34);  // 33 ring + 1 tip
    CHECK(mesh.indices.size() == 96);   // 32 * 3 (no cap indices)

    // No vertex should have a -Y normal (that would be cap center)
    for (const auto& v : mesh.vertices)
    {
        CHECK((v.normal.y != doctest::Approx(-1.0f)));
    }
}

TEST_CASE("cone: non-default slices")
{
    ConeGeometry geom;
    geom.slices = 8;
    auto mesh = generate_cone_mesh(geom);

    // 8+1 ring = 9, +1 tip, +1 cap center = 11 vertices (capped)
    CHECK(mesh.vertices.size() == 11);
    // side: 8*3 = 24, cap: 8*3 = 24, total = 48
    CHECK(mesh.indices.size() == 48);
}

// ── Tube: degenerate paths and uncapped ─────────────────────────────────────

TEST_CASE("tube: coincident points in path handled")
{
    TubeGeometry geom;
    // Two consecutive identical points should not crash
    geom.path = {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}};
    geom.radius = 0.1f;
    auto mesh = generate_tube_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("tube: all coincident points returns empty")
{
    TubeGeometry geom;
    geom.path = {{1, 2, 3}, {1, 2, 3}, {1, 2, 3}};
    geom.radius = 0.1f;
    auto mesh = generate_tube_mesh(geom);

    // All points coincide so no valid path segment can be constructed
    CHECK(mesh.vertices.empty());
}

TEST_CASE("tube: empty path returns empty")
{
    TubeGeometry geom;
    geom.path = {};
    auto mesh = generate_tube_mesh(geom);
    CHECK(mesh.vertices.empty());
}

TEST_CASE("tube: uncapped tube has fewer vertices")
{
    TubeGeometry geom;
    geom.path = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
    geom.radialSegments = 8;
    geom.capped = false;
    auto mesh = generate_tube_mesh(geom);

    // 3 points * (8+1) ring vertices = 27 (no cap centers)
    CHECK(mesh.vertices.size() == 3 * 9);
}

// ── Arrow3D: non-aligned direction and shaft-only ───────────────────────────

TEST_CASE("arrow3d: non-axis-aligned direction preserves orientation")
{
    Arrow3DGeometry geom;
    geom.start = {0.0f, 0.0f, 0.0f};
    geom.end   = {1.0f, 1.0f, 1.0f}; // diagonal
    auto mesh  = generate_arrow3d_mesh(geom);

    CHECK(!mesh.vertices.empty());

    // Tip vertex should be near (1, 1, 1)
    // The tip is the last vertex added in the head section
    bool foundTip = false;
    for (const auto& v : mesh.vertices)
    {
        if (v.position.x == doctest::Approx(1.0f) &&
            v.position.y == doctest::Approx(1.0f) &&
            v.position.z == doctest::Approx(1.0f))
        {
            foundTip = true;
            break;
        }
    }
    CHECK(foundTip);
}

TEST_CASE("arrow3d: shaft-only (zero head radius) produces cylinder")
{
    Arrow3DGeometry geom;
    geom.headRadius = 0.0f;  // no head cone
    geom.shaftRadius = 0.1f;
    auto mesh = generate_arrow3d_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());

    // Should only have shaft vertices (2 rings) — no head
    // 2 * (16+1) = 34 vertices for shaft
    CHECK(mesh.vertices.size() == 34);
    // 16 * 6 = 96 indices for shaft
    CHECK(mesh.indices.size() == 96);
}

TEST_CASE("arrow3d: head-only (zero shaft radius) produces cone")
{
    Arrow3DGeometry geom;
    geom.shaftRadius = 0.0f;  // no shaft
    geom.headRadius = 0.1f;
    geom.headLength = 1.0f;   // head spans full length
    auto mesh = generate_arrow3d_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());

    // Head only: 1 ring + 1 tip = (16+1) + 1 = 18 vertices
    CHECK(mesh.vertices.size() == 18);
}

// ── Axes: shaft-only ────────────────────────────────────────────────────────

TEST_CASE("axes: shaft-only (zero head radius)")
{
    AxesGeometry geom;
    geom.headRadius = 0.0f;  // no heads
    geom.length = 1.0f;
    geom.shaftRadius = 0.02f;
    auto mesh = generate_axes_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());

    // Per axis: only shaft = 2*(10+1) = 22 vertices, 10*6 = 60 indices
    // 3 axes = 66 vertices, 180 indices
    CHECK(mesh.vertices.size() == 66);
    CHECK(mesh.indices.size() == 180);
}

TEST_CASE("axes: head-only (zero shaft radius)")
{
    AxesGeometry geom;
    geom.shaftRadius = 0.0f;  // no shaft
    geom.headRadius = 0.06f;
    geom.headLength = 1.0f;   // head spans full length
    geom.length = 1.0f;
    auto mesh = generate_axes_mesh(geom);

    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.indices.empty());

    // Per axis: only head = (10+1)+1 = 12 vertices, 10*3 = 30 indices
    // 3 axes = 36 vertices, 90 indices
    CHECK(mesh.vertices.size() == 36);
    CHECK(mesh.indices.size() == 90);
}

// ── Billboard: UV coordinates ───────────────────────────────────────────────

TEST_CASE("billboard: UV coordinates are correct")
{
    BillboardGeometry geom;
    auto mesh = generate_billboard_mesh(geom);

    // Bottom-left: UV (0, 0)
    CHECK(mesh.vertices[0].uv.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].uv.y == doctest::Approx(0.0f));

    // Bottom-right: UV (1, 0)
    CHECK(mesh.vertices[1].uv.x == doctest::Approx(1.0f));
    CHECK(mesh.vertices[1].uv.y == doctest::Approx(0.0f));

    // Top-right: UV (1, 1)
    CHECK(mesh.vertices[2].uv.x == doctest::Approx(1.0f));
    CHECK(mesh.vertices[2].uv.y == doctest::Approx(1.0f));

    // Top-left: UV (0, 1)
    CHECK(mesh.vertices[3].uv.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[3].uv.y == doctest::Approx(1.0f));
}

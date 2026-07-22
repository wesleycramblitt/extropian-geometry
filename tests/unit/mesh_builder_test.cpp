#include <doctest/doctest.h>
#include <exd/geometry/geometry.hpp>

using namespace exd::geometry;

// ── MeshBuilder ────────────────────────────────────────────────────────────

TEST_CASE("MeshBuilder: empty build returns empty mesh")
{
    MeshBuilder builder;
    auto mesh = builder.build();

    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
    CHECK(mesh.topology == PrimitiveTopology::Triangles);
}

TEST_CASE("MeshBuilder: single triangle")
{
    MeshBuilder builder;
    Vertex v;
    v.position = {0.0f, 0.0f, 0.0f};
    auto a = builder.add_vertex(v);
    v.position = {1.0f, 0.0f, 0.0f};
    auto b = builder.add_vertex(v);
    v.position = {0.0f, 1.0f, 0.0f};
    auto c = builder.add_vertex(v);
    builder.add_triangle(a, b, c);

    auto mesh = builder.build();

    CHECK(mesh.vertices.size() == 3);
    CHECK(mesh.indices.size() == 3);
    CHECK(mesh.indices[0] == 0);
    CHECK(mesh.indices[1] == 1);
    CHECK(mesh.indices[2] == 2);
}

TEST_CASE("MeshBuilder: quad produces 2 triangles")
{
    MeshBuilder builder;
    Vertex v;
    auto a = builder.add_vertex(v);
    auto b = builder.add_vertex(v);
    auto c = builder.add_vertex(v);
    auto d = builder.add_vertex(v);
    builder.add_quad(a, b, c, d);

    auto mesh = builder.build();

    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6); // 2 triangles
}

TEST_CASE("MeshBuilder: reserve prevents reallocation")
{
    MeshBuilder builder;
    builder.reserve(1000, 3000);
    auto mesh = builder.build();

    // Should not crash even though we didn't add vertices
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("MeshBuilder: clear resets state")
{
    MeshBuilder builder;
    Vertex v;
    builder.add_vertex(v);
    builder.clear();

    auto mesh = builder.build();
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE("MeshBuilder: build consumes data (move semantics)")
{
    MeshBuilder builder;
    Vertex v;
    builder.add_vertex(v);

    auto mesh1 = builder.build();
    CHECK(mesh1.vertices.size() == 1);

    // Builder should be empty after build
    auto mesh2 = builder.build();
    CHECK(mesh2.vertices.empty());
    CHECK(mesh2.indices.empty());
}

TEST_CASE("MeshBuilder: custom topology")
{
    MeshBuilder builder;
    Vertex v;
    v.position = {0.0f, 0.0f, 0.0f};
    builder.add_vertex(v);
    v.position = {1.0f, 0.0f, 0.0f};
    builder.add_vertex(v);
    v.position = {0.0f, 1.0f, 0.0f};
    builder.add_vertex(v);

    auto mesh = builder.build(PrimitiveTopology::Points);

    CHECK(mesh.topology == PrimitiveTopology::Points);
    CHECK(mesh.vertices.size() == 3);
}

TEST_CASE("MeshBuilder: triangle winding is preserved")
{
    MeshBuilder builder;
    Vertex v;
    v.position = {0.0f, 0.0f, 0.0f};
    auto a = builder.add_vertex(v);
    v.position = {1.0f, 0.0f, 0.0f};
    auto b = builder.add_vertex(v);
    v.position = {0.0f, 1.0f, 0.0f};
    auto c = builder.add_vertex(v);
    builder.add_triangle(a, b, c);

    // Also test quad winding
    v.position = {1.0f, 1.0f, 0.0f};
    auto d = builder.add_vertex(v);
    builder.add_quad(a, b, c, d); // a-b-c, a-c-d

    auto mesh = builder.build();

    // 3 indices for triangle + 6 for quad = 9 total
    CHECK(mesh.indices.size() == 9);
    CHECK(mesh.indices[0] == 0);
    CHECK(mesh.indices[1] == 1);
    CHECK(mesh.indices[2] == 2);
    // Quad: a-b-c, a-c-d
    CHECK(mesh.indices[3] == 0);
    CHECK(mesh.indices[4] == 1);
    CHECK(mesh.indices[5] == 2);
    CHECK(mesh.indices[6] == 0);
    CHECK(mesh.indices[7] == 2);
    CHECK(mesh.indices[8] == 3);
}

TEST_CASE("MeshBuilder: vertex indices are sequential")
{
    MeshBuilder builder;
    Vertex v;
    for (int i = 0; i < 10; ++i)
    {
        v.position = {static_cast<float>(i), 0.0f, 0.0f};
        uint32_t idx = builder.add_vertex(v);
        CHECK(idx == static_cast<uint32_t>(i));
    }
}

TEST_CASE("MeshBuilder: multiple triangles share vertices correctly")
{
    MeshBuilder builder;
    Vertex v;

    // Two triangles sharing an edge: (0,1,2) and (1,3,2)
    v.position = {0.0f, 0.0f, 0.0f};
    auto v0 = builder.add_vertex(v);
    v.position = {1.0f, 0.0f, 0.0f};
    auto v1 = builder.add_vertex(v);
    v.position = {1.0f, 1.0f, 0.0f};
    auto v2 = builder.add_vertex(v);
    v.position = {0.0f, 1.0f, 0.0f};
    auto v3 = builder.add_vertex(v);

    builder.add_triangle(v0, v1, v2);
    builder.add_triangle(v1, v3, v2);

    auto mesh = builder.build();

    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);
    // First triangle
    CHECK(mesh.indices[0] == 0);
    CHECK(mesh.indices[1] == 1);
    CHECK(mesh.indices[2] == 2);
    // Second triangle
    CHECK(mesh.indices[3] == 1);
    CHECK(mesh.indices[4] == 3);
    CHECK(mesh.indices[5] == 2);
}

TEST_CASE("MeshBuilder: clear allows reuse")
{
    MeshBuilder builder;
    Vertex v;

    v.position = {0.0f, 0.0f, 0.0f};
    builder.add_vertex(v);
    auto mesh1 = builder.build();
    CHECK(mesh1.vertices.size() == 1);

    // Reuse builder
    v.position = {1.0f, 0.0f, 0.0f};
    builder.add_vertex(v);
    auto mesh2 = builder.build();
    CHECK(mesh2.vertices.size() == 1);
    CHECK(mesh2.vertices[0].position.x == doctest::Approx(1.0f));
}

TEST_CASE("MeshBuilder: default vertex values")
{
    MeshBuilder builder;
    Vertex v; // default-initialized
    auto idx = builder.add_vertex(v);
    auto mesh = builder.build();

    CHECK(idx == 0);
    CHECK(mesh.vertices[0].position.x == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].position.y == doctest::Approx(0.0f));
    CHECK(mesh.vertices[0].position.z == doctest::Approx(0.0f));
    // Default normal is +Y
    CHECK(mesh.vertices[0].normal.y == doctest::Approx(1.0f));
}

TEST_CASE("MeshBuilder: degenerate triangle still works")
{
    MeshBuilder builder;
    Vertex v;
    auto a = builder.add_vertex(v);
    // Same index used for all three vertices — degenerate
    builder.add_triangle(a, a, a);

    auto mesh = builder.build();
    CHECK(mesh.vertices.size() == 1);
    CHECK(mesh.indices.size() == 3);
    CHECK(mesh.indices[0] == 0);
    CHECK(mesh.indices[1] == 0);
    CHECK(mesh.indices[2] == 0);
}

TEST_CASE("MeshBuilder: build with LineStrip topology")
{
    MeshBuilder builder;
    Vertex v;
    v.position = {0.0f, 0.0f, 0.0f};
    builder.add_vertex(v);
    v.position = {1.0f, 0.0f, 0.0f};
    builder.add_vertex(v);
    v.position = {1.0f, 1.0f, 0.0f};
    builder.add_vertex(v);

    auto mesh = builder.build(PrimitiveTopology::LineStrip);
    CHECK(mesh.topology == PrimitiveTopology::LineStrip);
    CHECK(mesh.vertices.size() == 3);
}

TEST_CASE("MeshBuilder: build with Lines topology")
{
    MeshBuilder builder;
    Vertex v;
    v.position = {0.0f, 0.0f, 0.0f};
    builder.add_vertex(v);
    v.position = {1.0f, 0.0f, 0.0f};
    builder.add_vertex(v);

    auto mesh = builder.build(PrimitiveTopology::Lines);
    CHECK(mesh.topology == PrimitiveTopology::Lines);
    CHECK(mesh.vertices.size() == 2);
}

TEST_CASE("MeshBuilder: build with TriangleStrip topology")
{
    MeshBuilder builder;
    Vertex v;
    for (int i = 0; i < 4; ++i)
    {
        v.position = {static_cast<float>(i), 0.0f, 0.0f};
        builder.add_vertex(v);
    }

    auto mesh = builder.build(PrimitiveTopology::TriangleStrip);
    CHECK(mesh.topology == PrimitiveTopology::TriangleStrip);
    CHECK(mesh.vertices.size() == 4);
}

TEST_CASE("MeshBuilder: multiple builds from same builder without clear")
{
    MeshBuilder builder;
    Vertex v;
    builder.add_vertex(v);

    auto mesh1 = builder.build();
    CHECK(mesh1.vertices.size() == 1);

    // After build (which consumes), builder should be empty
    auto mesh2 = builder.build();
    CHECK(mesh2.vertices.empty());

    // Add new vertices after build and build again
    builder.add_vertex(v);
    builder.add_vertex(v);
    auto mesh3 = builder.build();
    CHECK(mesh3.vertices.size() == 2);
}

TEST_CASE("MeshBuilder: add_triangle with high indices does not crash")
{
    MeshBuilder builder;
    Vertex v;
    auto a = builder.add_vertex(v);
    auto b = builder.add_vertex(v);
    auto c = builder.add_vertex(v);

    // Indices beyond vertex count should still be accepted
    // (MeshBuilder doesn't validate indices at add time)
    builder.add_triangle(a + 100, b + 200, c + 300);

    auto mesh = builder.build();
    CHECK(mesh.vertices.size() == 3);
    CHECK(mesh.indices.size() == 3);
}

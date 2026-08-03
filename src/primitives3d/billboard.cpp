#include <exd/geometry/primitives3d.hpp>

namespace exd::geometry
{

MeshData generate_billboard_mesh(const BillboardGeometry& geometry)
{
    if (geometry.size.x <= 0.0f || geometry.size.y <= 0.0f)
        return {};

    float hw = geometry.size.x * 0.5f;
    float hh = geometry.size.y * 0.5f;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(4);
    indices.reserve(6);

    math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

    // Quad in XY plane, centered at origin
    // CCW winding when viewed from +Z
    // Bottom-left
    {
        Vertex v;
        v.position = {-hw, -hh, 0.0f};
        v.normal   = {0.0f, 0.0f, 1.0f};
        v.uv       = {0.0f, 0.0f, 0.0f};
        v.tangent  = tangent_id;
        v.color    = geometry.color;
        vertices.push_back(v);
    }
    // Bottom-right
    {
        Vertex v;
        v.position = {hw, -hh, 0.0f};
        v.normal   = {0.0f, 0.0f, 1.0f};
        v.uv       = {1.0f, 0.0f, 0.0f};
        v.tangent  = tangent_id;
        v.color    = geometry.color;
        vertices.push_back(v);
    }
    // Top-right
    {
        Vertex v;
        v.position = {hw, hh, 0.0f};
        v.normal   = {0.0f, 0.0f, 1.0f};
        v.uv       = {1.0f, 1.0f, 0.0f};
        v.tangent  = tangent_id;
        v.color    = geometry.color;
        vertices.push_back(v);
    }
    // Top-left
    {
        Vertex v;
        v.position = {-hw, hh, 0.0f};
        v.normal   = {0.0f, 0.0f, 1.0f};
        v.uv       = {0.0f, 1.0f, 0.0f};
        v.tangent  = tangent_id;
        v.color    = geometry.color;
        vertices.push_back(v);
    }

    // Two triangles forming the quad (CCW from +Z)
    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);

    indices.push_back(0);
    indices.push_back(2);
    indices.push_back(3);

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {{-hw, -hh, 0.0f}, {hw, hh, 0.0f}};

    return mesh;
}

} // namespace exd::geometry

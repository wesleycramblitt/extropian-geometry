#include <exd/geometry/mesh_builder.hpp>

namespace exd::geometry
{

void MeshBuilder::reserve(size_t verts, size_t idx)
{
    vertices_.reserve(verts);
    indices_.reserve(idx);
}

uint32_t MeshBuilder::add_vertex(const Vertex& v)
{
    uint32_t idx = static_cast<uint32_t>(vertices_.size());
    vertices_.push_back(v);
    return idx;
}

void MeshBuilder::add_triangle(uint32_t a, uint32_t b, uint32_t c)
{
    indices_.push_back(a);
    indices_.push_back(b);
    indices_.push_back(c);
}

void MeshBuilder::add_quad(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    indices_.push_back(a);
    indices_.push_back(b);
    indices_.push_back(c);
    indices_.push_back(a);
    indices_.push_back(c);
    indices_.push_back(d);
}

MeshData MeshBuilder::build(PrimitiveTopology topology)
{
    MeshData mesh;
    mesh.vertices = std::move(vertices_);
    mesh.indices  = std::move(indices_);
    mesh.topology = topology;
    clear();
    return mesh;
}

void MeshBuilder::clear()
{
    vertices_.clear();
    indices_.clear();
}

} // namespace exd::geometry

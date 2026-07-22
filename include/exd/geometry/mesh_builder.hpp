#pragma once

#include <exd/geometry/types.hpp>

#include <cstdint>
#include <vector>

namespace exd::geometry
{

/// Incremental mesh builder with auto-index generation.
///
/// Construct procedural meshes without manual index bookkeeping.
/// Call add_vertex() for each corner, then add_triangle() / add_quad()
/// to stitch them together.
class MeshBuilder
{
public:
    MeshBuilder() = default;

    void reserve(size_t verts, size_t idx);

    uint32_t add_vertex(const Vertex& v);
    void add_triangle(uint32_t a, uint32_t b, uint32_t c);
    void add_quad(uint32_t a, uint32_t b, uint32_t c, uint32_t d);

    MeshData build(PrimitiveTopology topology = PrimitiveTopology::Triangles);
    void clear();

private:
    std::vector<Vertex>   vertices_;
    std::vector<uint32_t> indices_;
};

} // namespace exd::geometry

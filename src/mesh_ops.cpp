#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <limits>

namespace exd::geometry
{

MeshData merge_meshes(std::span<const MeshData> meshes)
{
    if (meshes.empty())
    {
        return {};
    }

    // Validate topology consistency
    const auto topology = meshes[0].topology;
    for (size_t i = 1; i < meshes.size(); ++i)
    {
        if (meshes[i].topology != topology)
        {
            // Mismatched topology — fall back to first mesh's topology
            // but still merge the geometry data
        }
    }

    // Count totals
    size_t totalVertices = 0;
    size_t totalIndices  = 0;
    for (const auto& m : meshes)
    {
        totalVertices += m.vertices.size();
        totalIndices  += m.indices.size();
    }

    MeshData result;
    result.topology = topology;
    result.vertices.reserve(totalVertices);
    result.indices.reserve(totalIndices);

    uint32_t baseVertex = 0;
    for (const auto& m : meshes)
    {
        // Append vertices
        result.vertices.insert(result.vertices.end(),
                               m.vertices.begin(), m.vertices.end());

        // Append indices with offset
        for (auto idx : m.indices)
        {
            result.indices.push_back(baseVertex + idx);
        }

        baseVertex += static_cast<uint32_t>(m.vertices.size());
    }

    result.bounds = compute_bounds(result.vertices);
    return result;
}

// Helper: transform a Vec3f point by Mat4 (column-major, w=1 for point)
static math::Vec3f transform_point(const math::Vec3f& v, const math::Mat4& m)
{
    return {
        m.m[0] * v.x + m.m[4] * v.y + m.m[8]  * v.z + m.m[12],
        m.m[1] * v.x + m.m[5] * v.y + m.m[9]  * v.z + m.m[13],
        m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z + m.m[14]
    };
}

// Helper: transform a Vec3f direction by the upper 3x3 (inverse-transpose
// for normals; for rigid transforms this is just the rotation part)
static math::Vec3f transform_direction(const math::Vec3f& v, const math::Mat4& m)
{
    return {
        m.m[0] * v.x + m.m[1] * v.y + m.m[2]  * v.z,
        m.m[4] * v.x + m.m[5] * v.y + m.m[6]  * v.z,
        m.m[8] * v.x + m.m[9] * v.y + m.m[10] * v.z
    };
}

MeshData transform_mesh(const MeshData& mesh,
                        const math::Mat4& transform,
                        bool transformNormals)
{
    MeshData result;
    result.topology = mesh.topology;
    result.vertices.reserve(mesh.vertices.size());
    result.indices = mesh.indices; // indices unchanged

    for (const auto& v : mesh.vertices)
    {
        Vertex tv = v;
        tv.position = transform_point(v.position, transform);
        if (transformNormals)
        {
            tv.normal = transform_direction(v.normal, transform);
        }
        result.vertices.push_back(tv);
    }

    result.bounds = compute_bounds(result.vertices);
    return result;
}

Bounds compute_bounds(std::span<const Vertex> vertices)
{
    if (vertices.empty())
    {
        return {};
    }

    Bounds b;
    b.min = vertices[0].position;
    b.max = vertices[0].position;

    for (const auto& v : vertices)
    {
        b.min.x = std::min(b.min.x, v.position.x);
        b.min.y = std::min(b.min.y, v.position.y);
        b.min.z = std::min(b.min.z, v.position.z);
        b.max.x = std::max(b.max.x, v.position.x);
        b.max.y = std::max(b.max.y, v.position.y);
        b.max.z = std::max(b.max.z, v.position.z);
    }

    return b;
}

} // namespace exd::geometry

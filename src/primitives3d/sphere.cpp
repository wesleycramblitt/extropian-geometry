#include <exd/geometry/primitives3d.hpp>

#include <cmath>

namespace exd::geometry
{

MeshData generate_sphere_mesh(const SphereGeometry& geom)
{
    constexpr float pi = 3.14159265358979323846f;

    int stacks = static_cast<int>(geom.latitudeSegments);
    int slices = static_cast<int>(geom.longitudeSegments);
    float radius = geom.radius;

    math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

    size_t vert_count = static_cast<size_t>(stacks + 1) * (slices + 1);
    size_t idx_count  = static_cast<size_t>(stacks) * slices * 6;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(vert_count);
    indices.reserve(idx_count);

    for (int i = 0; i <= stacks; ++i)
    {
        float theta = static_cast<float>(i) * pi / static_cast<float>(stacks);
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (int j = 0; j <= slices; ++j)
        {
            float phi = static_cast<float>(j) * 2.0f * pi / static_cast<float>(slices);
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            float x = radius * sin_theta * cos_phi;
            float y = radius * cos_theta;
            float z = radius * sin_theta * sin_phi;

            Vertex v;
            v.position = math::Vec3f{x, y, z};

            if (geom.generateNormals)
                v.normal = math::Vec3f{sin_theta * cos_phi, cos_theta, sin_theta * sin_phi};

            if (geom.generateTexcoords)
                v.uv = math::Vec3f{
                    static_cast<float>(j) / static_cast<float>(slices),
                    static_cast<float>(i) / static_cast<float>(stacks),
                    0.0f};

            v.tangent = tangent_id;

            vertices.push_back(v);
        }
    }

    for (int i = 0; i < stacks; ++i)
    {
        for (int j = 0; j < slices; ++j)
        {
            uint32_t first  = static_cast<uint32_t>(i * (slices + 1) + j);
            uint32_t second = first + static_cast<uint32_t>(slices + 1);

            indices.push_back(first);
            indices.push_back(second);
            indices.push_back(first + 1);

            indices.push_back(second);
            indices.push_back(second + 1);
            indices.push_back(first + 1);
        }
    }

    return MeshData{std::move(vertices), std::move(indices), PrimitiveTopology::Triangles};
}

} // namespace exd::geometry

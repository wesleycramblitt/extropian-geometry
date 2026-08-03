#include <exd/geometry/primitives3d.hpp>

#include <cmath>

namespace exd::geometry
{

MeshData generate_ellipsoid_mesh(const EllipsoidGeometry& geom)
{
    constexpr float pi = 3.14159265358979323846f;

    if (geom.radii.x <= 0.0f || geom.radii.y <= 0.0f || geom.radii.z <= 0.0f)
        return {};

    int stacks = static_cast<int>(geom.latitudeSegments);
    int slices = static_cast<int>(geom.longitudeSegments);

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

            // Unit sphere direction, then scale by radii
            float ux = sin_theta * cos_phi;
            float uy = cos_theta;
            float uz = sin_theta * sin_phi;

            Vertex v;
            v.position = math::Vec3f{
                geom.radii.x * ux,
                geom.radii.y * uy,
                geom.radii.z * uz
            };

            // Normal: for an ellipsoid x²/a² + y²/b² + z²/c² = 1,
            // the unnormalized normal at (x,y,z) is (x/a², y/b², z/c²)
            float nx = ux / geom.radii.x;
            float ny = uy / geom.radii.y;
            float nz = uz / geom.radii.z;
            float nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
            v.normal = (nlen > 1e-8f)
                ? math::Vec3f{nx / nlen, ny / nlen, nz / nlen}
                : math::Vec3f{0.0f, 1.0f, 0.0f};

            v.uv = math::Vec3f{
                static_cast<float>(j) / static_cast<float>(slices),
                static_cast<float>(i) / static_cast<float>(stacks),
                0.0f
            };
            v.tangent = tangent_id;
            v.color   = geom.color;

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

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {
        {-geom.radii.x, -geom.radii.y, -geom.radii.z},
        { geom.radii.x,  geom.radii.y,  geom.radii.z}
    };
    return mesh;
}

} // namespace exd::geometry

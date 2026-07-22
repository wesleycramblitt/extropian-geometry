#include <exd/geometry/primitives3d.hpp>

#include <cmath>

namespace exd::geometry
{

MeshData generate_torus_mesh(const TorusGeometry& geometry)
{
    constexpr float pi = 3.14159265358979323846f;

    if (geometry.majorRadius <= 0.0f || geometry.minorRadius <= 0.0f)
        return {};

    uint32_t majorSegs = geometry.majorSegments < 3 ? 3 : geometry.majorSegments;
    uint32_t minorSegs = geometry.minorSegments < 3 ? 3 : geometry.minorSegments;

    float majorR = geometry.majorRadius;
    float minorR = geometry.minorRadius;

    size_t vertCount = static_cast<size_t>(majorSegs + 1) * (minorSegs + 1);
    size_t idxCount  = static_cast<size_t>(majorSegs) * minorSegs * 6;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(vertCount);
    indices.reserve(idxCount);

    math::Quat tangent_id{1.0f, 0.0f, 0.0f, 1.0f};

    // Torus in XZ plane: major angle theta around Y, minor angle phi around tube
    for (uint32_t i = 0; i <= majorSegs; ++i)
    {
        float theta = static_cast<float>(i) * 2.0f * pi / static_cast<float>(majorSegs);
        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);

        for (uint32_t j = 0; j <= minorSegs; ++j)
        {
            float phi = static_cast<float>(j) * 2.0f * pi / static_cast<float>(minorSegs);
            float cos_phi = std::cos(phi);
            float sin_phi = std::sin(phi);

            float x = (majorR + minorR * cos_phi) * cos_theta;
            float y = minorR * sin_phi;
            float z = (majorR + minorR * cos_phi) * sin_theta;

            Vertex v;
            v.position = {x, y, z};
            v.normal   = {cos_phi * cos_theta, sin_phi, cos_phi * sin_theta};
            v.uv       = {
                static_cast<float>(i) / static_cast<float>(majorSegs),
                static_cast<float>(j) / static_cast<float>(minorSegs),
                0.0f};
            v.tangent = tangent_id;

            vertices.push_back(v);
        }
    }

    // Indices: each major-minor cell is a quad (2 triangles)
    for (uint32_t i = 0; i < majorSegs; ++i)
    {
        for (uint32_t j = 0; j < minorSegs; ++j)
        {
            uint32_t i0 = static_cast<uint32_t>(i * (minorSegs + 1) + j);
            uint32_t i1 = i0 + 1;
            uint32_t i2 = i0 + static_cast<uint32_t>(minorSegs + 1);
            uint32_t i3 = i2 + 1;

            // Triangle 1: i0, i2, i1
            indices.push_back(i0);
            indices.push_back(i2);
            indices.push_back(i1);

            // Triangle 2: i1, i2, i3
            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i3);
        }
    }

    float extent = majorR + minorR;
    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {{-extent, -minorR, -extent}, {extent, minorR, extent}};

    return mesh;
}

} // namespace exd::geometry

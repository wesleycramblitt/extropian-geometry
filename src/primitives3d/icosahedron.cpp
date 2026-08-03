#include <exd/geometry/primitives3d.hpp>

#include <cmath>
#include <map>
#include <utility>

namespace exd::geometry
{

static math::Vec3f normalize(const math::Vec3f& v)
{
    float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len < 1e-8f)
        return {0.0f, 1.0f, 0.0f};
    float inv = 1.0f / len;
    return {v.x * inv, v.y * inv, v.z * inv};
}

MeshData generate_icosahedron_mesh(float radius, int subdivisions, math::Quat color)
{
    if (radius <= 0.0f)
        return {};

    if (subdivisions < 0)
        subdivisions = 0;

    constexpr float phi = 1.6180339887498948482f; // (1 + sqrt(5)) / 2

    // 12 vertices of a regular icosahedron (will be normalized to unit sphere)
    math::Vec3f baseVerts[12] = {
        { 0.0f,  1.0f,  phi},
        { 0.0f, -1.0f,  phi},
        { 0.0f,  1.0f, -phi},
        { 0.0f, -1.0f, -phi},
        { 1.0f,  phi,  0.0f},
        {-1.0f,  phi,  0.0f},
        { 1.0f, -phi,  0.0f},
        {-1.0f, -phi,  0.0f},
        { phi,  0.0f,  1.0f},
        {-phi,  0.0f,  1.0f},
        { phi,  0.0f, -1.0f},
        {-phi,  0.0f, -1.0f},
    };

    // Normalize to unit sphere
    for (int i = 0; i < 12; ++i)
        baseVerts[i] = normalize(baseVerts[i]);

    // 20 triangular faces
    uint32_t baseFaces[20][3] = {
        {0, 1, 8}, {0, 8, 4}, {0, 4, 5}, {0, 5, 9}, {0, 9, 1},
        {1, 6, 8}, {8, 10, 6}, {8, 4, 10}, {4, 2, 10}, {4, 5, 2},
        {5, 7, 2}, {5, 9, 7}, {9, 11, 7}, {9, 1, 11}, {1, 6, 11},
        {6, 3, 11}, {6, 10, 3}, {10, 2, 3}, {2, 7, 3}, {7, 11, 3},
    };

    std::vector<math::Vec3f> verts;
    verts.reserve(12);
    for (int i = 0; i < 12; ++i)
        verts.push_back(baseVerts[i]);

    std::vector<std::array<uint32_t, 3>> faces;
    faces.reserve(20);
    for (int i = 0; i < 20; ++i)
        faces.push_back({baseFaces[i][0], baseFaces[i][1], baseFaces[i][2]});

    // Subdivision: each level splits each triangle into 4
    using EdgeKey = std::pair<uint32_t, uint32_t>;
    std::map<EdgeKey, uint32_t> midpointCache;

    auto getMidpoint = [&](uint32_t i0, uint32_t i1) -> uint32_t
    {
        EdgeKey key = (i0 < i1) ? EdgeKey{i0, i1} : EdgeKey{i1, i0};
        auto it = midpointCache.find(key);
        if (it != midpointCache.end())
            return it->second;

        math::Vec3f mid = {
            (verts[i0].x + verts[i1].x) * 0.5f,
            (verts[i0].y + verts[i1].y) * 0.5f,
            (verts[i0].z + verts[i1].z) * 0.5f};
        mid = normalize(mid);

        uint32_t idx = static_cast<uint32_t>(verts.size());
        verts.push_back(mid);
        midpointCache[key] = idx;
        return idx;
    };

    for (int sub = 0; sub < subdivisions; ++sub)
    {
        std::vector<std::array<uint32_t, 3>> newFaces;
        newFaces.reserve(faces.size() * 4);
        midpointCache.clear();

        for (const auto& f : faces)
        {
            uint32_t a = f[0], b = f[1], c = f[2];
            uint32_t ab = getMidpoint(a, b);
            uint32_t bc = getMidpoint(b, c);
            uint32_t ca = getMidpoint(c, a);

            newFaces.push_back({a, ab, ca});
            newFaces.push_back({b, bc, ab});
            newFaces.push_back({c, ca, bc});
            newFaces.push_back({ab, bc, ca});
        }

        faces = std::move(newFaces);
    }

    // Build mesh
    size_t vertCount = verts.size();
    size_t idxCount  = faces.size() * 3;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(vertCount);
    indices.reserve(idxCount);

    for (size_t i = 0; i < vertCount; ++i)
    {
        math::Vec3f pos = {verts[i].x * radius, verts[i].y * radius, verts[i].z * radius};
        math::Vec3f nrm = normalize(verts[i]); // unit normal (same as normalized position)

        // Spherical UV mapping
        float u = 0.5f + std::atan2(nrm.z, nrm.x) / (2.0f * 3.14159265358979323846f);
        float v = 0.5f - std::asin(nrm.y) / 3.14159265358979323846f;

        Vertex vtx;
        vtx.position = pos;
        vtx.normal   = nrm;
        vtx.uv       = {u, v, 0.0f};
        vtx.color    = color;
        vertices.push_back(vtx);
    }

    for (const auto& f : faces)
    {
        indices.push_back(f[0]);
        indices.push_back(f[1]);
        indices.push_back(f[2]);
    }

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {{-radius, -radius, -radius}, {radius, radius, radius}};

    return mesh;
}

} // namespace exd::geometry

#include <exd/geometry/primitives3d.hpp>

#include <cmath>

namespace exd::geometry
{

MeshData generate_capsule_mesh(const CapsuleGeometry& geometry)
{
    constexpr float pi = 3.14159265358979323846f;

    if (geometry.radius <= 0.0f && geometry.height <= 0.0f)
        return {};

    uint32_t slices = geometry.slices < 3 ? 3 : geometry.slices;
    uint32_t stacks = geometry.stacks < 2 ? 2 : geometry.stacks;
    float radius = geometry.radius;
    float halfH = geometry.height * 0.5f;

    // Vertex layout:
    //   Top hemisphere:    (stacks+1) rings x (slices+1) verts  [indices 0 .. topHemiVerts-1]
    //   Bottom hemisphere: (stacks+1) rings x (slices+1) verts  [indices topHemiVerts .. end]
    //
    // The top hemisphere equator (ring `stacks`) and bottom hemisphere equator (ring `0`)
    // serve as the cylinder body's top and bottom rings — no separate cylinder vertices.

    size_t vertsPerRing = static_cast<size_t>(slices + 1);
    size_t topHemiVerts = static_cast<size_t>(stacks + 1) * vertsPerRing;
    size_t totalVerts   = topHemiVerts * 2;

    size_t hemiIdxCount = static_cast<size_t>(stacks) * slices * 6;
    size_t cylIdxCount  = static_cast<size_t>(slices) * 6;
    size_t totalIdxCount = hemiIdxCount * 2 + cylIdxCount;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(totalVerts);
    indices.reserve(totalIdxCount);

    // Helper: add a vertex on a sphere centered at (0, cy, 0) with given polar angle
    auto addSphereVertex = [&](float cy, float theta, float phi) -> Vertex
    {
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);
        float cos_phi   = std::cos(phi);
        float sin_phi   = std::sin(phi);

        Vertex v;
        v.position = {
            radius * sin_theta * cos_phi,
            cy + radius * cos_theta,
            radius * sin_theta * sin_phi};
        // Normal = normalized direction from sphere center
        v.normal = {sin_theta * cos_phi, cos_theta, sin_theta * sin_phi};
        // UV: u around circumference, v from pole (0) to equator (1) or vice versa
        v.uv = {
            static_cast<float>(0) / static_cast<float>(slices), // placeholder, set below
            0.0f,
            0.0f};
        v.color = geometry.color;
        return v;
    };

    // Top hemisphere: theta from 0 (pole) to pi/2 (equator)
    // Center at (0, halfH, 0)
    for (uint32_t ring = 0; ring <= stacks; ++ring)
    {
        float theta = static_cast<float>(ring) * (pi * 0.5f) / static_cast<float>(stacks);
        float v_uv  = static_cast<float>(ring) / static_cast<float>(stacks); // 0 at pole, 1 at equator

        for (uint32_t slice = 0; slice <= slices; ++slice)
        {
            float phi = static_cast<float>(slice) * 2.0f * pi / static_cast<float>(slices);
            float u_uv = static_cast<float>(slice) / static_cast<float>(slices);

            Vertex v = addSphereVertex(halfH, theta, phi);
            v.uv = {u_uv, v_uv, 0.0f};
            vertices.push_back(v);
        }
    }

    // Bottom hemisphere: theta from pi/2 (equator) to pi (pole)
    // Center at (0, -halfH, 0)
    for (uint32_t ring = 0; ring <= stacks; ++ring)
    {
        float theta = pi * 0.5f + static_cast<float>(ring) * (pi * 0.5f) / static_cast<float>(stacks);
        float v_uv  = static_cast<float>(ring) / static_cast<float>(stacks); // 0 at equator, 1 at pole

        for (uint32_t slice = 0; slice <= slices; ++slice)
        {
            float phi = static_cast<float>(slice) * 2.0f * pi / static_cast<float>(slices);
            float u_uv = static_cast<float>(slice) / static_cast<float>(slices);

            Vertex v = addSphereVertex(-halfH, theta, phi);
            v.uv = {u_uv, v_uv, 0.0f};
            vertices.push_back(v);
        }
    }

    // Top hemisphere indices: quads between adjacent rings
    for (uint32_t ring = 0; ring < stacks; ++ring)
    {
        for (uint32_t slice = 0; slice < slices; ++slice)
        {
            uint32_t i0 = ring * static_cast<uint32_t>(vertsPerRing) + slice;
            uint32_t i1 = i0 + 1;
            uint32_t i2 = i0 + static_cast<uint32_t>(vertsPerRing);
            uint32_t i3 = i2 + 1;

            indices.push_back(i0);
            indices.push_back(i2);
            indices.push_back(i1);

            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i3);
        }
    }

    // Bottom hemisphere indices: same pattern, offset by topHemiVerts
    uint32_t bottomOffset = static_cast<uint32_t>(topHemiVerts);
    for (uint32_t ring = 0; ring < stacks; ++ring)
    {
        for (uint32_t slice = 0; slice < slices; ++slice)
        {
            uint32_t i0 = bottomOffset + ring * static_cast<uint32_t>(vertsPerRing) + slice;
            uint32_t i1 = i0 + 1;
            uint32_t i2 = i0 + static_cast<uint32_t>(vertsPerRing);
            uint32_t i3 = i2 + 1;

            indices.push_back(i0);
            indices.push_back(i2);
            indices.push_back(i1);

            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i3);
        }
    }

    // Cylinder body: connect top hemisphere equator to bottom hemisphere equator
    // Top equator: top hemisphere, ring `stacks`
    // Bottom equator: bottom hemisphere, ring `0`
    uint32_t topEqBase    = stacks * static_cast<uint32_t>(vertsPerRing);
    uint32_t bottomEqBase = bottomOffset; // bottom hemisphere, ring 0

    for (uint32_t slice = 0; slice < slices; ++slice)
    {
        uint32_t t0 = topEqBase + slice;
        uint32_t t1 = t0 + 1;
        uint32_t b0 = bottomEqBase + slice;
        uint32_t b1 = b0 + 1;

        // Triangle 1: t0, b0, t1
        indices.push_back(t0);
        indices.push_back(b0);
        indices.push_back(t1);

        // Triangle 2: t1, b0, b1
        indices.push_back(t1);
        indices.push_back(b0);
        indices.push_back(b1);
    }

    // Bounds
    float maxY = halfH + radius;
    float minY = -halfH - radius;

    MeshData mesh;
    mesh.vertices = std::move(vertices);
    mesh.indices  = std::move(indices);
    mesh.topology = PrimitiveTopology::Triangles;
    mesh.bounds   = {{-radius, minY, -radius}, {radius, maxY, radius}};

    return mesh;
}

} // namespace exd::geometry

#include <exd/geometry/part.hpp>
#include <exd/geometry/mesh_ops.hpp>

namespace exd::geometry
{

Part as_part(std::string name, MeshData mesh)
{
    Part part;
    part.name = std::move(name);
    part.mesh = std::move(mesh);
    return part;
}

Patch make_patch_range(std::string name, uint32_t firstFace, uint32_t faceCount)
{
    Patch patch;
    patch.name = std::move(name);
    patch.faces.reserve(faceCount);
    for (uint32_t f = 0; f < faceCount; ++f)
        patch.faces.push_back(firstFace + f);
    return patch;
}

void tag_faces(Part& part, const std::string& name,
               const std::function<bool(uint32_t faceIndex)>& pred)
{
    const uint32_t nFaces = static_cast<uint32_t>(part.mesh.indices.size() / 3);

    Patch* patch = nullptr;
    for (auto& p : part.patches)
    {
        if (p.name == name)
        {
            patch = &p;
            break;
        }
    }
    if (patch == nullptr)
    {
        part.patches.emplace_back();
        patch = &part.patches.back();
        patch->name = name;
    }

    for (uint32_t f = 0; f < nFaces; ++f)
    {
        if (pred(f))
            patch->faces.push_back(f);
    }
}

Part transform_part(const Part& part, const math::Mat4& transform,
                    bool transformNormals)
{
    Part result;
    result.name   = part.name;
    result.mesh   = transform_mesh(part.mesh, transform, transformNormals);
    result.patches = part.patches;
    return result;
}

Assembly merge_parts(std::span<const Part> parts)
{
    Assembly assembly{};
    if (parts.empty())
        return assembly;

    assembly.parts.assign(parts.begin(), parts.end());

    bool have = false;
    for (const auto& part : assembly.parts)
    {
        if (part.mesh.vertices.empty())
            continue;
        const Bounds b = compute_bounds(part.mesh.vertices);
        if (!have)
        {
            assembly.bounds = b;
            have = true;
        }
        else
        {
            assembly.bounds.min.x = std::min(assembly.bounds.min.x, b.min.x);
            assembly.bounds.min.y = std::min(assembly.bounds.min.y, b.min.y);
            assembly.bounds.min.z = std::min(assembly.bounds.min.z, b.min.z);
            assembly.bounds.max.x = std::max(assembly.bounds.max.x, b.max.x);
            assembly.bounds.max.y = std::max(assembly.bounds.max.y, b.max.y);
            assembly.bounds.max.z = std::max(assembly.bounds.max.z, b.max.z);
        }
    }

    return assembly;
}

Part flatten(const Assembly& assembly)
{
    if (assembly.parts.empty())
        return {};

    if (assembly.parts.size() == 1)
        return assembly.parts[0];

    std::vector<MeshData> meshes;
    meshes.reserve(assembly.parts.size());
    for (const auto& part : assembly.parts)
        meshes.push_back(part.mesh);

    const MeshData merged = merge_meshes(meshes);

    Part result;
    result.mesh = merged;   // carries merged.bounds

    uint32_t triangleOffset = 0;
    for (const auto& part : assembly.parts)
    {
        for (const auto& patch : part.patches)
        {
            Patch remapped;
            remapped.name = part.name.empty() ? patch.name : part.name + "." + patch.name;
            remapped.faces.reserve(patch.faces.size());
            for (uint32_t f : patch.faces)
                remapped.faces.push_back(f + triangleOffset);
            result.patches.push_back(std::move(remapped));
        }
        triangleOffset += static_cast<uint32_t>(part.mesh.indices.size() / 3);
    }

    return result;
}

} // namespace exd::geometry
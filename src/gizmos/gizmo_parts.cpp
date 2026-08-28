#include <exd/geometry/gizmos.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <vector>

namespace exd::geometry
{

MeshData merge_gizmo_parts(const GizmoParts& parts)
{
    if (parts.empty())
        return {};

    std::vector<MeshData> meshes;
    meshes.reserve(parts.size());
    for (const auto& p : parts)
        meshes.push_back(p.mesh);

    return merge_meshes(meshes);
}

GizmoParts filter_gizmo_parts(const GizmoParts& parts, GizmoPartKind kind, GizmoAxis axis)
{
    GizmoParts out;
    for (const auto& p : parts)
    {
        if (p.kind != kind)
            continue;
        if (axis != GizmoAxis::None && p.axis != axis)
            continue;
        out.push_back(p);
    }
    return out;
}

} // namespace exd::geometry
#pragma once

// Internal: shared flattening + patch enumeration for CAE adapters (Phase B).
// Not installed; consumed by stl/msh/vtk/step writers.

#include <exd/geometry/cad_model.hpp>

#include <string>
#include <vector>

namespace exd::geometry
{

/// A flat, renderer-agnostic triangle mesh over a CADModel's parts.
///   • points  — concatenated vertex positions (deduplicated per part,
///     not across parts — parts stay separately addressable).
///   • cells   — triangle connectivity with global (0-based) point indices,
///     part index, and a 1-based global patch id (0 = face with no patch).
///   • part_names / patch_names — one entry per index; patch id k uses
///     patch_names[k-1].
struct FlatMesh
{
    struct Cell
    {
        uint32_t a, b, c;
        int32_t  part  = -1;
        int32_t  patch = 0;   // 1-based global; 0 = none
    };

    std::vector<math::Vec3f> points;
    std::vector<Cell>        cells;
    std::vector<std::string> part_names;
    std::vector<std::string> patch_names;   // index = global patch id - 1
};

/// Flatten the model. Stereo-typical patch assignment: a triangle belongs to
/// the FIRST patch (declaration order) containing it; otherwise no patch.
/// Non-Triangles topologies are skipped.
inline FlatMesh flatten_cad(const CADModel& model)
{
    FlatMesh fm;
    uint32_t pointOffset = 0;
    int32_t  patchBase = 0;   // global id offset for each part's patches

    for (size_t pi = 0; pi < model.parts.size(); ++pi)
    {
        const Part&   part = model.parts[pi];
        const MeshData& m = part.mesh;
        const std::string pname = part.name.empty()
                                ? ("part_" + std::to_string(pi)) : part.name;
        fm.part_names.push_back(pname);
        const int32_t nPatches = static_cast<int32_t>(part.patches.size());
        if (m.topology != PrimitiveTopology::Triangles)
        {
            patchBase += nPatches;
            continue;
        }

        // face (triangle ordinal) → global patch id; first matching patch wins
        const size_t triCount = m.indices.size() / 3;
        std::vector<int32_t> facePatch(triCount, 0);
        for (int32_t j = 0; j < nPatches; ++j)
        {
            const Patch& pa = part.patches[static_cast<size_t>(j)];
            fm.patch_names.push_back(pname + "." + pa.name);
            for (uint32_t f : pa.faces)
                if (f < triCount && facePatch[f] == 0)
                    facePatch[f] = patchBase + j + 1;
        }

        for (const Vertex& v : m.vertices)
            fm.points.push_back(v.position);

        for (size_t t = 0; t < triCount; ++t)
        {
            fm.cells.push_back({pointOffset + m.indices[3 * t],
                                pointOffset + m.indices[3 * t + 1],
                                pointOffset + m.indices[3 * t + 2],
                                static_cast<int32_t>(pi), facePatch[t]});
        }
        pointOffset += static_cast<uint32_t>(m.vertices.size());
        patchBase += nPatches;
    }
    return fm;
}

} // namespace exd::geometry

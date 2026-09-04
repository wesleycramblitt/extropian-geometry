#include <exd/geometry/export.hpp>
#include "cae_common.hpp"

#include <cstdio>
#include <map>
#include <sstream>
#include <vector>

namespace exd::geometry
{
namespace
{

std::string fmt_f(float v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.8g", static_cast<double>(v));
    return buf;
}

} // namespace

std::string to_msh(const CADModel& model)
{
    const FlatMesh fm = flatten_cad(model);

    // ── Physical surface group assignment (deterministic, first-use order) ──
    //   "part" groups (dim 2) for every part that produced triangles, and
    //   "part.patch" groups per patch, both as surface physical names.
    std::map<std::string, int> groupTag;         // name → physical tag
    std::vector<std::string>   groupNames;       // by tag-1
    auto group_for = [&](std::string name) -> int {
        const auto it = groupTag.find(name);
        if (it != groupTag.end())
            return it->second;
        const int tag = static_cast<int>(groupNames.size()) + 1;
        groupTag.emplace(name, tag);
        groupNames.push_back(std::move(name));
        return tag;
    };

    std::vector<int> cellPhys(fm.cells.size(), 0);
    for (size_t c = 0; c < fm.cells.size(); ++c)
    {
        // First patch containing the face wins; faces without a patch go into
        // the part-level group ("part"). A part-level group is only created
        // when an unpatched face actually needs it.
        int tag;
        if (fm.cells[c].patch > 0)
        {
            const int pi = fm.cells[c].patch - 1;
            tag = group_for(fm.patch_names[static_cast<size_t>(pi)]);
        }
        else
        {
            tag = group_for(fm.part_names[static_cast<size_t>(fm.cells[c].part)]);
        }
        cellPhys[c] = tag;
    }

    std::ostringstream os;
    os << "$MeshFormat\n2.2 0 8\n$EndMeshFormat\n";

    // ── Physical names ──────────────────────────────────────────────────
    os << "$PhysicalNames\n" << groupNames.size() << "\n";
    for (int t = 0; t < static_cast<int>(groupNames.size()); ++t)
        os << (t + 1) << " 2 \"" << groupNames[static_cast<size_t>(t)] << "\"\n";
    os << "$EndPhysicalNames\n";

    // ── Nodes (1-based global) ──────────────────────────────────────────
    os << "$Nodes\n" << fm.points.size() << "\n";
    for (size_t i = 0; i < fm.points.size(); ++i)
        os << (i + 1) << " " << fmt_f(fm.points[i].x) << " " << fmt_f(fm.points[i].y)
           << " " << fmt_f(fm.points[i].z) << "\n";
    os << "$EndNodes\n";

    // ── Elements (type 2 = 3-node triangle; phys + elementary tags) ─────
    os << "$Elements\n" << fm.cells.size() << "\n";
    for (size_t c = 0; c < fm.cells.size(); ++c)
    {
        const FlatMesh::Cell& cell = fm.cells[c];
        const int elemTag = cell.part + 1;   // elementary region = part
        os << (c + 1) << " 2 2 " << cellPhys[c] << " " << elemTag << " 3 "
           << (cell.a + 1) << " " << (cell.b + 1) << " " << (cell.c + 1) << "\n";
    }
    os << "$EndElements\n";
    return os.str();
}

} // namespace exd::geometry

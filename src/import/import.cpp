#include <exd/geometry/import.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <istream>
#include <map>
#include <sstream>
#include <vector>

namespace exd::geometry
{
namespace
{

// ── Token helpers ───────────────────────────────────────────────────────────

std::vector<std::string> split_ws(const std::string& s)
{
    std::vector<std::string> out;
    std::istringstream iss(s);
    std::string tok;
    while (iss >> tok)
        out.push_back(tok);
    return out;
}

math::Vec3f parse_coord(const std::vector<std::string>& tok, size_t base)
{
    math::Vec3f v{0, 0, 0};
    if (tok.size() >= base + 3)
    {
        v.x = std::strtof(tok[base].c_str(), nullptr);
        v.y = std::strtof(tok[base + 1].c_str(), nullptr);
        v.z = std::strtof(tok[base + 2].c_str(), nullptr);
    }
    return v;
}

void set_bounds(MeshData& m)
{
    m.bounds = compute_bounds(m.vertices);
}

// ── OBJ ─────────────────────────────────────────────────────────────────────

MeshData parse_obj_inner(const std::string& text)
{
    MeshData out;
    out.topology = PrimitiveTopology::Triangles;
    std::vector<math::Vec3f> pos{{0, 0, 0}};   // 1-based indices
    std::vector<math::Vec3f> nrm{{0, 1, 0}};
    std::istringstream iss(text);
    std::string line;
    while (std::getline(iss, line))
    {
        const std::vector<std::string> t = split_ws(line);
        if (t.empty())
            continue;
        if (t[0] == "v")
            pos.push_back(parse_coord(t, 1));
        else if (t[0] == "vn")
            nrm.push_back(parse_coord(t, 1));
        else if (t[0] == "f")
        {
            // vertex[/tex][/norm] per face point
            std::vector<int> idx;
            for (size_t k = 1; k < t.size(); ++k)
            {
                const std::string& token = t[k];
                const std::string vtok = token.substr(0, token.find('/'));
                const int vi = std::atoi(vtok.c_str());
                idx.push_back(vi > 0 ? vi - 1 : static_cast<int>(pos.size()) + vi);
            }
            if (idx.size() < 3)
                continue;
            for (size_t k = 2; k < idx.size(); ++k)
            {
                out.indices.push_back(static_cast<uint32_t>(idx[0]));
                out.indices.push_back(static_cast<uint32_t>(idx[k - 1]));
                out.indices.push_back(static_cast<uint32_t>(idx[k]));
            }
        }
    }
    out.vertices.reserve(pos.size());
    for (size_t i = 1; i < pos.size(); ++i)
    {
        Vertex v;
        v.position = pos[i];
        v.normal = (i < nrm.size()) ? nrm[i] : math::Vec3f{0, 1, 0};
        out.vertices.push_back(v);
    }
    set_bounds(out);
    return out;
}

// ── STL ─────────────────────────────────────────────────────────────────────

bool looks_binary(const std::string& data)
{
    // Multi-solid files carry one 84-byte header per part. Binary if we can
    // partition the stream into k ≥ 1 header + (triangles × 50) segments.
    if (data.size() < 84 || data.compare(0, 5, "solid") == 0)
        return false;
    for (size_t k = 1; k <= 16 && k * 84 <= data.size(); ++k)
        if ((data.size() - k * 84) % 50 == 0 && (data.size() - k * 84) >= 50 * 0)
            return true;
    return false;
}

float read_f32(const char* p)
{
    float v = 0.0f;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

MeshData parse_stl_binary_inner(const std::string& data)
{
    MeshData out;
    out.topology = PrimitiveTopology::Triangles;
    size_t cursor = 0;
    while (cursor + 84 <= data.size())
    {
        uint32_t triCount = 0;
        std::memcpy(&triCount, data.data() + cursor + 80, 4);
        if (triCount == 0)
            break;
        for (uint32_t t = 0; t < triCount; ++t)
        {
            const size_t base = cursor + 84 + static_cast<size_t>(t) * 50;
            if (base + 44 > data.size())
                break;
            // normal (ignored), then three vertices
            const uint32_t vi = static_cast<uint32_t>(out.vertices.size());
            for (int k = 0; k < 3; ++k)
            {
                const char* p = data.data() + base + 12 + static_cast<size_t>(k) * 12;
                Vertex v;
                v.position = {read_f32(p), read_f32(p + 4), read_f32(p + 8)};
                out.vertices.push_back(v);
            }
            out.indices.push_back(vi);
            out.indices.push_back(vi + 1);
            out.indices.push_back(vi + 2);
        }
        cursor += 84 + static_cast<size_t>(triCount) * 50;
    }
    set_bounds(out);
    return out;
}

MeshData parse_stl_ascii_inner(const std::string& text)
{
    MeshData out;
    out.topology = PrimitiveTopology::Triangles;
    std::vector<math::Vec3f> verts;
    std::istringstream iss(text);
    std::string line;
    while (std::getline(iss, line))
    {
        const std::vector<std::string> t = split_ws(line);
        if (t.size() >= 4 && t[0] == "vertex")
            verts.push_back(parse_coord(t, 1));
    }
    const size_t triCount = verts.size() / 3;
    out.vertices.reserve(triCount * 3);
    out.indices.reserve(triCount * 3);
    for (size_t t = 0; t < triCount; ++t)
    {
        for (int k = 0; k < 3; ++k)
        {
            Vertex v;
            v.position = verts[t * 3 + k];
            out.vertices.push_back(v);
        }
        out.indices.push_back(static_cast<uint32_t>(t * 3));
        out.indices.push_back(static_cast<uint32_t>(t * 3 + 1));
        out.indices.push_back(static_cast<uint32_t>(t * 3 + 2));
    }
    set_bounds(out);
    return out;
}

// ── Gmsh .msh (v2.2 ascii) ─────────────────────────────────────────────────

struct MshGroup
{
    int tag = 0;
    int dim = 0;
    std::string name;
};

struct MshNode
{
    math::Vec3f p;
};

struct MshElem
{
    int type = 0;
    int phys = 0;
    int elem = 0;
    std::vector<int> nodes;
};

} // namespace

MeshData parse_obj(const std::string& text)
{
    return parse_obj_inner(text);
}

MeshData parse_stl(const std::string& data)
{
    // Auto-detect: leading "solid" → ascii; otherwise binary iff size matches
    // (84-byte header + n×50); fall back on ascii parse.
    if (looks_binary(data))
        return parse_stl_binary_inner(data);
    return parse_stl_ascii_inner(data);
}

CADModel import_obj(const std::string& text, const std::string& name)
{
    MeshData m = parse_obj_inner(text);
    return make_cad_model(name, std::vector<Part>{as_part(name, std::move(m))});
}

CADModel import_stl(const std::string& data, const std::string& name)
{
    MeshData m = parse_stl(data);
    return make_cad_model(name, std::vector<Part>{as_part(name, std::move(m))});
}

CADModel import_msh(const std::string& text, const std::string& name)
{
    std::vector<MshGroup> groups;
    std::vector<MshNode> nodes;
    std::vector<MshElem> elems;

    std::string section;
    long expectedCount = -1;
    long parsedCount = 0;

    std::istringstream iss(text);
    std::string line;
    const auto newSection = [&](const std::string& token) {
        section = token;
        expectedCount = -1;
        parsedCount = 0;
    };

    while (std::getline(iss, line))
    {
        const std::vector<std::string> t = split_ws(line);
        if (t.empty())
            continue;
        if (t[0] == "$MeshFormat" || t[0] == "$EndMeshFormat" ||
            t[0] == "$EndPhysicalNames" || t[0] == "$EndNodes" || t[0] == "$EndElements")
            continue;
        if (t[0] == "$PhysicalNames") { newSection("PhysicalNames"); continue; }
        if (t[0] == "$Nodes")         { newSection("Nodes"); continue; }
        if (t[0] == "$Elements")      { newSection("Elements"); continue; }
        if (section.empty())
            continue;

        if (expectedCount < 0 && t.size() == 1)
        {
            expectedCount = std::atol(t[0].c_str());
            continue;
        }

        if (section == "PhysicalNames" && t.size() >= 3)
        {
            MshGroup g;
            g.tag  = std::atoi(t[0].c_str());
            g.dim  = std::atoi(t[1].c_str());
            std::string nm = t[2];
            if (nm.size() >= 2 && nm.front() == '"' && nm.back() == '"')
                nm = nm.substr(1, nm.size() - 2);
            g.name = nm;
            groups.push_back(g);
        }
        else if (section == "Nodes" && t.size() >= 4)
        {
            MshNode n;
            n.p = {std::strtof(t[1].c_str(), nullptr),
                   std::strtof(t[2].c_str(), nullptr),
                   std::strtof(t[3].c_str(), nullptr)};
            nodes.push_back(n);
        }
        else if (section == "Elements" && t.size() >= 7)
        {
            MshElem e;
            e.type   = std::atoi(t[1].c_str());
            const int nTags = std::atoi(t[2].c_str());
            if (nTags >= 1) e.phys = std::atoi(t[3].c_str());
            if (nTags >= 2) e.elem = std::atoi(t[4].c_str());
            // element line: id type nTags tags... nn n1 n2...  →  nn at 3+nTags
            const int nnIndex = 3 + nTags;
            const int nn = std::atoi(t[static_cast<size_t>(nnIndex)].c_str());
            for (int k = 0; k < nn; ++k)
                e.nodes.push_back(std::atoi(t[static_cast<size_t>(nnIndex + 1 + k)].c_str()));
            if (e.type == 2 && e.nodes.size() == 3)
                elems.push_back(e);
        }
        ++parsedCount;
        (void)parsedCount;
    }

    // group lookup by tag
    auto group_name = [&](int tag) -> std::string {
        for (const MshGroup& g : groups)
            if (g.tag == tag)
                return g.name;
        return {};
    };

    // split "part.patch" → {part, patch}; plain "part" → {part, ""}
    auto split_name = [](const std::string& nm, std::string& part, std::string& patch) {
        const size_t dot = nm.find('.');
        if (dot != std::string::npos)
        {
            part  = nm.substr(0, dot);
            patch = nm.substr(dot + 1);
        }
        else
        {
            part  = nm;
            patch.clear();
        }
    };

    // group elements by elementary region tag (ordered by first appearance)
    std::map<int, std::vector<size_t>, std::less<>>  byElem;
    std::vector<int> elemOrder;
    for (size_t i = 0; i < elems.size(); ++i)
    {
        const MshElem& e = elems[i];
        if (byElem.find(e.elem) == byElem.end())
        {
            byElem[e.elem] = {};
            elemOrder.push_back(e.elem);
        }
        byElem[e.elem].push_back(i);
    }

    std::vector<Part> parts;
    for (const int elemTag : elemOrder)
    {
        const auto& indexes = byElem[elemTag];

        // part name: first dot-name prefix, else part-level group, else "part_N"
        std::string partName;
        std::string fallbackName;
        bool havePatchName = false;
        for (const size_t i : indexes)
        {
            const MshElem& e = elems[i];
            const std::string nm = group_name(e.phys);
            if (nm.empty())
                continue;
            std::string pn, pc;
            split_name(nm, pn, pc);
            if (!pc.empty())
            {
                if (!havePatchName)
                {
                    partName = pn;
                    havePatchName = true;
                }
                else if (pn != partName)
                {
                    havePatchName = false;   // inconsistent → use fallback
                    partName.clear();
                    break;
                }
            }
            else if (fallbackName.empty())
                fallbackName = pn;
        }
        if (partName.empty())
            partName = !fallbackName.empty() ? fallbackName : ("part_" + std::to_string(elemTag));

        // mesh
        MeshData m;
        m.topology = PrimitiveTopology::Triangles;
        std::map<int, uint32_t, std::less<>> localIndex;
        std::vector<Patch> patches;
        std::map<std::string, std::vector<uint32_t>, std::less<>> patchFaces;

        for (const size_t i : indexes)
        {
            const MshElem& e = elems[i];
            uint32_t tri[3];
            for (int k = 0; k < 3; ++k)
            {
                const int nid = e.nodes[k];
                const auto it = localIndex.find(nid);
                if (it != localIndex.end())
                {
                    tri[k] = it->second;
                }
                else
                {
                    const uint32_t vi = static_cast<uint32_t>(m.vertices.size());
                    Vertex v;
                    v.position = (nid >= 1 && static_cast<size_t>(nid) <= nodes.size())
                               ? nodes[static_cast<size_t>(nid) - 1].p : math::Vec3f{};
                    m.vertices.push_back(v);
                    localIndex[nid] = vi;
                    tri[k] = vi;
                }
            }
            m.indices.push_back(tri[0]);
            m.indices.push_back(tri[1]);
            m.indices.push_back(tri[2]);
            const uint32_t faceOrdinal = static_cast<uint32_t>(m.indices.size() / 3) - 1;

            const std::string nm = group_name(e.phys);
            if (!nm.empty())
            {
                std::string pn, pc;
                split_name(nm, pn, pc);
                if (!pc.empty())
                    patchFaces[pc].push_back(faceOrdinal);
            }
        }
        // recompute outward-ish normals post-hoc (flat, then smooth is out of
        // scope for import — flat normals from index winding via recompute_normals)
        m = recompute_normals(m, NormalMode::Smooth);
        set_bounds(m);

        Part part = as_part(partName, std::move(m));
        for (auto& [pname, faces] : patchFaces)
            part.patches.push_back({pname, std::move(faces)});
        parts.push_back(std::move(part));
    }

    return make_cad_model(name, parts);
}

} // namespace exd::geometry

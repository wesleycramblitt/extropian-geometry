#include <exd/geometry/export.hpp>
#include "cae_common.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>

namespace exd::geometry
{
namespace
{

/// Triangle geometry + facet normal (right-hand rule), SR (positions).
/// Returns false if the index range is out of bounds; degenerate triangles
/// get a zero normal.
bool tri_geom(const MeshData& m, size_t t, math::Vec3f& n,
              math::Vec3f& pa, math::Vec3f& pb, math::Vec3f& pc)
{
    if (t * 3 + 2 >= m.indices.size())
        return false;
    pa = m.vertices[m.indices[t * 3]].position;
    pb = m.vertices[m.indices[t * 3 + 1]].position;
    pc = m.vertices[m.indices[t * 3 + 2]].position;
    const math::Vec3f e1 = pb - pa;
    const math::Vec3f e2 = pc - pa;
    n = e1.cross(e2);
    const float len = n.length();
    if (len > 1e-12f)
        n = n / len;
    else
        n = {0.0f, 0.0f, 0.0f};
    return true;
}

std::string fmt_f(float v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.8g", static_cast<double>(v));
    return buf;
}

void append_f32(std::string& s, float v)
{
    char b[sizeof(float)];
    std::memcpy(b, &v, sizeof(float));
    s.append(b, sizeof(float));
}

void append_u32(std::string& s, uint32_t v)
{
    char b[4];
    std::memcpy(b, &v, 4);
    s.append(b, 4);
}

void append_u16(std::string& s, uint16_t v)
{
    char b[2];
    std::memcpy(b, &v, 2);
    s.append(b, 2);
}

void append_vec(std::string& s, const math::Vec3f& v)
{
    append_f32(s, v.x);
    append_f32(s, v.y);
    append_f32(s, v.z);
}

void write_stl_ascii_solid(std::ostringstream& os, const MeshData& m, const std::string& name)
{
    os << "solid " << name << "\n";
    const size_t triCount = m.indices.size() / 3;
    for (size_t t = 0; t < triCount; ++t)
    {
        math::Vec3f n, pa, pb, pc;
        if (!tri_geom(m, t, n, pa, pb, pc))
            continue;
        os << "  facet normal " << fmt_f(n.x) << " " << fmt_f(n.y) << " " << fmt_f(n.z) << "\n";
        os << "    outer loop\n";
        os << "      vertex " << fmt_f(pa.x) << " " << fmt_f(pa.y) << " " << fmt_f(pa.z) << "\n";
        os << "      vertex " << fmt_f(pb.x) << " " << fmt_f(pb.y) << " " << fmt_f(pb.z) << "\n";
        os << "      vertex " << fmt_f(pc.x) << " " << fmt_f(pc.y) << " " << fmt_f(pc.z) << "\n";
        os << "    endloop\n";
        os << "  endfacet\n";
    }
    os << "endsolid " << name << "\n";
}

void write_stl_binary_solid(std::string& out, const MeshData& m, const std::string& name)
{
    const size_t triCount = m.indices.size() / 3;
    char header[80];
    std::memset(header, 0, sizeof(header));
    std::strncpy(header, name.c_str(), sizeof(header) - 1);
    out.append(header, sizeof(header));
    append_u32(out, static_cast<uint32_t>(triCount));
    for (size_t t = 0; t < triCount; ++t)
    {
        math::Vec3f n, pa, pb, pc;
        if (!tri_geom(m, t, n, pa, pb, pc))
            continue;
        append_vec(out, n);
        append_vec(out, pa);
        append_vec(out, pb);
        append_vec(out, pc);
        append_u16(out, 0u);   // attribute byte count (uint16 per STL spec)
    }
}

} // namespace

std::string to_stl_ascii(const MeshData& mesh)
{
    std::ostringstream os;
    write_stl_ascii_solid(os, mesh, "mesh");
    return os.str();
}

std::string to_stl_binary(const MeshData& mesh)
{
    std::string out;
    write_stl_binary_solid(out, mesh, "mesh");
    return out;
}

std::string to_stl_ascii(const CADModel& model)
{
    std::ostringstream os;
    for (size_t i = 0; i < model.parts.size(); ++i)
    {
        const std::string name = model.parts[i].name.empty()
                               ? ("part_" + std::to_string(i)) : model.parts[i].name;
        write_stl_ascii_solid(os, model.parts[i].mesh, name);
    }
    return os.str();
}

std::string to_stl_binary(const CADModel& model)
{
    std::string out;
    for (size_t i = 0; i < model.parts.size(); ++i)
    {
        const std::string name = model.parts[i].name.empty()
                               ? ("part_" + std::to_string(i)) : model.parts[i].name;
        write_stl_binary_solid(out, model.parts[i].mesh, name);
    }
    return out;
}

} // namespace exd::geometry

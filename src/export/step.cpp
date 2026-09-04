#include <exd/geometry/export.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace exd::geometry
{
namespace
{

std::string fmt(double v)
{
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return buf;
}

/// STEP strings escape a single quote by doubling it.
std::string esc(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s)
    {
        if (c == '\'')
            out += "''";
        else
            out += c;
    }
    return out;
}

/// Emit one triangle as a planar ADVANCED_FACE (faceted solid B-rep). All
/// entities are appended via `id(line)`; returns the face entity id, or -1 if
/// the triangle is degenerate (skipped).
int emit_face(MeshData const& m, size_t t, const std::vector<int>& vertexPoint,
              const std::function<int(const std::string&)>& id)
{
    const uint32_t ia = m.indices[3 * t];
    const uint32_t ib = m.indices[3 * t + 1];
    const uint32_t ic = m.indices[3 * t + 2];
    const math::Vec3f pa = m.vertices[ia].position;
    const math::Vec3f pb = m.vertices[ib].position;
    const math::Vec3f pc = m.vertices[ic].position;

    const math::Vec3f n = (pb - pa).cross(pc - pa);
    const float len3 = n.length();
    if (len3 < 1e-12f)
        return -1;
    const math::Vec3f nn = n / len3;

    // Orthonormal in-plane reference direction for AXIS2_PLACEMENT_3D.
    math::Vec3f base{0.0f, 0.0f, 1.0f};
    if (std::abs(nn.z) > 0.99f)
        base = {0.0f, 1.0f, 0.0f};
    const math::Vec3f rawX = base - nn * base.dot(nn);
    const math::Vec3f xdir = rawX / rawX.length();

    const int planeLoc = id("CARTESIAN_POINT('',(" + fmt(pa.x) + "," + fmt(pa.y) + "," + fmt(pa.z) + "))");
    const int planeDir = id("DIRECTION('',(" + fmt(nn.x) + "," + fmt(nn.y) + "," + fmt(nn.z) + "))");
    const int refDir   = id("DIRECTION('',(" + fmt(xdir.x) + "," + fmt(xdir.y) + "," + fmt(xdir.z) + "))");
    const int axis     = id("AXIS2_PLACEMENT_3D('',#" + std::to_string(planeLoc) + ",#" +
                            std::to_string(planeDir) + ",#" + std::to_string(refDir) + ")");
    const int plane    = id("PLANE('',#" + std::to_string(axis) + ")");

    // Directed edge a→b→c→a (CCW about the outward normal → outer bound).
    auto emit_edge = [&](uint32_t u, uint32_t v) -> int {
        const math::Vec3f& pu = m.vertices[u].position;
        const math::Vec3f& pv = m.vertices[v].position;
        const math::Vec3f d = pv - pu;
        const math::Vec3f dir = d.length() > 1e-12f ? d / d.length() : math::Vec3f{1.0f, 0.0f, 0.0f};
        const int ep = id("CARTESIAN_POINT('',(" + fmt(pu.x) + "," + fmt(pu.y) + "," + fmt(pu.z) + "))");
        const int ed = id("DIRECTION('',(" + fmt(dir.x) + "," + fmt(dir.y) + "," + fmt(dir.z) + "))");
        const int vc = id("VECTOR('',#" + std::to_string(ed) + ",1.)");
        const int ln = id("LINE('',#" + std::to_string(ep) + ",#" + std::to_string(vc) + ")");
        const int ec = id("EDGE_CURVE('',#" + std::to_string(vertexPoint[u]) + ",#" +
                          std::to_string(vertexPoint[v]) + ",#" + std::to_string(ln) + ",.T.)");
        return id("ORIENTED_EDGE('',*,*,#" + std::to_string(ec) + ",.T.)");
    };

    const int oe1 = emit_edge(ia, ib);
    const int oe2 = emit_edge(ib, ic);
    const int oe3 = emit_edge(ic, ia);
    const int loop = id("EDGE_LOOP('',(#" + std::to_string(oe1) + ",#" +
                        std::to_string(oe2) + ",#" + std::to_string(oe3) + "))");
    const int bound = id("FACE_OUTER_BOUND('',#" + std::to_string(loop) + ",.T.)");
    return id("ADVANCED_FACE('',(#" + std::to_string(bound) + "),#" + std::to_string(plane) + ",.T.)");
}

} // namespace

std::string to_step_faceted(const CADModel& model, const StepOptions& options)
{
    std::ostringstream os;
    os << "ISO-10303-21;\n";
    os << "HEADER;\n";
    os << "FILE_DESCRIPTION((''),'2;1');\n";
    os << "FILE_NAME('" << esc(options.model_name) << "','2026-09-04T00:00:00',"
       << "('extropian-geometry'),(''),'','extropian-geometry','');\n";
    const std::string schema = (options.schema == "AUTOMOTIVE_DESIGN")
                             ? "AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 }"
                             : options.schema;
    os << "FILE_SCHEMA(('" << schema << "'));\n";
    os << "ENDSEC;\nDATA;\n";

    int next = 1;
    const auto id = [&](const std::string& body) -> int {
        os << "#" << next << "=" << body << ";\n";
        return next++;
    };

    // ── Units + representation context (SI millimetre — faceted, scale as authored) ──
    const int unitLen = id("LENGTH_UNIT()NAMED_UNIT(* )SI_UNIT(.MILLI.,.METRE.)");
    id("NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.)");
    id("NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT()");
    const int unc = id("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-4),#" +
                       std::to_string(unitLen) +
                       ",'distance_accuracy_value','confusion accuracy')");
    const int ctx = id("(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
                       "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" + std::to_string(unc) +
                       "))REPRESENTATION_CONTEXT('extropian-geometry','STEP faceted B-rep context'))");
    const int appCtx = id("APPLICATION_CONTEXT('automotive design')");

    for (const Part& part : model.parts)
    {
        const MeshData& m = part.mesh;
        if (m.topology != PrimitiveTopology::Triangles || m.vertices.empty() || m.indices.empty())
            continue;

        // Watertight gate (public closed_manifold_gate, D16): only closed
        // solids can be emitted as valid BREP solids.
        const Bounds b = compute_bounds(m.vertices);
        const math::Vec3f span{b.max.x - b.min.x, b.max.y - b.min.y, b.max.z - b.min.z};
        const float diag = span.length();
        const float posEps = diag > 0.0f ? std::max(diag * 1e-7f, 1e-9f) : 1e-6f;
        if (!closed_manifold_gate(m, posEps))
            continue;

        const std::string pname = part.name.empty() ? "part" : part.name;

        // ── Product chain (named part for the importer's part list) ──
        const int ptype = id("PRODUCT_TYPE('part','','')");
        const int prod  = id("PRODUCT('" + esc(pname) + "','" + esc(pname) + "','',(#" +
                             std::to_string(ptype) + "))");
        const int form  = id("PRODUCT_DEFINITION_FORMATION('','',#" + std::to_string(prod) + ")");
        const int pctx  = id("PRODUCT_CONTEXT('',#" + std::to_string(appCtx) + ",'mechanical')");
        const int pdef  = id("PRODUCT_DEFINITION('design','',#" + std::to_string(form) + ",#" +
                             std::to_string(pctx) + ")");
        const int pshape = id("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(pdef) + ")");

        // ── Vertices (one shared CARTESIAN_POINT + VERTEX_POINT per mesh vertex) ──
        std::vector<int> cpPoint(m.vertices.size(), 0);
        std::vector<int> vertexPoint(m.vertices.size(), 0);
        for (size_t vi = 0; vi < m.vertices.size(); ++vi)
        {
            const math::Vec3f& p = m.vertices[vi].position;
            cpPoint[vi]    = id("CARTESIAN_POINT('',(" + fmt(p.x) + "," + fmt(p.y) + "," + fmt(p.z) + "))");
            vertexPoint[vi] = id("VERTEX_POINT('',#" + std::to_string(cpPoint[vi]) + ")");
        }

        // ── Faces ──
        std::vector<int> faces;
        const size_t triCount = m.indices.size() / 3;
        for (size_t t = 0; t < triCount; ++t)
        {
            const int face = emit_face(m, t, vertexPoint, id);
            if (face >= 0)
                faces.push_back(face);
        }
        if (faces.empty())
            continue;

        // ── Shell, solid, representation, shape definition ──
        std::string fidList;
        for (const int f : faces)
        {
            fidList += "#";
            fidList += std::to_string(f);
            fidList += ",";
        }
        fidList.pop_back();
        const int shell = id("CLOSED_SHELL('',(" + fidList + "))");
        const int solid = id("MANIFOLD_SOLID_BREP('',#" + std::to_string(shell) + ")");
        const int srep  = id("ADVANCED_BREP_SHAPE_REPRESENTATION('',(#" +
                             std::to_string(solid) + "),#" + std::to_string(ctx) + ")");
        id("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(pshape) + ",#" +
           std::to_string(srep) + ")");
    }

    os << "ENDSEC;\nEND-ISO-10303-21;\n";
    return os.str();
}

} // namespace exd::geometry

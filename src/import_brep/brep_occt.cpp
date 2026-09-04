// OCCT backend — compiled only when EXD_GEOMETRY_ENABLE_OCCT=ON.
// Uses the most stable OCCT APIs (TKSTEP/TKIGES/TKBRep/TKTopAlgo/TKMesh);
// XCAF product-name mapping is a documented follow-up.

#include <exd/geometry/import_brep.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Reader.hxx>
#include <Standard_Handle.hxx>
#include <Standard_Version.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <vector>

namespace exd::geometry
{

bool occt_available()
{
    return true;
}

std::string occt_version()
{
    return OCC_VERSION_STRING;
}

namespace
{

/// World-space bounding diagonal of a shape (drives tessellation deflection
/// as 0.1% of the diagonal — device-independent surface quality).
double shape_diagonal(const TopoDS_Shape& shape)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid())
        return 1.0;
    gp_Pnt mn, mx;
    box.Get(mn, mx);
    const gp_XYZ d = mx.XYZ() - mn.XYZ();
    return d.Modulus();
}

/// Tessellate one SOLID into a body-local MeshData part. Face triangulations
/// are copied with their location transform; normals are regenerated smoothly
/// (import normals are not meaningful for a faceted analytic).
MeshData tessellate_solid(const TopoDS_Shape& solid)
{
    const double deflection = std::max(shape_diagonal(solid) * 1e-3, 1e-4);
    BRepMesh_IncrementalMesh mesher(solid, deflection);
    mesher.Perform();

    MeshData out;
    out.topology = PrimitiveTopology::Triangles;
    TopExp_Explorer faceExp(solid, TopAbs_FACE);
    for (; faceExp.More(); faceExp.Next())
    {
        const TopoDS_Face face = TopoDS::Face(faceExp.Current());
        TopLoc_Location loc;
        const Handle(Poly_Triangulation)& tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull())
            continue;
        const gp_Trsf tr = loc.Transformation();

        const int nNodes = tri->NbNodes();
        const int nTris  = tri->NbTriangles();
        const uint32_t base = static_cast<uint32_t>(out.vertices.size());

        out.vertices.reserve(out.vertices.size() + static_cast<size_t>(nNodes));
        for (int i = 1; i <= nNodes; ++i)
        {
            const gp_Pnt& p = tri->Node(i).Transformed(tr);
            Vertex v;
            v.position = {static_cast<float>(p.X()), static_cast<float>(p.Y()),
                          static_cast<float>(p.Z())};
            out.vertices.push_back(v);
        }
        out.indices.reserve(out.indices.size() + static_cast<size_t>(nTris) * 3);
        for (int tc = 1; tc <= nTris; ++tc)
        {
            const Poly_Triangle& faceTri = tri->Triangle(tc);
            int n1, n2, n3;
            faceTri.Get(n1, n2, n3);
            out.indices.push_back(base + static_cast<uint32_t>(n1 - 1));
            out.indices.push_back(base + static_cast<uint32_t>(n2 - 1));
            out.indices.push_back(base + static_cast<uint32_t>(n3 - 1));
        }
    }
    out = recompute_normals(out, NormalMode::Smooth);
    out.bounds = compute_bounds(out.vertices);
    return out;
}

std::vector<TopoDS_Shape> collect_solids(const TopoDS_Shape& root)
{
    std::vector<TopoDS_Shape> out;
    TopExp_Explorer exp(root, TopAbs_SOLID);
    for (; exp.More(); exp.Next())
        out.push_back(exp.Current());
    if (out.empty())
        out.push_back(root);   // assemblies/shells fall back to the whole shape
    return out;
}

BrepImportResult import_shape(const TopoDS_Shape& shape)
{
    BrepImportResult result;
    int n = 0;
    for (const TopoDS_Shape& s : collect_solids(shape))
    {
        MeshData m = tessellate_solid(s);
        if (m.vertices.empty())
            continue;
        Part part = as_part("solid_" + std::to_string(++n), std::move(m));
        result.model.parts.push_back(std::move(part));
    }
    if (result.model.parts.empty())
    {
        result.error = "import produced no tessellated solids";
        return result;
    }
    result.model.name = "imported_cad";
    result.ok = true;
    return result;
}

/// Sew one part's triangle faces into a watertight solid (BRepBuilderAPI).
/// Only parts passing geometry's own closed-manifold gate are exported.
TopoDS_Shape sew_mesh_to_solid(const MeshData& m, double sewTol)
{
    BRepBuilderAPI_Sewing sewer(sewTol);
    const size_t triCount = m.indices.size() / 3;
    for (size_t t = 0; t < triCount; ++t)
    {
        const math::Vec3f& pa = m.vertices[m.indices[3 * t + 0]].position;
        const math::Vec3f& pb = m.vertices[m.indices[3 * t + 1]].position;
        const math::Vec3f& pc = m.vertices[m.indices[3 * t + 2]].position;
        BRepBuilderAPI_MakePolygon poly;
        poly.Add(gp_Pnt(pa.x, pa.y, pa.z));
        poly.Add(gp_Pnt(pb.x, pb.y, pb.z));
        poly.Add(gp_Pnt(pc.x, pc.y, pc.z));
        poly.Close();
        if (!poly.IsDone())
            continue;
        BRepBuilderAPI_MakeFace mkFace(poly.Wire());
        mkFace.Build();
        if (!mkFace.IsDone())
            continue;   // degenerate triangle
        sewer.Add(mkFace.Face());
    }
    sewer.Perform();
    return sewer.SewedShape();
}

} // namespace

BrepImportResult import_brep(const std::string& path, BrepImportFormat format)
{
    if (format == BrepImportFormat::Step)
    {
        STEPControl_Reader reader;
        if (reader.ReadFile(path.c_str()) != IFSelect_RetDone)
            return {false, "STEPControl_Reader failed to read: " + path, {}};
        reader.TransferRoots();
        const TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull())
            return {false, "STEP import produced no shape: " + path, {}};
        return import_shape(shape);
    }
    if (format == BrepImportFormat::Iges)
    {
        IGESControl_Reader reader;
        if (reader.ReadFile(path.c_str()) != IFSelect_RetDone)
            return {false, "IGESControl_Reader failed to read: " + path, {}};
        reader.TransferRoots();
        const TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull())
            return {false, "IGES import produced no shape: " + path, {}};
        return import_shape(shape);
    }
    if (format == BrepImportFormat::Brep)
    {
        BRep_Builder builder;
        TopoDS_Shape shape;
        if (!BRepTools::Read(shape, path.c_str(), builder))
            return {false, "BRepTools::Read failed: " + path, {}};
        return import_shape(shape);
    }
    return {false, "unsupported import format", {}};
}

BrepImportResult import_brep_file(const std::string& path)
{
    const std::string l = [&]{
        std::string out = path;
        for (char& c : out)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return out;
    }();
    if (l.find(".step") != std::string::npos || l.find(".stp") != std::string::npos)
        return import_brep(path, BrepImportFormat::Step);
    if (l.find(".iges") != std::string::npos || l.find(".igs") != std::string::npos)
        return import_brep(path, BrepImportFormat::Iges);
    return import_brep(path, BrepImportFormat::Brep);
}

bool export_brep_step(const CADModel& model, const std::string& path, std::string& error)
{
    STEPControl_Writer writer;
    int transferred = 0;
    for (const Part& part : model.parts)
    {
        const MeshData& m = part.mesh;
        if (m.topology != PrimitiveTopology::Triangles || m.vertices.empty())
            continue;
        const Bounds b = compute_bounds(m.vertices);
        const float diag = (b.max - b.min).length();
        const float posEps = diag > 0.0f ? std::max(diag * 1e-7f, 1e-9f) : 1e-6f;
        if (!closed_manifold_gate(m, posEps))
            continue;   // only watertight solids become valid BREP solids
        const TopoDS_Shape solid = sew_mesh_to_solid(m, posEps);
        if (solid.IsNull())
            continue;
        if (writer.Transfer(solid, STEPControl_AsIs) != IFSelect_RetDone)
        {
            error = "STEPControl_Writer::Transfer failed for part '" + part.name + "'";
            return false;
        }
        ++transferred;
    }
    if (transferred == 0)
    {
        error = "no watertight parts to export";
        return false;
    }
    if (writer.Write(path.c_str()) != IFSelect_RetDone)
    {
        error = "STEPControl_Writer::Write failed: " + path;
        return false;
    }
    return true;
}

bool export_brep_step(const CADModel& model, const std::string& path)
{
    std::string err;
    return export_brep_step(model, path, err);
}

} // namespace exd::geometry

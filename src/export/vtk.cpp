#include <exd/geometry/export.hpp>
#include "cae_common.hpp"

#include <cstdio>
#include <sstream>

namespace exd::geometry
{
namespace
{

std::string fmt_f(float v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.7g", static_cast<double>(v));
    return buf;
}

std::string fmt_p(const math::Vec3f& v)
{
    return fmt_f(v.x) + " " + fmt_f(v.y) + " " + fmt_f(v.z);
}

/// Shared compact render path: writes the three data sections both writers
/// need, so legacy VTK and XML VTU stay structurally identical.
void point_lines(std::ostringstream& os, const FlatMesh& fm)
{
    for (const math::Vec3f& p : fm.points)
        os << fmt_p(p) << "\n";
}

void cell_lines(std::ostringstream& os, const FlatMesh& fm)
{
    for (const auto& cell : fm.cells)
        os << "3 " << cell.a << " " << cell.b << " " << cell.c << "\n";
}

} // namespace

/// VTK legacy unstructured grid (ASCII). Scalars PartID (0-based part index)
/// and PatchID (1-based global patch id; 0 = no patch).
std::string to_vtk(const CADModel& model)
{
    const FlatMesh fm = flatten_cad(model);
    const std::string title = model.name.empty() ? "model" : model.name;

    std::ostringstream os;
    os << "# vtk DataFile Version 3.0\n";
    os << title << "\n";
    os << "ASCII\n";
    os << "DATASET UNSTRUCTURED_GRID\n\n";

    os << "POINTS " << fm.points.size() << " float\n";
    point_lines(os, fm);
    os << "\n";

    os << "CELLS " << fm.cells.size() << " " << (fm.cells.size() * 4) << "\n";
    cell_lines(os, fm);
    os << "\n";

    os << "CELL_TYPES " << fm.cells.size() << "\n";
    for (size_t c = 0; c < fm.cells.size(); ++c)
        os << "5\n";   // VTK_TRIANGLE
    os << "\n";

    os << "CELL_DATA " << fm.cells.size() << "\n";
    os << "SCALARS PartID int 1\nLOOKUP_TABLE default\n";
    for (const auto& cell : fm.cells)
        os << cell.part << "\n";
    os << "\n";
    os << "SCALARS PatchID int 1\nLOOKUP_TABLE default\n";
    for (const auto& cell : fm.cells)
        os << cell.patch << "\n";
    os << "\n";
    return os.str();
}

/// VTK XML unstructured grid (.vtu), ascii data arrays — ParaView-native.
std::string to_vtu(const CADModel& model)
{
    const FlatMesh fm = flatten_cad(model);
    const std::string title = model.name.empty() ? "model" : model.name;

    std::ostringstream os;
    os << "<?xml version=\"1.0\"?>\n";
    os << "<VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\" header_type=\"UInt64\">\n";
    os << "  <UnstructuredGrid>\n";
    os << "    <FieldData>\n";
    os << "      <DataArray type=\"String\" Name=\"Title\" format=\"ascii\">" << title << "</DataArray>\n";
    os << "    </FieldData>\n";
    os << "    <Piece NumberOfPoints=\"" << fm.points.size()
       << "\" NumberOfCells=\"" << fm.cells.size() << "\">\n";
    os << "      <Points>\n";
    os << "        <DataArray type=\"Float32\" Name=\"Points\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    point_lines(os, fm);
    os << "        </DataArray>\n";
    os << "      </Points>\n";
    os << "      <Cells>\n";
    os << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
    for (const auto& cell : fm.cells)
        os << cell.a << " " << cell.b << " " << cell.c << "\n";
    os << "        </DataArray>\n";
    os << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
    for (size_t c = 1; c <= fm.cells.size(); ++c)
        os << (c * 3) << "\n";
    os << "        </DataArray>\n";
    os << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n";
    for (size_t c = 0; c < fm.cells.size(); ++c)
        os << "5\n";   // VTK_TRIANGLE
    os << "        </DataArray>\n";
    os << "      </Cells>\n";
    os << "      <CellData Scalars=\"PartID\">\n";
    os << "        <DataArray type=\"Int32\" Name=\"PartID\" format=\"ascii\">\n";
    for (const auto& cell : fm.cells)
        os << cell.part << "\n";
    os << "        </DataArray>\n";
    os << "        <DataArray type=\"Int32\" Name=\"PatchID\" format=\"ascii\">\n";
    for (const auto& cell : fm.cells)
        os << cell.patch << "\n";
    os << "        </DataArray>\n";
    os << "      </CellData>\n";
    os << "    </Piece>\n";
    os << "  </UnstructuredGrid>\n";
    os << "</VTKFile>\n";
    return os.str();
}

} // namespace exd::geometry

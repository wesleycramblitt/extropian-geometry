// Stub backend — compiled when EXD_GEOMETRY_ENABLE_OCCT is off (the default).
// Keeps the analytic-BREP API present (and the WASM-clean build story intact)
// while reporting the missing backend with an actionable message.

#include <exd/geometry/import_brep.hpp>

namespace exd::geometry
{

bool occt_available()
{
    return false;
}

std::string occt_version()
{
    return "none (stub build)";
}

namespace
{

BrepImportResult unavailable(const std::string& op)
{
    BrepImportResult r;
    r.ok = false;
    r.error = "analytic BREP (" + op + ") requires the OCCT backend — rebuild "
              "with -DEXD_GEOMETRY_ENABLE_OCCT=ON and OpenCASCADE installed "
              "(e.g. sudo apt install libopencascade-dev).";
    return r;
}

} // namespace

BrepImportResult import_brep(const std::string& /*path*/, BrepImportFormat /*format*/)
{
    return unavailable("import_brep");
}

BrepImportResult import_brep_file(const std::string& /*path*/)
{
    return unavailable("import_brep_file");
}

bool export_brep_step(const CADModel& /*model*/, const std::string& /*path*/,
                      std::string& error)
{
    error = "analytic STEP export requires the OCCT backend — rebuild with "
            "-DEXD_GEOMETRY_ENABLE_OCCT=ON and OpenCASCADE installed "
            "(e.g. sudo apt install libopencascade-dev).";
    return false;
}

bool export_brep_step(const CADModel& /*model*/, const std::string& /*path*/)
{
    std::string err;
    return export_brep_step(CADModel{}, "", err);
}

} // namespace exd::geometry

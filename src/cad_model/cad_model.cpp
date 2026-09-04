#include <exd/geometry/cad_model.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <set>
#include <vector>

namespace exd::geometry
{

// ── DataTable ───────────────────────────────────────────────────────────────

float DataTable::sample(float xi) const
{
    if (x.empty() || y.size() != x.size())
        return 0.0f;
    if (x.size() == 1)
        return y[0];
    if (xi <= x.front())
        return y.front();
    if (xi >= x.back())
        return y.back();
    for (size_t i = 1; i < x.size(); ++i)
    {
        if (xi <= x[i])
        {
            const float dx = x[i] - x[i - 1];
            const float t  = dx > 0.0f ? (xi - x[i - 1]) / dx : 0.0f;
            return y[i - 1] * (1.0f - t) + y[i] * t;
        }
    }
    return y.back();
}

// ── MaterialDB ──────────────────────────────────────────────────────────────

const Material* MaterialDB::find(std::string_view name) const
{
    for (const Material& m : materials_)
        if (m.name == name)
            return &m;
    return nullptr;
}

bool MaterialDB::register_(const Material& m)
{
    if (m.name.empty() || find(m.name) != nullptr)
        return false;
    materials_.push_back(m);
    return true;
}

std::vector<std::string> MaterialDB::names() const
{
    std::vector<std::string> out;
    out.reserve(materials_.size());
    for (const Material& m : materials_)
        out.push_back(m.name);
    return out;
}

MaterialDB MaterialDB::defaults()
{
    MaterialDB db;

    Material steel;
    steel.name = "steel-1045";
    steel.density = 7850.0f;
    steel.youngs_modulus = 205.0e9f;
    steel.poisson = 0.29f;
    steel.yield_stress = 310.0e6f;
    steel.thermal_conductivity = 50.5f;
    steel.specific_heat = 500.0f;
    steel.cte = 11.0e-6f;
    db.register_(steel);

    Material alum;
    alum.name = "aluminum-6061";
    alum.density = 2700.0f;
    alum.youngs_modulus = 68.9e9f;
    alum.poisson = 0.33f;
    alum.yield_stress = 241.0e6f;
    alum.thermal_conductivity = 167.0f;
    alum.specific_heat = 896.0f;
    alum.cte = 23.6e-6f;
    db.register_(alum);

    Material stainless;
    stainless.name = "stainless-304";
    stainless.density = 8000.0f;
    stainless.youngs_modulus = 193.0e9f;
    stainless.poisson = 0.29f;
    stainless.yield_stress = 215.0e6f;
    stainless.thermal_conductivity = 16.2f;
    stainless.specific_heat = 500.0f;
    stainless.cte = 17.2e-6f;
    db.register_(stainless);

    Material copper;
    copper.name = "copper";
    copper.density = 8960.0f;
    copper.youngs_modulus = 110.0e9f;
    copper.poisson = 0.34f;
    copper.yield_stress = 70.0e6f;
    copper.thermal_conductivity = 385.0f;
    copper.specific_heat = 385.0f;
    copper.cte = 16.5e-6f;
    db.register_(copper);

    Material nylon;
    nylon.name = "nylon-66";
    nylon.density = 1140.0f;
    nylon.youngs_modulus = 2.8e9f;
    nylon.poisson = 0.39f;
    nylon.yield_stress = 70.0e6f;
    nylon.thermal_conductivity = 0.25f;
    nylon.specific_heat = 1700.0f;
    nylon.cte = 80.0e-6f;
    db.register_(nylon);

    Material rubber;
    rubber.name = "rubber";
    rubber.density = 1100.0f;
    rubber.youngs_modulus = 0.01e9f;
    rubber.poisson = 0.49f;
    rubber.yield_stress = 5.0e6f;
    rubber.thermal_conductivity = 0.13f;
    rubber.specific_heat = 1800.0f;
    rubber.cte = 150.0e-6f;
    db.register_(rubber);

    Material water;
    water.name = "water";
    water.density = 997.0f;
    water.youngs_modulus = 2.2e9f;
    water.poisson = 0.5f;
    water.yield_stress = 0.0f;
    water.thermal_conductivity = 0.6f;
    water.specific_heat = 4184.0f;
    water.cte = 0.0f;
    db.register_(water);

    return db;
}

// ── CADModel helpers ────────────────────────────────────────────────────────

const Material* CADModel::material_for(const Part& p) const
{
    if (p.meta.material.empty())
        return nullptr;
    return materials.find(p.meta.material);
}

const Material* CADModel::material_for(const FaceRef& ref) const
{
    for (const Part& p : parts)
    {
        if (p.name != ref.part)
            continue;
        for (const Patch& pa : p.patches)
        {
            if (pa.name != ref.patch)
                continue;
            if (!pa.material.empty())
                return materials.find(pa.material);
            if (!p.meta.material.empty())
                return materials.find(p.meta.material);
            return nullptr;
        }
        return nullptr;
    }
    return nullptr;
}

bool CADModel::validate(std::vector<std::string>& errors) const
{
    errors.clear();
    auto err = [&](const std::string& msg) { errors.push_back(msg); };

    // ── Part-level integrity ────────────────────────────────────────────
    std::set<std::string> partNames;
    for (const Part& p : parts)
    {
        if (!partNames.insert(p.name).second)
            err("part name '" + p.name + "' is duplicated");
        if (p.mesh.vertices.empty() || p.mesh.indices.empty())
        {
            err("part '" + p.name + "' has an empty mesh");
            continue;
        }
        const uint32_t triCount = static_cast<uint32_t>(p.mesh.indices.size() / 3);
        std::set<std::string> patchNames;
        for (const Patch& pa : p.patches)
        {
            if (!patchNames.insert(pa.name).second)
                err("part '" + p.name + "' has duplicate patch '" + pa.name + "'");
            for (uint32_t f : pa.faces)
                if (f >= triCount)
                {
                    err("part '" + p.name + "' patch '" + pa.name + "' face " +
                        std::to_string(f) + " out of range (triangles: " +
                        std::to_string(triCount) + ")");
                    break;
                }
            if (!pa.material.empty() && !materials.find(pa.material))
                err("part '" + p.name + "' patch '" + pa.name + "' material '" +
                    pa.material + "' not in MaterialDB");
        }
        if (!p.meta.material.empty() && !materials.find(p.meta.material))
            err("part '" + p.name + "' material '" + p.meta.material + "' not in MaterialDB");
    }

    // Resolve a patch reference to nullptr-or-existing.
    auto findPatchRef = [&](const FaceRef& ref) -> const Patch* {
        for (const Part& p : parts)
        {
            if (p.name != ref.part)
                continue;
            for (const Patch& pa : p.patches)
                if (pa.name == ref.patch)
                    return &pa;
        }
        return nullptr;
    };
    auto findPartName = [&](const std::string& n) -> bool {
        return partNames.count(n) != 0;
    };

    // ── Regions ─────────────────────────────────────────────────────────
    std::set<std::string> regionNames;
    for (const Region& r : regions)
    {
        if (!regionNames.insert(r.name).second)
        {
            err("region name '" + r.name + "' is duplicated");
            continue;
        }
        if (r.boundary.empty())
            err("region '" + r.name + "' has no boundary faces");
        for (const FaceRef& ref : r.boundary)
            if (!findPatchRef(ref))
                err("region '" + r.name + "' references missing face " +
                    ref.part + "/" + ref.patch);
        if (r.active.empty())
            err("region '" + r.name + "' has no active physics");
        for (const auto& [ph, mat] : r.material)
            if (!materials.find(mat))
                err("region '" + r.name + "' material '" + mat + "' not in MaterialDB");
    }

    // ── Domains ─────────────────────────────────────────────────────────
    std::set<std::string> domainNames;
    for (const Domain& d : domains)
    {
        if (!domainNames.insert(d.name).second)
        {
            err("domain name '" + d.name + "' is duplicated");
            continue;
        }
        for (const std::string& rn : d.regions)
            if (!regionNames.count(rn))
                err("domain '" + d.name + "' references missing region '" + rn + "'");
    }

    // ── Interfaces ──────────────────────────────────────────────────────
    for (const Interface& i : interfaces)
    {
        if (!i.surface_a.part.empty() && !findPatchRef(i.surface_a))
            err("interface '" + i.name + "' references missing face " +
                i.surface_a.part + "/" + i.surface_a.patch);
        if (!i.surface_b.part.empty() && !findPatchRef(i.surface_b))
            err("interface '" + i.name + "' references missing face " +
                i.surface_b.part + "/" + i.surface_b.patch);
        if (!i.domain_a.empty() && !domainNames.count(i.domain_a))
            err("interface '" + i.name + "' references missing domain '" + i.domain_a + "'");
        if (!i.domain_b.empty() && !domainNames.count(i.domain_b))
            err("interface '" + i.name + "' references missing domain '" + i.domain_b + "'");
    }

    // ── BCs & body loads (physics-tagged records) ───────────────────────
    for (const BoundaryCondition& bc : bcs)
    {
        if (!findPartName(bc.part))
            err("boundary condition '" + bc.name + "' references missing part '" + bc.part + "'");
        else if (!findPatchRef({bc.part, bc.patch}))
            err("boundary condition '" + bc.name + "' references missing face " +
                bc.part + "/" + bc.patch);
    }
    for (const BodyLoad& bl : body_loads)
        if (!regionNames.count(bl.region))
            err("body load '" + bl.name + "' references missing region '" + bl.region + "'");

    // ── Symmetry / cyclic / frames ──────────────────────────────────────
    for (const SymmetryPlane& sp : symmetry_planes)
        if (!findPartName(sp.part) || !findPatchRef({sp.part, sp.patch}))
            err("symmetry plane '" + sp.name + "' references missing face " +
                sp.part + "/" + sp.patch);
    for (const CyclicSector& cs : cyclic_sectors)
        if (!findPartName(cs.part) || !findPatchRef({cs.part, cs.patch}))
            err("cyclic sector '" + cs.name + "' references missing face " +
                cs.part + "/" + cs.patch);
    for (const RotatingReferenceFrame& rf : frames)
        if (!findPartName(rf.part))
            err("reference frame '" + rf.name + "' references missing part '" + rf.part + "'");

    // ── Mechanism (composes validate_mechanism; it clears its own vector) ─
    if (!mechanism.joints.empty() || !mechanism.couplings.empty())
    {
        std::vector<std::string> mechErrors;
        if (!validate_mechanism(mechanism, mechErrors))
            for (const std::string& e : mechErrors)
                errors.push_back(e);
    }

    return errors.empty();
}

// ── Construction ────────────────────────────────────────────────────────────

CADModel make_cad_model(std::string name,
                        std::span<const Part> parts,
                        const Mechanism& mech,
                        UnitSystem units)
{
    CADModel model;
    model.name     = std::move(name);
    model.units    = units;
    model.parts.assign(parts.begin(), parts.end());
    model.mechanism = mech;

    bool first = true;
    for (const Part& p : model.parts)
    {
        if (p.mesh.vertices.empty())
            continue;
        const Bounds b = compute_bounds(p.mesh.vertices);
        if (first)
        {
            model.bounds = b;
            first = false;
        }
        else
        {
            model.bounds.min.x = std::min(model.bounds.min.x, b.min.x);
            model.bounds.min.y = std::min(model.bounds.min.y, b.min.y);
            model.bounds.min.z = std::min(model.bounds.min.z, b.min.z);
            model.bounds.max.x = std::max(model.bounds.max.x, b.max.x);
            model.bounds.max.y = std::max(model.bounds.max.y, b.max.y);
            model.bounds.max.z = std::max(model.bounds.max.z, b.max.z);
        }
    }
    return model;
}

} // namespace exd::geometry

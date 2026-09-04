#pragma once

#include <exd/geometry/types.hpp>
#include <exd/geometry/part.hpp>
#include <exd/geometry/mechanism.hpp>
#include <exd/math/vec3.hpp>

#include <map>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace exd::geometry
{

// ═══════════════════════════════════════════════════════════════════════════
//  CADModel — the unified, format-agnostic model IR (D11).
//
//  One object per part/machine: body-local parts with patches/materials,
//  region/domain/interface topology for multiphysics, physics-tagged BCs and
//  body loads, symmetry/cyclic/reference-frame metadata, and the Mechanism.
//  Every file format is an ADAPTER over this object (to_obj/to_stl/to_msh/
//  to_vtk/to_step/to_mjcf/to_urdf); every importer BUILDS one. The model
//  declares physics; adapters and solvers configure (docs/cad-model.md).
//  Deterministic: same construction calls → identical field values.
// ═══════════════════════════════════════════════════════════════════════════

/// Authoring unit system. Exporters normalize to SI.
enum class UnitSystem
{
    SI,      // m, kg, s, N, Pa, W (recommended)
    MMKgS,   // mm, kg, s
    US       // in, lb, s (imperial)
};

/// Governing-equation tag for BCs, loads, and region activity.
enum class Physics
{
    Structural,
    Thermal,
    Fluid,
    Electric,
    Magnetic,
    Acoustic,
    Radiation
};

// ── Data tables (piecewise-linear, deterministic) ───────────────────────────

/// Piecewise-linear data (independent variable x ascending). Used for
/// transient BC magnitudes and state-dependent material properties.
struct DataTable
{
    std::vector<float> x;   // independent variable (t, T, strain…) — non-decreasing
    std::vector<float> y;   // dependent values

    bool empty() const { return x.empty(); }
    float sample(float xi) const;   // clamped-end linear interpolation
};

// ── Materials ───────────────────────────────────────────────────────────────

/// Isotropic material property record (SI base units). Optional state-dependent
/// tables extend the constant properties.
struct Material
{
    std::string name;

    // structural
    float density        = 7850.0f;    // kg/m³
    float youngs_modulus = 205.0e9f;   // Pa
    float poisson        = 0.29f;      // –
    float yield_stress   = 310.0e6f;   // Pa

    // thermal
    float thermal_conductivity = 50.5f;   // W/(m·K)
    float specific_heat        = 500.0f;  // J/(kg·K)
    float cte                  = 11.0e-6f; // 1/K (linear CTE)

    // state-dependent (optional)
    DataTable k_of_T;        // x = temperature [K] → conductivity
    DataTable stress_strain; // x = strain → true stress [Pa]
};

/// Deterministic named-material registry (D14). Curated defaults supplied by
/// `defaults()`; `register_` rejects duplicate names.
class MaterialDB
{
public:
    const Material* find(std::string_view name) const;
    bool register_(const Material& m);     // true on success; false if name exists
    size_t size() const { return materials_.size(); }
    std::vector<std::string> names() const;

    /// Curated in-repo table: steel-1045, aluminum-6061, stainless-304,
    /// copper, nylon-66, rubber, water.
    static MaterialDB defaults();

private:
    std::vector<Material> materials_;   // insertion order (deterministic)
};

// ── Multiphysics topology ───────────────────────────────────────────────────

/// A named face set in a part — the unit every BC/load/interface references.
struct FaceRef
{
    std::string part;
    std::string patch;
};

/// A closed volume region — the physics unit. `boundary` faces must bound a
/// closed manifold; the pre-processor volume-meshes the enclosed solid using
/// the patch `mesh_size` hints and region materials.
struct Region
{
    std::string name;
    std::vector<FaceRef> boundary;              // closed manifold face set
    std::vector<Physics> active;                // equations active in this region
    std::map<Physics, std::string> material;    // physics → MaterialDB name
};

/// A physics domain: a named collection of regions sharing a model.
struct Domain
{
    std::string name;
    std::vector<std::string> regions;
};

/// Coupling between two domains at faces (multiphysics connective tissue).
enum class InterfaceKind
{
    FSI,              // fluid ↔ structure (wet surface)
    ThermalContact,   // thermal resistance across a contact
    ElectroThermal,   // Joule heating coupling
    MRFStage,         // rotor/stator mixed-plane / frozen-rotor interface
    Sliding,          // sliding mesh interface
    Contact,          // mechanical contact pair
    Tie               // bonded/tied faces
};

struct Interface
{
    std::string name;
    InterfaceKind kind = InterfaceKind::Contact;
    std::string domain_a, domain_b;
    FaceRef surface_a, surface_b;   // faces on each side (may reference the same geometry)
    float friction = 0.0f;          // for Contact
    bool  tie      = false;
};

// ── Boundary conditions & body loads ────────────────────────────────────────

/// BC kind per physics. A face may carry one BC per Physics: e.g. Fixed
/// (Structural) + Adiabatic (Thermal) + wall (Fluid) on the same patch.
enum class BCKind
{
    Fixed,                // structural: fully constrained face
    Displacement,         // structural: prescribed displacement vector
    Force,                // structural: applied load
    Torque,               // structural: applied moment
    Pressure,             // fluid/structural: surface pressure magnitude
    SpecifiedTemperature, // thermal: prescribed temperature [K]
    HeatFlux,             // thermal: applied flux [W/m²]
    Convection,           // thermal: h coefficient + ambient temperature
    FlowRate,             // fluid: prescribed mass/volume flow
    OpenFlow,             // fluid: zero-gradient / far-field open face
    Adiabatic             // thermal: zero heat flux
};

struct BoundaryCondition
{
    std::string name;
    Physics     physics = Physics::Structural;
    BCKind      kind    = BCKind::Fixed;
    std::string part;
    std::string patch;
    math::Vec3f value{0.0f, 0.0f, 0.0f};   // vector load / displacement / axis
    float       magnitude = 0.0f;          // scalar: pressure, temperature, flux…
    DataTable   magnitude_over_time;       // transient (empty = steady)
    std::string document;                  // description / source
};

enum class BodyLoadKind
{
    Gravity,
    Centrifugal,
    HeatGeneration,
    BodyForce,
    MagneticForce
};

struct BodyLoad
{
    std::string name;
    Physics     physics = Physics::Structural;
    BodyLoadKind kind    = BodyLoadKind::Gravity;
    std::string region;                    // CADModel::regions name
    math::Vec3f vector{0.0f, 0.0f, 0.0f};
    float       magnitude = 0.0f;
    DataTable   magnitude_over_time;
    std::string document;
};

// ── Symmetry, cyclic & reference frames ─────────────────────────────────────

struct SymmetryPlane
{
    std::string name;
    std::string part;
    std::string patch;                     // the symmetry face set
    math::Vec3f normal{0.0f, 1.0f, 0.0f};
};

struct CyclicSector
{
    std::string name;
    std::string part;
    std::string patch;                     // sector boundary face set (periodic pair)
    int         instances = 1;             // full count around the axis
    float       sector_angle_deg = 0.0f;
    math::Vec3f axis{0.0f, 0.0f, 1.0f};
};

struct RotatingReferenceFrame
{
    std::string name;
    std::string part;                      // part in the rotating frame
    float       speed_rpm = 0.0f;
    math::Vec3f axis{0.0f, 0.0f, 1.0f};
};

// ── The unified model ───────────────────────────────────────────────────────

struct CADModel
{
    std::string name;
    UnitSystem units = UnitSystem::SI;

    std::vector<Part> parts;                     // body-local frames (exporter contract)
    std::vector<Region> regions;
    std::vector<Domain> domains;
    std::vector<Interface> interfaces;
    std::vector<BoundaryCondition> bcs;
    std::vector<BodyLoad> body_loads;
    std::vector<SymmetryPlane> symmetry_planes;
    std::vector<CyclicSector> cyclic_sectors;
    std::vector<RotatingReferenceFrame> frames;
    Mechanism mechanism;                         // may be empty (single-part models)
    MaterialDB materials;
    Bounds bounds;                               // union over all parts

    /// Resolve `Part.meta.material` against this model's MaterialDB.
    /// Returns nullptr for empty or unresolvable names.
    const Material* material_for(const Part& p) const;

    /// Resolve material for a face set: patch override → part → nullptr.
    const Material* material_for(const FaceRef& ref) const;

    /// CAE-readiness gate (D16): reference integrity + material resolution.
    /// Deterministic; fills actionable error strings.
    bool validate(std::vector<std::string>& errors) const;
};

/// Build a model from body-local parts + optional mechanism. Computes bounds
/// (union over parts). Deterministic; copies inputs.
CADModel make_cad_model(std::string name,
                        std::span<const Part> parts,
                        const Mechanism& mech = {},
                        UnitSystem units = UnitSystem::SI);

} // namespace exd::geometry

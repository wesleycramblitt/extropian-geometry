# CADModel — Unified Model IR & CAE Format Roadmap

> **2026-09.** Companion to `plan.md` (kernel), `recipes.md` (domain recipes),
> and `mechanisms.md` (connectivity). This is the design record for the
> **unified model**: one in-memory object that carries geometry, patches,
> materials, multiphysics topology, loads, and mechanism constraints — and
> the adapter/import layer that turns it into and out of CAD/CAE file formats.

## 1. The core principle

**Generation is not format selection.** We never choose an output format at
request time. There are two boundaries, answered separately:

1. **Generation** — *what to compute* (a descriptor in, one object out).
2. **Serialization / import** — *which adapter* (a file-format function).

```
  descriptor ──generate──▶ CADModel (unified in-memory IR)
  file ────────import────▶ CADModel

  CADModel ──adapters──▶ MeshData │ STEP │ STL │ Gmsh-MSG │ VTK │ MJCF │ URDF │ OBJ
```

- **CADModel is the unified format** — one object per part or machine,
  format-agnostic, deterministic, pure CPU/WASM-clean.
- **Every file format is an adapter (projection) over it.** Each projection is
  lossy: MJCF drops CAE patch semantics and loads; STEP drops compliance and
  actuators; `MeshData` keeps only the vertex/index arrays.
- **Every importer builds a CADModel** — import is the inverse adapter.

### 1a. Two request tiers

| Tier | API | Returns | Use |
|---|---|---|---|
| Fast path (simple shapes) | `generate_*_mesh(desc)` | `MeshData` | primitives, renderers — unchanged, no CADModel built |
| Unified path (recipes/machines) | `generate_*_cadmodel(desc)` | `CADModel` | everything with patches, materials, mechanism, physics |

`MeshData` is a **lossy projection** of `CADModel`: a renderer or legacy caller
can take `CADModel → MeshData` at the boundary, but cannot reconstruct patches
or materials from vertices. Keep the model when downstream needs more than
triangles.

## 2. Position in the architecture

```
                    ┌──────────────────────────────────────────────┐
  recipes/builders ─▶│  CADModel                                   │
                    │  parts·patches·materials·regions·domains     │
                    │  interfaces·bcs·body_loads·symmetry·frames   │
                    │  mechanism·units                              │
                    └──────────┬─────────────────────┬─────────────┘
                               │                     │
                   adapters    │                     │  importers (in-repo)
                   OBJ/STL/    │                     │  STL · OBJ · Gmsh MSH
                   MSH/VTK/VTU │                     │  (pure C++, WASM-clean)
                   + faceted   │                     │
                   STEP        │                     │
                               └─────────────────────┘
```

- geometry (<- this repo) owns the IR, the adapters, and the mesh importers
  (STL/OBJ/Gmsh). Wasm-clean, no heavy dependencies.
- **Analytic BREP (STEP/IGES/BREP) is out of scope here.** The library is a
  mesh import/export library; analytic CAD formats need an external CAD
  kernel (e.g. OpenCASCADE) and are deliberately not wired in (D17/D18).
- **The model declares physics; adapters/solvers configure.** We do not model
  solver configuration (element order, tolerances, timestep), result fields
  (VTU covers output), or material-property solve engines.

## 3. Frame & units contract

- **Frame contract is unchanged** (mechanisms.md §2): parts are **body-local**;
  their local origin IS their joint anchor; exporters emit parent-local frames.
- **Units:** `CADModel.units` declares the authoring system
  (`SI` / `MMKgS` / `US`). **Exporters normalize to SI.** All physics fields
  below are authored in the declared system and documented per-record.

## 4. Schema

### 4a. Patch extensions (in `part.hpp`)

```cpp
enum class PatchSemantic { Unspecified, Wall, Inlet, Outlet, Symmetry,
                           Fixed, Sliding, Interface, Free };

struct Patch {            // existing fields preserved
    std::string name;
    std::vector<uint32_t> faces;
    // Phase A additions (CAE/multiphysics):
    PatchSemantic semantic = PatchSemantic::Unspecified;  // CAE role
    float  mesh_size = 0.0f;      // target cell/edge size hint [m]; 0 = solver default
    std::string material;         // per-patch override; empty = part's material
};
```

Legacy construction-relative names (`wall`, `cap_start`, `+x`…) remain valid;
`semantic` is additive and defaults to `Unspecified`, so every existing
initializer and `make_patch_range()` keeps working unmodified.

### 4b. Physics, data, materials (`cad_model.hpp`)

```cpp
enum class Physics { Structural, Thermal, Fluid,
                     Electric, Magnetic, Acoustic, Radiation };

enum class UnitSystem { SI, MMKgS, US };

struct DataTable {                       // piecewise-linear, x ascending
    std::vector<float> x, y;
    bool empty() const;
    float sample(float xi) const;        // clamped-end linear interpolation
};

struct Material {                        // isotropic property record
    std::string name;
    float density=7850, youngs_modulus=205e9, poisson=0.29, yield_stress=310e6;
    float thermal_conductivity=50.5, specific_heat=500, cte=11e-6;
    DataTable k_of_T;                    // temperature-dependent conductivity
    DataTable stress_strain;             // → true stress
};

class MaterialDB {                       // deterministic registry
    const Material* find(std::string_view) const;
    bool register_(const Material&);     // false on duplicate name
    static MaterialDB defaults();        // curated: steel-1045, alum-6061,
};                                       // stainless-304, copper, nylon-66,
                                         // rubber, water
```

### 4c. Multiphysics topology (`cad_model.hpp`)

```cpp
struct FaceRef { std::string part; std::string patch; };

struct Region {                          // closed volume = physics unit
    std::string name;
    std::vector<FaceRef> boundary;       // faces bounding a closed manifold
    std::vector<Physics> active;         // equations active here
    std::map<Physics, std::string> material; // physics → MaterialDB name
};

struct Domain { std::string name; std::vector<std::string> regions; };

enum class InterfaceKind { FSI, ThermalContact, ElectroThermal,
                           MRFStage, Sliding, Contact, Tie };

struct Interface {                       // coupling between domains
    std::string name;
    InterfaceKind kind;
    std::string domain_a, domain_b;
    FaceRef surface_a, surface_b;
    float friction = 0.0f;               // for Contact
    bool  tie      = false;
};
```

Because a pre-processor volume-meshes enclosed surfaces, a `Region` bounded
by a closed patch set **is** the volume mesh request: the pre-processor fills
the enclosed solid per the `mesh_size` hints and region material.

### 4d. BCs, body loads (`cad_model.hpp`)

```cpp
enum class BCKind { Fixed, Displacement, Force, Torque, Pressure,
                    SpecifiedTemperature, HeatFlux, Convection,
                    FlowRate, OpenFlow, Adiabatic };

struct BoundaryCondition {
    std::string name;
    Physics   physics = Physics::Structural;  // which equation this applies to
    BCKind    kind    = BCKind::Fixed;
    std::string part, patch;                   // one patch of one part
    math::Vec3f value{0,0,0};
    float   magnitude = 0.0f;
    DataTable magnitude_over_time;             // transient; empty = steady
    std::string document;
};

enum class BodyLoadKind { Gravity, Centrifugal, HeatGeneration,
                          BodyForce, MagneticForce };

struct BodyLoad {
    std::string name;
    Physics physics = Physics::Structural;
    BodyLoadKind kind = BodyLoadKind::Gravity;
    std::string region;
    math::Vec3f vector{0,0,0};
    float magnitude = 0.0f;
    DataTable magnitude_over_time;
    std::string document;
};
```

A face may legally carry **one BC per physics**: fixed support (Structural),
adiabatic wall (Thermal), no-slip wall (Fluid) on the same patch — this is
what real coupled analyses look like.

### 4e. Symmetry, cyclic, reference frames (`cad_model.hpp`)

```cpp
struct SymmetryPlane           { std::string name; std::string part;
                                 std::string patch; math::Vec3f normal{0,1,0}; };
struct CyclicSector            { std::string name; std::string part;
                                 std::string patch; int instances=1;
                                 float sector_angle_deg=0; math::Vec3f axis{0,0,1}; };
struct RotatingReferenceFrame  { std::string name; std::string part;
                                 float speed_rpm=0; math::Vec3f axis{0,0,1}; };
```

These are first-class because the turbine/compressor/steam recipes already
produce rotating, symmetric machinery; cyclic symmetry, periodic pairs, and
rotor-stator interfaces (MRFStage) are the bread-and-butter of turbomachinery
CAE.

### 4f. The model (`cad_model.hpp`)

```cpp
struct CADModel {
    std::string name;
    UnitSystem units = UnitSystem::SI;
    std::vector<Part> parts;              // body-local (exporter frame)
    std::vector<Region> regions;
    std::vector<Domain> domains;
    std::vector<Interface> interfaces;
    std::vector<BoundaryCondition> bcs;
    std::vector<BodyLoad> body_loads;
    std::vector<SymmetryPlane> symmetry_planes;
    std::vector<CyclicSector> cyclic_sectors;
    std::vector<RotatingReferenceFrame> frames;
    Mechanism mechanism;                  // may be empty (single-part models)
    MaterialDB materials;
    Bounds bounds;                        // union over all parts

    const Material* material_for(const Part&) const;      // meta.material → DB
    const Material* material_for(const FaceRef&) const;   // patch override → part → DB
    bool validate(std::vector<std::string>& errors) const; // D16 gate
};

CADModel make_cad_model(std::string name, std::span<const Part> parts,
                        const Mechanism& mech = {}, UnitSystem units = SI);
```

## 5. Adapter mapping (planned, Phase B)

| Adapter | Scope | Projection notes | Status |
|---|---|---|---|
| `to_obj` | surface mesh | geometry only | ✅ (MeshData + CADModel) |
| `to_stl` | tessellation | ascii + binary; binary is the CAE default; per-solid header per part | ✅ Phase B |
| `to_msh` (Gmsh) | surface mesh | **PhysicalSurfaces = patches** 1:1; unpatched faces → part-level group; region volume groups deferred (no volume mesh) | ✅ Phase B |
| `to_vtk` / `to_vtu` | mesh + region data | CellData = PartID (0-based) + PatchID (1-based global enum) | ✅ Phase B |
| `to_step_faceted` | CAD | faceted solid B-rep (AP203/214) per watertight part — deterministic, imports anywhere; analytic BREP is deferred (D17) | ✅ Phase B |
| `to_mjcf` / `to_urdf` | physics | mechanism slice; contact = `meta.contact` parts; density from material resolution | ✅ (existing) |
| analytic STEP (via external CAD kernel) | CAD | deferred — not owned here | 🔜 future |

All adapters are deterministic writers; same model → byte-identical output.

## 6. Import (planned, Phase C/D)

| Class | Formats | Owner | Status |
|---|---|---|---|
| Tessellated / lightweight | STL, OBJ, Gmsh MSH (PLY, glTF, 3MF later) | **in-repo, pure C++, WASM-clean** | ✅ Phase C |
| Analytic / BREP | STEP, IGES, BREP | **not owned here** — requires an external kernel; deferred | 🔜 future |

Importers produce `CADModel`.
- `parse_stl` auto-detects ascii/binary (multi-solid binary supported);
  triangle soup, no welding.
- `parse_obj` keeps indexed topology + optional normals; quads fan-triangulated.
- `import_msh` maps PhysicalSurfaces → Patches 1:1 ("part.patch"), elementary
  regions → Parts, and round-trips `to_msh` exactly.
- STEP/IGES/BREP (analytic) are deferred — importing them would require an
  external CAD kernel, which is outside this mesh library's scope.

## 7. Validation (D16)

`CADModel::validate()` extends `validate_mechanism`:
- part names unique; every part has non-empty mesh
- patch face ordinals within `indices.size()/3`
- every `PartMeta.material`, `patch.material`, and region material resolves in
  the `MaterialDB` (empty = default, allowed where documented)
- region/domain/interface/BC/body-load/plane/sector/frame references resolve
- errors are actionable strings; deterministic

Mesh-level gates (manifold, consistent winding, patch coverage) reuse the
existing watertight machinery where applicable; per-mesh checks are Phase C.

## 8. Roadmap phases

| Phase | Theme | Work | Milestone |
|---|---|---|---|
| **A** | **CADModel IR** | `cad_model.hpp` + `MaterialDB` + patch semantics + `validate` + `make_cad_model` | gearbox model: parts+patches+materials+mechanism validate true; multiphysics sample (solid+fluid region, FSI interface) validates |
| B | CAE adapters | STL, Gmsh MSH, VTK/VTU, tessellated (faceted) STEP export | round-trip: MSH self parse; STEP structural self-check |
| C | Import (light) | STL/OBJ/PLY/MSH importers → CADModel | file → CADModel → render |
| D | Analytic BREP | deferred — external CAD kernel would be required; never in-repo | — |
| E | Optional solver decks | OpenFOAM skeleton, Abaqus `.inp`, Nastran `.bdf` | on demand |

## 9. Documented decisions

- **D11 — CADModel is the unified in-memory IR.** File formats are adapters;
  importers build it. No canonical JSON serialization added (consistent with
  D9); the IR is memory-only.
- **D12 — MeshData is a lossy projection.** Reis not the model; keep the model
  when patches/materials/physics are needed downstream.
- **D13 — Two request tiers.** Single-part generators keep returning
  `MeshData`; recipes build `CADModel`. Nothing existing breaks.
- **D14 — Materials: in-repo curated `MaterialDB`**, referenced by name, bound
  per part, per patch, and per physics per region.
- **D15 — Multiphysics topology.** Regions (closed volumes) + domains +
  interfaces; BCs/loads are physics-tagged. Model declares; solvers configure.
- **D16 — CAE validation is first-class** with actionable, deterministic
  diagnostic strings.
- **D17 — Tessellated STEP first** (faceted solid B-rep, shipped); analytic
  BREP deferred — an external CAD kernel (e.g. OCCT) would be required if
  ever pursued. Never a hand-rolled BREP parser.
- **D18 — Import split.** Tessellated formats in-repo (shipped); analytic
  BREP deferred (external kernel would be required).
- **D19 — Symmetry/cyclic/rotating-frame metadata is first-class** for the
  turbomachinery recipes.

## 10. Non-goals (for now)

- Solver configuration (element order, tolerances, timestep, solver choice)
- Result/solution field storage (VTK/VTU output covers)
- Analytic BREP inside geometry; material-property solve engines
- Exact gear contact meshing, NURBS modeling (see recipes.md non-goals)

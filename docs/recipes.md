# Domain Recipe Roadmap

> **Status: planning.** Companion to `plan.md` (the kernel roadmap). Recipes are
> domain-semantic generators built **on** kernel operators; this document plans
> *what to build and in what order* — `plan.md` plans the operators themselves.
> A recipe is cheap once its operators exist; the ordering below sequences
> operator work against the recipes it unlocks, so every phase ships value.

## 1. Goal

Parametric, simulation-ready geometry for the multiphysics domains the library
serves: rotating machines, reciprocating machines, electromagnetic machines,
vehicles & aircraft, ordnance, RF/antenna systems, fluid systems, thermal
systems, and organic/reconstruction shapes. Every recipe returns a **patched
`Assembly`** (named parts + boundary patches + motion/material metadata) so the
gizmo layer, the patch layer, and downstream solvers all see the same model.

## 2. Principles

1. **Component + machine layering.** Two recipe tiers:
   - **Component recipes** — reusable building blocks: shaft, gear, piston,
     port, blade row, wheel, horn, fin, spring, bearing, ….
   - **Machine recipes** — assemblies that compose components with simple
     kinematics (engine, motor, gearbox, gear pump, chassis, glider, …).
   Machines are never bespoke meshers. Exceptions only where irreducible domain
   math lives (involute profiles, blade camber — the turbine precedent).
2. **State-parametric descriptors.** Machines are mechanisms. Descriptors carry
   state — `crank_angle`, `steering_angle`, `suspension_travel`,
   `control_deflection`, `elevation`/`azimuth` — and `generate_*_assembly()`
   evaluates the mechanism internally and returns *placed* parts. Result:
   state-parameterizable geometry for simulation sweeps, without any animation
   framework.
3. **Simulation empathy (the contract).** Recipes return: named parts
   (`rotor_0`, `casing`, `block`, `piston`…), named BC patches
   (`inlet`, `outlet`, `wall`, `interface_rotor_stator`, `contact`…),
   **watertight solids** where the part is solid (reuse the boolean closed-manifold
   gate as the QA check), and per-part motion/material metadata (Phase 0).
4. **Determinism + tests.** Same descriptor → same mesh. Every recipe ships
   doctest cases pinning parts/patches/watertightness/counts. SI units, sane
   defaults, all-defaulted descriptors.
5. **Reuse before recipe.** New shapes normally compose components and shared
   internal machinery (the `turbine_internal.hpp` precedent), not new meshers.

## 3. Recipe contract (checklist per recipe)

- descriptor struct: SI units, all-defaulted fields, documented state params
- `generate_<name>_assembly()` → patched `Assembly` (part/patch names in header)
- `generate_<name>_mesh()` convenience (flatten) where single-mesh renders occur
- closed-manifold gate passes for solid parts; mass properties exposed when solid
- `generate_<name>()` returns `{ Assembly, Mechanism }` (posed assembly for
  rendering + joint/transmission graph for simulators)
- `to_mjcf()` / `to_urdf()` export smoke-tested per recipe (well-formed,
  correct joint frames/ratios/inertials)
- doctest cases; README bullet; recipes.md row

## 4. Kernel dependency map

| Operator | Status | Gates |
|---|---|---|
| Loft/skin (multi-section) | ✅ shipped | ducts, housings, horns, fuselage v1, wings v1 |
| Loft rails (guide curves) | 📋 plan | wing planform, fuselage quality, manifold centerlines |
| Sweep, arbitrary profile + parallel-transport frames | 📋 plan | non-circular ducts/manifolds, spars, threads (real) |
| Helix (circular profile) | ✅ shipped | springs, windings v1, threads v1 (cosmetic) |
| Booleans V1 (non-coplanar contact) | ✅ shipped | bore subtract without coplanar contact, spokes v1 |
| Booleans V2 (coplanar contact) | 📋 plan | **engine-class gate** — blocks, housings, casings |
| Shelling (hollow solids) | 📋 plan | water jackets, manifolds, skin-and-rib parts |
| Fillets / chamfers (mesh-level) | 📋 plan | powertrain-class quality (cranks, rods, housings) |
| Involute profile generator (2D) | 📋 plan | spur/helical gears, gear pumps, toothed pulleys |
| Mass properties (V, A, centroid, inertia) | 📋 plan (bool volume exists privately) | every solid recipe; simulator inputs |
| Part metadata (motion / material / density / contact) | 📋 plan | every machine recipe; sim export |
| Motion graph (joints, couplings, limits, tree FK) | 📋 plan (Phase 0b) | every connected machine recipe |
| MJCF / URDF constraint export | 📋 plan (Phase 0b) | simulator consumers of every machine recipe |
| Assembly instancing (place parts × N) | 📋 plan (transform_part + merge_parts exist) | every machine recipe |
| Kinematics pose helpers (crank-slider, 4-bar, rack-pinion) | ✅ crank-slider in steam engine (recipe-local, per D2); general helpers 📋 plan | engines, suspensions, landing gear |
| Parametric surfaces / NURBS-lite | 🔭 long | quality fuselage/wing skins (v2) |
| SDF blends + marching cubes | ✅ shipped | organic shapes (heart) v1 |

## 5. Component catalog

**Lathe / revolve family** (existing ops)

| Component | Kernel needs | Phase | Size |
|---|---|---|---|
| Shaft / axle (steps, keyway) | lathe + boolean bore | 1 | S |
| Flywheel | lathe + bolt-hole boolean | 1 | S | (✅ shipped inside the steam engine recipe: V-groove pulley disc) |
| Pulley (groove + bore) | lathe + boolean | 1 | S |
| Nozzle / diffuser / venturi | lathe profiles | 1 | S |
| Projectile body (ogive / tangent / secant / boattail) | lathe (real ordnance vocabulary) | 1 | S |
| Tire (toroidal profile) | lathe | 1 | S |
| Parabolic reflector (exact parabola lathe) | lathe | 1 | S |
| Helical antenna | helix | 1 | S |
| Dipole / whip / Yagi elements | tube | 1 | S |
| Poppet valve (seat + stem) | lathe | 2 | S |
| Spring (helical, closed-ground ends) | helix (variable pitch v2) | 1 | S |
| Wheel rim + hub | lathe + spoke booleans | 4 | M |
| Piston + rings | lathe + grooves | 4 | S |
| Coupling / flange | lathe + bolt-circle boolean | 4 | S |
| Impeller (backward-curved blades) | blade-row machinery + volute loft | 3 | M |
| Propeller / rotor blade row | turbine blade machinery, prop params | 3 | M |

**2D-profile extruded family**

| Component | Kernel needs | Phase | Size |
|---|---|---|---|
| Spur gear | involute profile + extrude | 2 | M |
| Helical gear | involute + helix skew | 2 | L |
| Cam | 2D cam profile + extrude | 2 | S |
| Toothed pulley | involute-ish profile | 2 | S |
| Stator / rotor lamination stack | stamped 2D slot profile + extrude | 2 | M |
| Heat-exchanger fin | offset fin profile + extrude | 3 | S |
| Washer / flange / gasket | 2D ring | 1 | S |

**Loft / sweep family**

| Component | Kernel needs | Phase | Size |
|---|---|---|---|
| Duct / manifold runner | loft sections; rails v2 | 3 | M |
| Horn antenna (square→square flare) | loft | 3 | S |
| Intake / exhaust port | lofted runner | 3 | S |
| Nacelle | revolve + intake loft | 3 | M |
| Wing surface (sections + split control surfaces) | loft; planform rails v2 | 3 | L |
| Fuselage v1 | section loft | 3 | L |
| Fin (airfoil-ish profile) | extrude/loft | 1/3 | S |
| Valve body (lofted ports + solid housing) | loft + booleans | 4 | M |

**Boolean-heavy solids** (gate: boolean V2 coplanar + fillets)

| Component | Kernel needs | Phase | Size |
|---|---|---|---|
| Engine block v1 (single-cylinder) | box + bore subtract + head face (coplanar!) | 4 | L |
| Cylinder head (hollow + ports) | booleans + port lofts | 5 | M |
| Motor housing + end caps | revolve + booleans | 4 | M |
| Gearbox / casing shell | half-shell + booleans | 4 | M |
| Spoked wheel | rim + spoke union + hub bore | 4 | M |
| Bearing (races + cage + rollers) | torus + booleans | 4 | M |
| Manifold (runners + flange) | sweep + flange boolean | 4 | M |
| Connecting rod | extrude + boolean + fillet | 5 | M |
| Crankshaft v1 | journal/crank-web booleans + fillet | 5 | L |

**RF / antenna family** — parabolic dish 1, horn 3, waveguide (box + boolean) 4,
Yagi (tube) 1, dipole 1, helical 1, phased-array tile (structured repeat) 3.

## 6. Machine catalog

| Machine | Composed components | State params | Phase | Size |
|---|---|---|---|---|
| Steam engine (single-cyl) | ✅ v1 geometry (2026-08); migrating to the mechanism contract (0b): body-local parts + joints (continuous shaft, revolute conrod ends, prismatic crosshead) + MJCF/URDF export | crank angle | 5 → 0b | M |
| Combustion engine (single-cyl air-cooled) | block, head, piston+rings, conrod, crankshaft v1, valves, springs, flywheel | crank angle, valve lift | 5 | L |
| Electric motor (PM BLDC) | stator stack, windings, rotor core, magnets, shaft, housing, end caps, fan | rotor angle | 5 | M |
| Gearbox v1 (2-shaft) | spur gears, shafts, housing, bearings | shaft angle | 5 | M |
| Gear pump | 2 spur gears + casing | shaft angle | 5 | M |
| Centrifugal pump | impeller, volute, casing | impeller angle | 6 | M |
| Projectile (guided) | body, fins, nose cap, antenna window | — | 1→4 | M |
| Antenna system | dish/horn/Yagi + mount | elevation, azimuth | 4 | S |
| Rolling chassis v1 | tube frame, wheels+tires, suspension arms, axle | steering, travel | 6 | L |
| Aircraft v1 (glider) | fuselage v1, wing, empennage, control surfaces | deflections | 6 | L |
| Rotor / prop head | blade rows + hub + shaft | collective | 6 | M |
| Radiator + fan | fin core, shroud, fan | fan angle | 6 | M |
| Turbocharger | compressor + turbine + center housing + shaft | shaft speed | 6 | M |

## 7. Cross-cutting sim utilities (Phase 0)

- **Assembly instancing** — `(Part, Mat4)` list → `Assembly`, instance naming
  (`cyl_0`, `cyl_1`), patches per instance; builds on `transform_part` +
  `merge_parts`.
- **Mass properties** — `mesh_volume()`, `surface_area()`, `centroid()`,
  `inertia_tensor()` (double accumulation; promote the boolean signed-volume
  helper to public).
- **Part metadata** — `PartMeta { PartMotion motion; std::string material; }`
  (motion: Stationary / Rotating / Reciprocating / Oscillating); recipes
  populate; solvers consume.
- **Boolean V2** — coplanar-face overlap handling (the engine-block gate) and
  **shelling** (`shell_part(part, thickness)` = offset + subtract, v1
  approximate — offset surfaces are research-grade, documented).
- **Kinematics pose helpers** — crank-slider, 4-bar, rack-pinion (2D pose math;
  lives in recipes).
- **Motion graph (connectivity core)** — `Mechanism { joints, couplings,
  driver }`: fixed/revolute/continuous/prismatic joints with limits +
  gear/belt/rack transmissions coupling joint coordinates (ratio + sense);
  tree forward kinematics `evaluate_poses(mechanism, state)` → per-part
  world poses; `assemble(mechanism, parts)` via ordinal-preserving
  `transform_part` (patches survive). Closed loops (slider-cranks, gear
  trains) are carried by exported constraints, not resolved by in-library FK.
- **Simulation export** — `to_mjcf()` (primary: gears via `equality polycoef`,
  belts via `tendon divisor`, auto-frame parent-local joints, actuation +
  limits) and `to_urdf()` (secondary: widest importer coverage; gears as
  `<mimic>`, no loops). Recipes return `{ Assembly, Mechanism }` so constraints
  travel with the geometry; glTF stays geometry-only for renderers.
- **Contact labeling** — `PartMeta.contact` (explicit opt-in flag): only
  parts marked `contact = true` export their patches as collision surfaces.
- **Deformable bodies (Tier 1 shipped: compliance; Tier 2: soft meshes)** —
  Tier 1: joints carry stiffness/damping/armature/frictionloss (soft
  interfaces: belts, springs as compliant joints + cosmetic helix); Tier 2:
  `PartMeta.kind = Deformable` + constitutive params, exported as MJCF
  `deformable`/`flex` boundary meshes (MuJoCo 3.x tetrahedralizes
  internally), welded to rigid bodies via the part's contact patches.
  Deformables are MJCF-only (URDF has no soft-body story). FK treats them
  as rigid placeholders; deformed state comes back from the simulator.

## 8. Roadmap phases

| Phase | Theme | Operator work | Recipes | Milestone / validation |
|---|---|---|---|---|
| 0 | Sim core | instancing, mass props, metadata, boolean V2 coplanar, shelling, kinematics | — | boolean V2 suite: block-with-2-bores subtract watertight |
| 0b | Connectivity & sim export | motion graph (joints/couplings + tree FK + assemble), public mass/inertia props, MJCF + URDF exporters, PartMeta.contact | steam engine migration to mechanism contract | steam engine: one descriptor → rendered assembly + MJCF loads in MuJoCo |
| 1 | Lathe & profile family | (none — shipped) | projectile, dish, spring, nozzle, flywheel, pulley, tire, whip/Yagi, helical antenna | projectile assembly: watertight + mass properties |
| 2 | Gear & lamination family | involute generator | spur gear, helical gear, cam, toothed pulley, lamination stacks | involute profile suite; gear assembly patches |
| 3 | Sweep & rails family | arbitrary-profile sweep + parallel transport; loft rails | ducts, horn antenna, ports, nacelle, wing v1, fuselage v1, impeller, prop | manifold runner; wing with control-surface split patches |
| 4 | Solid-pack family | fillets/chamfers; shelling production; boolean V2 production | wheel, housings, bearing, valve body, engine block v1 | single-cyl block: bores + water jacket shell, watertight |
| 5 | Machines I | (compose) | steam engine, single-cyl ICE, PM motor, gearbox, gear pump, projectile complete, antenna systems | ICE assembly at 3 crank angles: watertight + patched |
| 6 | Machines II / vehicles | (compose) | rolling chassis, glider, rotor/prop, turbocharger, centrifugal pump, radiator | chassis steering sweep (state-parametric) |
| 7 | Organic & quality | loft rails quality, parametric surfaces, SDF heart | fuselage v2, heart v1, fairings | heart assembly (v1 approximation) |

Rough sizing: 0 M · 0b M · 1 M · 2 L · 3 L · 4 XL · 5 L · 6 L · 7 ongoing.

## 9. Documented decisions & tradeoffs

- **D1 — flat assemblies.** No part hierarchy in v1; recipes name instances
  (`cyl_0`) so `flatten()` prefixing stays valid. Hierarchy = future.
- **D2 — kinematics in recipes.** State-param math (crank-slider…) lives in
  recipes, not the kernel. Kernel stays shape, recipes carry mechanism.
- **D3 — minimal metadata.** `PartMeta{PartMotion motion; std::string material;
  float density; bool contact}` — motion for the scene layer, density for
  mass-property computation, `contact` as the explicit opt-in flag that
  exports a part's patches as collision surfaces. Deliberately not a
  heavyweight schema; extend per domain later.
- **D4 — boolean V2 is the engine-class gate.** Nothing engine/housing-like
  before coplanar booleans; V1 returns `{}` on coplanar contact by design.
- **D5 — fillets gate powertrain-class quality.** Sharp-edged v1 components may
  ship with the limitation documented; fillets raised where quality demands.
- **D6 — rails/parametric surfaces are quality gates, not hard gates.** v1 wing
  and fuselage are acceptable without rails/patched surfaces.
- **D7 — threads v1 are cosmetic helixes.** Real thread-profile sweep waits for
  the arbitrary-profile sweep operator.
- **D8 — shells are approximate** (offset + subtract); exact offset surfaces are
  out of scope and documented as such.
- **D9 — MJCF is the primary constraint export; URDF the secondary.** Gears
  (`equality joint polycoef`), belts/rack (`tendon divisor`), and weld-loops
  survive the MJCF round trip; it auto-inherits inertials, ships official
  WASM/JS bindings (same artifact feeds the WebGL path), and is the converter
  hub (SDF/USD/Newton). URDF offers the widest importer coverage but no
  transmissions (position-level `<mimic>` only) and no kinematic loops. No
  custom JSON canonical serialization; no UsdPhysics/glTF-physics exports
  (unratified draft, no gears). glTF stays geometry-only for renderers.
- **D10 — inertials are computed, not placeholder.** `mesh_properties(mesh,
  density) → {volume, mass, centroid, inertia about COM}` from watertight
  solids (the boolean gate qualifies parts); both exporters emit these values
  (URDF requires them; MJCF takes ours rather than compiler inference).

## 10. Non-goals (for now)

- Exact gear contact meshing (backlash/contact analysis belongs to solvers)
- NURBS / free-form CAD import; parametric-surface tessellation is v2 quality
- Automatic clearance/tolerance modeling — clearances are explicit descriptor params
- FSI morphing — geometry supplies the boundary surface; deformation belongs to the
  deform layer the solver drives

## 11. Interaction with the rest of the ecosystem

Recipes are the natural clients of every layer this library builds:
- **Parts** are gizmo pick/transform units — machine recipes are immediately
  editable in the interaction layer.
- **Patches** are solver BCs — recipe constructor names (`inlet`, `outlet`,
  `interface_rotor_stator`) map to solver patch names downstream.
- **State params** give the scene layer time/state sweeps for animations and
  transient simulation setup without touching geometry code.

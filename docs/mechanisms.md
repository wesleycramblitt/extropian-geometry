# Motion Graph & Simulation Export — Design Notes

> **2026-08.** Companion to `plan.md` (kernel) and `recipes.md` (recipes). This
> is the design record for the connectivity layer: how machines declare their
> joints/couplings, how frames are kept consistent end-to-end, and how the
> same descriptor feeds both the renderer and physics simulators.

## 1. The contract

Every machine recipe returns `{ Assembly, Mechanism }` (plus body-local
parts):

```cpp
struct SteamEngineResult {          // the pattern all machines follow
    Assembly assembly;              // world-posed render
    Mechanism mechanism;            // joints + couplings + driver
    std::vector<Part> body;         // parts in body-local frames
};
```

- `assembly` is what the gizmo/scene layer renders.
- `mechanism` is what `to_mjcf()` / `to_urdf()` serialize: **constraints travel
  with the geometry**.
- `body` is the same geometry in the frame the exporters and
  `evaluate_poses()` expect.

## 2. Frame convention (the one rule)

**A part's body-local origin IS its joint anchor.**

- `Joint.anchor`/`axis` are expressed in the PARENT part's frame; `""` parent
  = the world (static base).
- The child's world pose = `parentWorld · T(anchor) · R(axis, q)` (or the
  prismatic/fixed variant) — the **MJCF convention**, used by the internal FK,
  by every recipe's local-frame construction, and by both exporters. No
  back-shift, no per-exporter compensation.
- Static parts (no incoming joint) have their local frame == the world frame;
  the FK treats them as identity for their children.
- Recipes with closed loops (the steam engine's slider-crank) resolve the
  loop with analytic pose math at generation time (roadmap D2); the exported
  mechanism carries the same loop as constraints, and the *simulator* solves
  it. `evaluate_poses()` is tree-only by design.

## 3. Schema

```cpp
Joint   { name, kind (Fixed/Revolute/Continuous/Prismatic),
          parent, child, anchor, axis,
          limit_low/high, effort_max, velocity_max,
          stiffness, damping, armature, frictionloss }   // compliance = soft interfaces
Coupling{ name, kind (Gear/Belt/Rack), joint_a, joint_b, ratio }   // q_b = ratio·q_a
Mechanism{ joints, couplings, driver_joint }
```

- **Couplings couple joint coordinates, not bodies** — the separation that
  makes gear trains declarative and maps 1:1 to MJCF `equality joint
  polycoef`, MJCF tendons (belts), URDF `<mimic>`.
- A part with several incoming joints is a **kinematic loop**: the first
  declared joint owns the FK chain; the others are exported as constraints
  (`<connect>` point welds; URDF cannot express loops at all).
- **Deformables, Tier 1** (shipped): compliant joints (stiffness/damping/
  armature/frictionloss) model springs, belts, mounts — soft interfaces, no
  soft meshes. **Tier 2** (schema): `PartMeta.kind = Deformable` + constitutive
  params; exported as MJCF `flex` boundary meshes (MuJoCo 3.x tetrahedralizes
  internally); attachments = the part's contact patches; FK treats the body as
  a rigid placeholder, the deformed state returns from the simulator.
  Deformables are MJCF-only (URDF has no soft-body story).

## 4. Export mapping

| Concept | MJCF (primary) | URDF (secondary) |
|---|---|---|
| Global base | `worldbody` | `world` link |
| Revolute w/ limits | hinge + `range` | revolute + `limit` |
| Continuous | hinge (no range) | continuous |
| Prismatic | slide | prismatic |
| Fixed/weld | no joint (welded body) | fixed joint |
| Gear / belt / rack | `<equality><joint polycoef="0 −ratio 1"/>` | `<mimic joint= multiplier=ratio>` (position-level) |
| Kinematic loop | `<equality><connect anchor=` (world coords @ rest) `/>` | impossible |
| Driver | motor actuator + `ctrlrange` | — (joint limits only) |
| Inertials | `<inertial>` from `mesh_properties` (D10) | `<inertia>` full tensor |
| Collision | `contype/conaffinity` on `meta.contact` parts | `<collision>` block |
| Compliance | joint dynamics attrs | `<dynamics>` |
| Deformable | `deformable`/`flex` (surface mesh; attachments via patches) | — |

Decisions D9/D10 (recipes.md): MJCF is the primary interchange (gears/loops/
inertials/compliance are native, official WASM bindings feed the WebGL path,
converter hub to SDF/USD/Newton); URDF exists for the widest importer coverage.
No custom JSON canonical serialization; no UsdPhysics/glTF-physics exports
(unratified, no gears). glTF stays geometry-only for renderers.

## 5. Mass properties

`mesh_properties(mesh, density)` → `{volume, surface_area, mass, centroid,
inertia about COM}` via Mirtich tetrahedron decomposition (double
accumulation): `∫x²·dV = V6/10·Σ(px², pxqx…)`, `∫xy·dV = V6/20·(2Σpᵢqᵢ +
Σ(pᵢqⱼ+pⱼqᵢ))` (mixed products!), parallel-axis shift to the centroid held in
`math::Mat3`. Exact on analytic boxes/cylinders; faceted surfaces converge.
Winding must be consistent (the boolean gate qualifies parts; a regression
test pins the primitive cylinder whose caps used to be misoriented, cutting
signed volume to 1/3).

## 6. Worked example: steam engine

| Joint | Kind | Parent → Child | Anchor (parent frame) |
|---|---|---|---|
| `shaft` | Continuous | world → crankshaft | `(crank_x, 0, 0)`, axis +Z |
| `fly_fix` | Fixed | crankshaft → flywheel | `(0,0,0)` |
| `pin_fix` | Fixed | flywheel → crank_pin | `(rc, 0, pin_z_center)` |
| `conrod_pin` | Revolute | crank_pin → conrod | `(0,0,0)`, +Z |
| `piston_sl` | Prismatic | cylinder → piston | `(x_c(0), 0, rod_plane_z)`, +X |
| `cross_fix` | Fixed | piston → crosshead | `(0,0,0)` |
| `conrod_cs` | Revolute | crosshead → conrod | `(0,0,0)`, +Z — **loop edge** |

The conrod's local mesh has its skew (rod plane above the pin's z-span)
baked in, origin at the big end: its small end sits at `(L, 0, Δz)` in the
pin-aligned frame; the exporter emits `<connect body1="crosshead"
body2="conrod" anchor=world(rest)>` to close the loop. The recipe renders
the loop with the analytic crank-slider
(`x_c = x_cc + rc·cosθ + √(L²−rc²·sin²θ)`; conrod world angle
`ψ = −asin(rc·sinθ/L)`); the FK at state 0 reproduces the same rest pose,
which the tests pin part-for-part.

## 7. Validation & testing

- `validate_mechanism`: unknown/self parts, degenerate axes, duplicate
  names, bad couplings (unknown joints, zero/NaN ratio), missing driver.
- Tests pin: wheel-on-axle rotation, gear ratios (both drive directions),
  prismatic limits, anchored revolute frames, determinism (byte-identical
  exports), patch preservation through `apply_poses`, watertight+gated
  volumes per part, the steam engine rest-pose equivalence, and a **gated
  MuJoCo round-trip** (`python3 -c "import mujoco"` available → the exported
  MJCF must actually load; otherwise the test skips with a message).

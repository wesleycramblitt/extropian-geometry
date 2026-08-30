#pragma once

#include <exd/geometry/part.hpp>
#include <exd/math/mat4.hpp>
#include <exd/math/vec3.hpp>

#include <map>
#include <string>
#include <vector>

namespace exd::geometry
{

// ═══════════════════════════════════════════════════════════════════════════
//  Motion graph — "how named parts connect" (connectivity core, Phase 0b).
//
//  A Mechanism is a graph of Joints (body-to-body DOFs) plus Couplings
//  (transmissions that couple JOINT COORDINATES: gears, belts, rack-pinion).
//  Recipes return { Assembly, Mechanism } so the constraints travel with the
//  geometry: `to_mjcf()`/`to_urdf()` exporters serialize them for physics
//  simulators, while `evaluate_poses()` provides FORWARD KINEMATICS for open
//  chains (gear trains, wheel-on-axle, robot arms). Closed loops
//  (slider-cranks: the steam engine) are carried by the exported constraints;
//  the simulator solves them — in-library pose math for loops stays in the
//  recipe (roadmap D2).
//
//  Conventions:
//    • Joint parent/child are PART NAMES; parent == "" means the world
//      (static base). Static parts (no incoming joint) weld to the world.
//    • anchor/axis are in the PARENT part's local frame (MJCF convention).
//    • A part may be the child of several joints — a kinematic loop: the
//      evaluator uses the FIRST incoming joint (declaration order) as the FK
//      chain; the others are exported as loop constraints.
// ═══════════════════════════════════════════════════════════════════════════

enum class JointKind
{
    Fixed,       // no DOF: weld child to parent
    Revolute,    // rotation about `axis` through `anchor`
    Continuous,  // rotation about `axis`; no limits
    Prismatic    // translation along `axis`
};

struct Joint
{
    std::string name;
    JointKind   kind   = JointKind::Fixed;
    std::string parent = "";   // part name; "" = world
    std::string child;         // part name
    math::Vec3f anchor{0.0f, 0.0f, 0.0f};   // parent frame
    math::Vec3f axis{0.0f, 0.0f, 1.0f};     // parent frame
    float limit_low  = -1e30f;   // rad (revolute/continuous) or m (prismatic)
    float limit_high =  1e30f;
    float effort_max = 1e30f;    // N·m or N
    float velocity_max = 1e30f;  // rad/s or m/s
};

/// Two joint coordinates coupled as q_b = ratio · q_a (ratio sign carries
/// the sense: negative = counter-rotating gear pair, belt reversal…).
enum class CouplingKind
{
    Gear,   // q_b = ratio·q_a            (MJCF: equality joint polycoef)
    Belt,   // q_b = ratio·q_a            (MJCF: tendon with pulley divisor)
    Rack    // q_b = ratio·q_a, q in {m, rad} mixed units allowed
};

struct Coupling
{
    std::string name;
    CouplingKind kind = CouplingKind::Gear;
    std::string joint_a;
    std::string joint_b;
    float ratio = 1.0f;   // nonzero; sign = sense
};

struct Mechanism
{
    std::vector<Joint>    joints;
    std::vector<Coupling> couplings;
    std::string driver_joint;   // the joint whose coordinate the FK state drives
};

/// Validate graph integrity: unknown/self parent-child parts, degenerate
/// axes, duplicate joint/coupling names, couplings referencing unknown
/// joints, NaN/zero ratios. Returns false and fills `errors` when invalid.
bool validate_mechanism(const Mechanism& mech, std::vector<std::string>& errors);

/// Tree forward kinematics: resolves the driver joint's coordinate to `state`
/// (clamped to the joint limits), propagates couplings transitively where
/// acyclic (deterministic order), and composes world poses along the FK
/// chains. Parts with no incoming joint get identity (static at the world).
/// Closed loops resolve via the first incoming joint; the coupling traversal
/// stops at cycles. Returns part name → world matrix.
std::map<std::string, math::Mat4> evaluate_poses(const Mechanism& mech, float state);

/// Apply poses to parts (patch-preserving: transform_part keeps triangle
/// ordinals, so patches survive) and merge into a world-frame Assembly.
/// Parts absent from `poses` are placed at identity. Deterministic.
Assembly apply_poses(const Mechanism& mech, std::span<const Part> parts,
                     const std::map<std::string, math::Mat4>& poses);

} // namespace exd::geometry

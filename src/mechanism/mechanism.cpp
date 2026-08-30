#include <exd/geometry/mechanism.hpp>

#include <exd/geometry/mesh_ops.hpp>
#include <exd/math/quat.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace exd::geometry
{
namespace
{

math::Quat axis_rotation(const math::Vec3f& axis, float angle)
{
    return math::Quat::from_axis_angle(axis.normalized(), angle);
}

/// Joint pose relative to the parent, in the MJCF frame convention: the
/// child's origin IS the joint anchor, so the child pose is
/// T(anchor) · R(axis, q) (or T(anchor) · S(axis·q)) with NO back-shift.
/// This keeps internal FK, recipe body-local frames, and the exporters on
/// one convention: a part's local origin is its joint point.
math::Mat4 joint_offset(const Joint& j, float q)
{
    const math::Mat4 T = math::Mat4::trs(j.anchor, math::Quat{1.0f, 0.0f, 0.0f, 0.0f},
                                         math::Vec3f{1.0f, 1.0f, 1.0f});
    switch (j.kind)
    {
    case JointKind::Fixed:
        return T;
    case JointKind::Revolute:
    case JointKind::Continuous:
        return math::Mat4::mul(T, math::Mat4::trs(math::Vec3f{0.0f, 0.0f, 0.0f},
                                                  axis_rotation(j.axis, q),
                                                  math::Vec3f{1.0f, 1.0f, 1.0f}));
    case JointKind::Prismatic:
        return math::Mat4::mul(T, math::Mat4::trs(j.axis.normalized() * q,
                                                  math::Quat{1.0f, 0.0f, 0.0f, 0.0f},
                                                  math::Vec3f{1.0f, 1.0f, 1.0f}));
    }
    return math::Mat4::identity();
}

bool is_world(const std::string& s) { return s.empty(); }

} // namespace

bool validate_mechanism(const Mechanism& mech, std::vector<std::string>& errors)
{
    errors.clear();
    bool ok = true;
    auto fail = [&](const std::string& msg) { errors.push_back(msg); ok = false; };

    std::unordered_set<std::string> jointNames, couplingNames;
    std::unordered_map<std::string, std::string> partParent;   // part → first incoming joint
    std::unordered_set<std::string> staticParts;

    for (const Joint& j : mech.joints)
    {
        if (j.name.empty()) { fail("joint with empty name"); continue; }
        if (!jointNames.insert(j.name).second) { fail("duplicate joint name '" + j.name + "'"); }
        if (is_world(j.child)) { fail("joint '" + j.name + "' has empty child"); }
        if (!is_world(j.parent) && j.parent == j.child)
            fail("joint '" + j.name + "' self-parents part '" + j.child + "'");
        if (!is_world(j.parent) && partParent.count(j.parent) == 0 && mech.driver_joint == j.name)
            ;   // driver joint parent may be static; fine
        if (partParent.count(j.child) == 0)
            partParent[j.child] = j.name;
        if (j.kind != JointKind::Fixed && j.axis.length() < 1e-6f)
            fail("joint '" + j.name + "' has a degenerate axis");
        if (j.kind == JointKind::Fixed)
        {
            if (j.limit_low > 1e29f || j.limit_high < -1e29f) {}
        }
        else if (j.limit_low > j.limit_high)
            fail("joint '" + j.name + "' has limit_low > limit_high");
    }

    for (const Coupling& c : mech.couplings)
    {
        if (c.name.empty()) { fail("coupling with empty name"); continue; }
        if (!couplingNames.insert(c.name).second)
            fail("duplicate coupling name '" + c.name + "'");
        if (c.joint_a.empty() || c.joint_b.empty())
            fail("coupling '" + c.name + "' references an empty joint");
        if (c.joint_a == c.joint_b)
            fail("coupling '" + c.name + "' couples a joint to itself");
        if (jointNames.count(c.joint_a) == 0)
            fail("coupling '" + c.name + "' references unknown joint '" + c.joint_a + "'");
        if (jointNames.count(c.joint_b) == 0)
            fail("coupling '" + c.name + "' references unknown joint '" + c.joint_b + "'");
        if (!std::isfinite(c.ratio) || c.ratio == 0.0f)
            fail("coupling '" + c.name + "' has a zero/non-finite ratio");
    }

    if (mech.driver_joint.empty())
        fail("mechanism has no driver joint");
    else if (jointNames.count(mech.driver_joint) == 0)
        fail("driver joint '" + mech.driver_joint + "' is not declared");

    return ok;
}

std::map<std::string, math::Mat4> evaluate_poses(const Mechanism& mech, float state)
{
    std::map<std::string, math::Mat4> poses;

    // ── joint coordinates: driver + acyclic coupling feed-forward ──
    std::map<std::string, float> q;
    std::unordered_map<std::string, size_t> jointIndex;
    for (size_t i = 0; i < mech.joints.size(); ++i)
    {
        jointIndex[mech.joints[i].name] = i;
        q[mech.joints[i].name] = 0.0f;
    }

    if (jointIndex.count(mech.driver_joint))
    {
        const Joint& dj = mech.joints[jointIndex[mech.driver_joint]];
        q[mech.driver_joint] = dj.kind == JointKind::Fixed
                                   ? 0.0f
                                   : std::clamp(state, dj.limit_low, dj.limit_high);
    }

    // Coupled coordinates: BFS from the driver, deterministic order; cycles
    // (kinematic loops) stop the propagation and are left at 0.
    std::unordered_set<std::string> visited{mech.driver_joint};
    std::vector<std::string> frontier{mech.driver_joint};
    while (!frontier.empty())
    {
        std::vector<std::string> next;
        for (const std::string& jn : frontier)
        {
            for (const Coupling& c : mech.couplings)
            {
                std::string other;
                float scale = 0.0f;
                if (c.joint_a == jn && !visited.count(c.joint_b))
                {
                    other = c.joint_b;
                    scale = c.ratio;
                }
                else if (c.joint_b == jn && !visited.count(c.joint_a))
                {
                    other = c.joint_a;
                    scale = 1.0f / c.ratio;
                }
                if (other.empty()) continue;
                const Joint& oj = mech.joints[jointIndex[other]];
                q[other] = std::clamp(q[jn] * scale, oj.limit_low, oj.limit_high);
                visited.insert(other);
                next.push_back(other);
            }
        }
        frontier = std::move(next);
    }

    // ── pose composition along FK chains (first incoming joint per part) ──
    std::unordered_map<std::string, std::string> parentJointOfPart;   // part → joint
    std::unordered_map<std::string, const Joint*> byName;
    for (const Joint& j : mech.joints)
    {
        byName[j.name] = &j;
        if (!is_world(j.child) && parentJointOfPart.count(j.child) == 0)
            parentJointOfPart[j.child] = j.name;   // declaration order = FK chain
    }

    std::unordered_map<std::string, math::Mat4> world;
    std::vector<const Joint*> work;
    size_t had = 0;
    for (const Joint& j : mech.joints)
        if (is_world(j.parent))
            work.push_back(&j);
    // worklist: unresolved entries requeue until their parent resolves;
    // bounded to avoid infinite loops on cyclic declarations.
    const size_t maxPasses = mech.joints.size() * mech.joints.size() + 16;
    size_t requeues = 0;
    for (size_t i = 0; i < work.size(); ++i)
    {
        const Joint* j = work[i];
        if (world.count(j->child))
            continue;
        math::Mat4 parentWorld = math::Mat4::identity();
        if (!is_world(j->parent))
        {
            const auto it = world.find(j->parent);
            if (it == world.end())
            {
                if (++requeues > maxPasses) continue;
                work.push_back(j);
                continue;
            }
            parentWorld = it->second;
        }
        world[j->child] = math::Mat4::mul(parentWorld, joint_offset(*j, q[j->name]));
        for (const Joint& kid : mech.joints)
            if (kid.parent == j->child && !world.count(kid.child))
                work.push_back(&kid);
    }

    // emit world poses: static parts that parent joints are at identity
    for (const auto& [part, pose] : world)
        poses[part] = pose;
    return poses;
}

Assembly apply_poses(const Mechanism& mech, std::span<const Part> parts,
                     const std::map<std::string, math::Mat4>& poses)
{
    Assembly a;
    for (const Part& part : parts)
    {
        Part placed = part;
        const auto it = poses.find(part.name);
        if (it != poses.end())
            placed = transform_part(part, it->second);
        a.parts.push_back(std::move(placed));
    }
    // bounds union (compressor/steam pattern)
    a.bounds = {};
    bool have = false;
    for (const Part& part : a.parts)
    {
        if (part.mesh.vertices.empty()) continue;
        const Bounds b = compute_bounds(part.mesh.vertices);
        if (!have) { a.bounds = b; have = true; }
        else
        {
            a.bounds.min.x = std::min(a.bounds.min.x, b.min.x);
            a.bounds.min.y = std::min(a.bounds.min.y, b.min.y);
            a.bounds.min.z = std::min(a.bounds.min.z, b.min.z);
            a.bounds.max.x = std::max(a.bounds.max.x, b.max.x);
            a.bounds.max.y = std::max(a.bounds.max.y, b.max.y);
            a.bounds.max.z = std::max(a.bounds.max.z, b.max.z);
        }
    }
    return a;
}

} // namespace exd::geometry

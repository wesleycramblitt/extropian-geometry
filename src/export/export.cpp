#include <exd/geometry/export.hpp>

#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace exd::geometry
{
namespace
{

std::string fmt(float v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", static_cast<double>(v));
    return buf;
}

std::string fmt_v(const math::Vec3f& v)
{
    return fmt(v.x) + " " + fmt(v.y) + " " + fmt(v.z);
}

std::string to_obj_inner(const MeshData& mesh)
{
    std::ostringstream os;
    for (const Vertex& v : mesh.vertices)
        os << "v " << v.position.x << ' ' << v.position.y << ' ' << v.position.z << '\n';
    for (const Vertex& v : mesh.vertices)
        os << "vn " << v.normal.x << ' ' << v.normal.y << ' ' << v.normal.z << '\n';
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3)
        os << "f " << (mesh.indices[t] + 1) << ' ' << (mesh.indices[t + 1] + 1) << ' '
           << (mesh.indices[t + 2] + 1) << '\n';
    return os.str();
}

/// Part lookup by name.
const Part* find_part(std::span<const Part> parts, const std::string& name)
{
    for (const Part& p : parts)
        if (p.name == name) return &p;
    return nullptr;
}

/// Inertial element text (body-local part mesh assumed).
std::string inertial_xml(const Part& p, float default_density, bool enabled)
{
    if (!enabled) return {};
    const float density = p.meta.density > 0.0f ? p.meta.density : default_density;
    const MassProperties mp = mesh_properties(p.mesh, density);
    if (mp.mass <= 0.0f) return {};
    std::ostringstream os;
    os << "<inertial pos=\"" << fmt_v(mp.centroid) << "\" mass=\"" << fmt(mp.mass)
       << "\" diaginertia=\"" << fmt(mp.inertia.m[0]) << ' ' << fmt(mp.inertia.m[4])
       << ' ' << fmt(mp.inertia.m[8]) << "\"/>\n";
    return os.str();
}

/// Geom element (mesh + optional contact group).
std::string geom_xml(const Part& p, const ExportOptions& options, const std::string& pad)
{
    std::ostringstream os;
    os << pad << "<geom type=\"mesh\" mesh=\"" << p.name << "\" group=\"1\"";
    if (options.emit_contact && p.meta.contact)
        os << " contype=\"1\" conaffinity=\"1\"";
    const float density = p.meta.density > 0.0f ? p.meta.density : options.default_density;
    os << " density=\"" << fmt(density) << "\"/>\n";
    return os.str();
}

/// Joint element on the child body (MJCF parent-local frames).
std::string joint_xml(const Joint& j, const std::string& pad)
{
    if (j.kind == JointKind::Fixed) return {};
    std::ostringstream os;
    os << pad << "<joint name=\"" << j.name << "\" type=\""
       << (j.kind == JointKind::Prismatic ? "slide" : "hinge")
       << "\" pos=\"0 0 0\" axis=\"" << fmt_v(j.axis.normalized()) << "\"";
    if (j.kind == JointKind::Revolute)
        os << " range=\"" << fmt(j.limit_low) << " " << fmt(j.limit_high) << "\"";
    if (j.stiffness != 0.0f)   os << " stiffness=\"" << fmt(j.stiffness) << "\"";
    if (j.damping != 0.0f)     os << " damping=\"" << fmt(j.damping) << "\"";
    if (j.armature != 0.0f)    os << " armature=\"" << fmt(j.armature) << "\"";
    if (j.frictionloss != 0.0f) os << " frictionloss=\"" << fmt(j.frictionloss) << "\"";
    os << "/>\n";
    return os.str();
}

} // namespace

std::string to_obj(const MeshData& mesh)
{
    return to_obj_inner(mesh);
}

std::string to_obj(const CADModel& model)
{
    std::ostringstream os;
    for (size_t i = 0; i < model.parts.size(); ++i)
    {
        const Part& p = model.parts[i];
        os << "o " << (p.name.empty() ? ("part_" + std::to_string(i)) : p.name) << "\n";
        os << to_obj_inner(p.mesh);
    }
    return os.str();
}

ExportBundle to_mjcf(const Mechanism& mech, std::span<const Part> parts,
                     const ExportOptions& options)
{
    ExportBundle bundle;
    for (const Part& p : parts)
        bundle.meshes[p.name] = to_obj_inner(p.mesh);

    // first incoming joint per part (tree) + loop-carrier joints
    std::unordered_map<std::string, const Joint*> childJoint;
    std::unordered_set<std::string> children;
    std::vector<const Joint*> loopJoints;   // 2nd+ incoming joints on a part
    for (const Joint& j : mech.joints)
    {
        if (j.child.empty()) continue;
        if (!childJoint.count(j.child))
            childJoint[j.child] = &j;
        else
            loopJoints.push_back(&j);
        children.insert(j.child);
    }

    // parent → children adjacency (tree emission order)
    std::unordered_map<std::string, std::vector<std::string>> childrenOf;
    for (const Joint& j : mech.joints)
        childrenOf[j.parent.empty() ? "" : j.parent].push_back(j.child);
    for (const Part& p : parts)
        if (!childJoint.count(p.name))
            childrenOf[""].push_back(p.name);
    for (auto& [k, v] : childrenOf)
        std::sort(v.begin(), v.end());

    std::ostringstream os;
    os << "<mujoco model=\"" << options.model_name << "\">\n";
    os << "  <compiler angle=\"radian\"/>\n";
    os << "  <asset>\n";
    for (const Part& p : parts)
        os << "    <mesh name=\"" << p.name << "\" file=\"" << p.name << ".obj\"/>\n";
    os << "  </asset>\n";
    os << "  <worldbody>\n";

    std::unordered_set<std::string> done;

    std::function<void(const std::string&, const std::string&)> emit_tree;
    emit_tree = [&](const std::string& parentName, const std::string& pad) {
        const auto it = childrenOf.find(parentName);
        if (it == childrenOf.end()) return;
        for (const std::string& k : it->second)
        {
            if (done.count(k)) continue;
            done.insert(k);
            const Part* p = find_part(parts, k);
            if (!p) continue;
            const Joint* j = nullptr;
            const auto jit = childJoint.find(k);
            if (jit != childJoint.end()) j = jit->second;

            os << pad << "<body name=\"" << k << "\" pos=\""
               << fmt_v(j ? j->anchor : math::Vec3f{0.0f, 0.0f, 0.0f}) << "\">\n";
            if (j) os << joint_xml(*j, pad + "  ");
            os << geom_xml(*p, options, pad + "  ");
            if (options.emit_inertials)
                os << pad + "  " << inertial_xml(*p, options.default_density, true);
            emit_tree(k, pad + "  ");
            os << pad << "</body>\n";
        }
    };

    emit_tree("", "  ");
    os << "  </worldbody>\n";

    // couplings (gears/belts/rack) + kinematic-loop point welds → one equality block
    if (!mech.couplings.empty() || !loopJoints.empty())
    {
        // MJCF connect anchors are GLOBAL: transform each loop joint's
        // parent-frame anchor by the parent's REST pose (state 0)
        const auto rest = evaluate_poses(mech, 0.0f);
        auto world_anchor = [&](const Joint& j) {
            const auto it = rest.find(j.parent);
            if (it == rest.end()) return j.anchor;   // static/unknown parent
            const math::Mat4& m = it->second;
            return math::Vec3f{m.m[0] * j.anchor.x + m.m[4] * j.anchor.y + m.m[8] * j.anchor.z + m.m[12],
                               m.m[1] * j.anchor.x + m.m[5] * j.anchor.y + m.m[9] * j.anchor.z + m.m[13],
                               m.m[2] * j.anchor.x + m.m[6] * j.anchor.y + m.m[10] * j.anchor.z + m.m[14]};
        };
        os << "  <equality>\n";
        for (const Coupling& c : mech.couplings)
            os << "    <joint joint1=\"" << c.joint_a << "\" joint2=\"" << c.joint_b
               << "\" polycoef=\"0 " << fmt(-c.ratio) << " 1\"/>\n";
        for (const Joint* j : loopJoints)
            os << "    <connect body1=\"" << j->parent << "\" body2=\"" << j->child
               << "\" anchor=\"" << fmt_v(world_anchor(*j)) << "\"/>\n";
        os << "  </equality>\n";
    }

    const Joint* dj = nullptr;
    for (const Joint& j : mech.joints)
        if (j.name == mech.driver_joint) { dj = &j; break; }
    if (dj && dj->kind != JointKind::Fixed && dj->effort_max < 1e29f)
    {
        os << "  <actuator>\n";
        os << "    <motor joint=\"" << mech.driver_joint << "\" ctrlrange=\""
           << fmt(-dj->effort_max) << ' ' << fmt(dj->effort_max) << "\"/>\n";
        os << "  </actuator>\n";
    }

    os << "</mujoco>\n";
    bundle.xml = os.str();
    return bundle;
}

ExportBundle to_urdf(const Mechanism& mech, std::span<const Part> parts,
                     const ExportOptions& options)
{
    ExportBundle bundle;
    for (const Part& p : parts)
        bundle.meshes[p.name] = to_obj_inner(p.mesh);

    // mimic targets: joint_b mimics joint_a at `ratio` (position-level gears)
    std::unordered_map<std::string, const Coupling*> mimics;
    for (const Coupling& c : mech.couplings)
        if (!mimics.count(c.joint_b))
            mimics[c.joint_b] = &c;

    std::ostringstream os;
    os << "<?xml version=\"1.0\"?>\n";
    os << "<robot name=\"" << options.model_name << "\">\n";

    static const char* kWorld = "world";
    for (const Part& p : parts)
    {
        os << "  <link name=\"" << p.name << "\">\n";
        if (options.emit_inertials)
        {
            const float density = p.meta.density > 0.0f ? p.meta.density : options.default_density;
            const MassProperties mp = mesh_properties(p.mesh, density);
            if (mp.mass > 0.0f)
            {
                os << "    <inertial>\n      <origin xyz=\"" << fmt_v(mp.centroid) << "\" rpy=\"0 0 0\"/>\n";
                os << "      <mass value=\"" << fmt(mp.mass) << "\"/>\n";
                os << "      <inertia ixx=\"" << fmt(mp.inertia.m[0])
                   << "\" ixy=\"" << fmt(mp.inertia.m[1])
                   << "\" ixz=\"" << fmt(mp.inertia.m[2])
                   << "\" iyy=\"" << fmt(mp.inertia.m[4])
                   << "\" iyz=\"" << fmt(mp.inertia.m[5])
                   << "\" izz=\"" << fmt(mp.inertia.m[8]) << "\"/>\n    </inertial>\n";
            }
        }
        os << "    <visual>\n      <geometry><mesh filename=\"" << p.name << ".obj\"/></geometry>\n    </visual>\n";
        if (options.emit_contact && p.meta.contact)
            os << "    <collision>\n      <geometry><mesh filename=\"" << p.name << ".obj\"/></geometry>\n    </collision>\n";
        os << "  </link>\n";
    }

    for (const Joint& j : mech.joints)
    {
        static const char* type[] = {"fixed", "revolute", "continuous", "prismatic"};
        os << "  <joint name=\"" << j.name << "\" type=\"" << type[static_cast<int>(j.kind)] << "\">\n";
        os << "    <parent link=\"" << (j.parent.empty() ? std::string(kWorld) : j.parent) << "\"/>\n";
        os << "    <child link=\"" << j.child << "\"/>\n";
        os << "    <origin xyz=\"" << fmt_v(j.anchor) << "\" rpy=\"0 0 0\"/>\n";
        if (j.kind != JointKind::Fixed)
        {
            os << "    <axis xyz=\"" << fmt_v(j.axis.normalized()) << "\"/>\n";
            os << "    <limit lower=\"" << fmt(j.limit_low) << "\" upper=\"" << fmt(j.limit_high)
               << "\" effort=\"" << fmt(j.effort_max) << "\" velocity=\"" << fmt(j.velocity_max) << "\"/>\n";
            if (j.damping != 0.0f || j.frictionloss != 0.0f || j.stiffness != 0.0f)
                os << "    <dynamics damping=\"" << fmt(j.damping) << "\" friction=\""
                   << fmt(j.frictionloss) << "\" stiffness=\"" << fmt(j.stiffness) << "\"/>\n";
        }
        const auto mit = mimics.find(j.name);
        if (mit != mimics.end())
            os << "    <mimic joint=\"" << mit->second->joint_a << "\" multiplier=\""
               << fmt(mit->second->ratio) << "\" offset=\"0\"/>\n";
        os << "  </joint>\n";
    }

    os << "</robot>\n";
    bundle.xml = os.str();
    return bundle;
}

ExportBundle to_mjcf(const CADModel& model, const ExportOptions& options)
{
    std::vector<Part> parts = model.parts;
    for (Part& p : parts)
        if (!p.meta.material.empty())
            if (const Material* mat = model.materials.find(p.meta.material); mat != nullptr)
                p.meta.density = mat->density;
    return to_mjcf(model.mechanism, parts, options);
}

ExportBundle to_urdf(const CADModel& model, const ExportOptions& options)
{
    std::vector<Part> parts = model.parts;
    for (Part& p : parts)
        if (!p.meta.material.empty())
            if (const Material* mat = model.materials.find(p.meta.material); mat != nullptr)
                p.meta.density = mat->density;
    return to_urdf(model.mechanism, parts, options);
}

} // namespace exd::geometry

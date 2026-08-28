#include <exd/geometry/compressor.hpp>

#include "../turbine/turbine_internal.hpp"

#include <exd/geometry/extrusion.hpp>
#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace exd::geometry
{
namespace
{

/// Append the three blade-row patches (blade_surface, hub_cap, shroud_cap)
/// for a row mesh built with the given per-blade build info, replicating the
/// stride expansion used by generate_turbine_assembly.
void add_blade_row_patches(Part& part, const detail::BladeRowBuildInfo& info)
{
    const uint32_t stride = info.stridePerBlade;
    if (stride == 0) return;

    const uint32_t blades = uint32_t(part.mesh.indices.size() / 3) / stride;
    std::vector<uint32_t> skinFaces, hubFaces, shroudFaces;
    for (uint32_t k = 0; k < blades; ++k) {
        for (uint32_t f = 0; f < info.skinPerBlade; ++f)
            skinFaces.push_back(k * stride + f);
        for (uint32_t f = 0; f < info.hubCapPerBlade; ++f)
            hubFaces.push_back(k * stride + info.skinPerBlade + f);
        for (uint32_t f = 0; f < info.shroudCapPerBlade; ++f)
            shroudFaces.push_back(k * stride + info.skinPerBlade + info.hubCapPerBlade + f);
    }
    part.patches.push_back({"blade_surface", std::move(skinFaces)});
    part.patches.push_back({"hub_cap", std::move(hubFaces)});
    part.patches.push_back({"shroud_cap", std::move(shroudFaces)});
}

} // namespace

Assembly generate_compressor_assembly(const CompressorDefinition& compressor)
{
    Assembly asm_;

    const bool emptyFlow = compressor.flow_path.hub_points.size() < 2 &&
                           compressor.flow_path.shroud_points.size() < 2;
    if (emptyFlow && compressor.stages.empty() && !compressor.has_igv &&
        compressor.spinner.shape == HubShape::None)
        return asm_;

    // ── spinner ──
    if (compressor.spinner.shape != HubShape::None) {
        MeshData mesh = generate_hub_mesh(compressor.spinner, compressor.revolve_segments);
        if (!mesh.vertices.empty()) {
            Part p = as_part("spinner", std::move(mesh));
            p.patches.push_back(make_patch_range("surface", 0, uint32_t(p.mesh.indices.size() / 3)));
            asm_.parts.push_back(std::move(p));
        }
    }

    // ── casing ──
    if (compressor.flow_path.shroud_points.size() >= 2) {
        std::vector<math::Vec3f> profile =
            detail::build_meridional_profile(compressor.flow_path.shroud_points);
        if (profile.size() >= 2) {
            LatheGeometry lathe;
            lathe.profile  = std::move(profile);
            lathe.axis     = LatheAxis::Z;
            lathe.segments = std::max(8u, compressor.revolve_segments);
            lathe.capped   = true;

            MeshData mesh = generate_lathe_mesh(lathe);
            if (!mesh.vertices.empty()) {
                Part p = as_part("casing", std::move(mesh));

                // Mirror generate_lathe_part's patch layout for the same lathe
                // description: side surface first, then the end caps.
                const uint32_t SEG       = std::max(3u, lathe.segments);
                const uint32_t nProf     = static_cast<uint32_t>(lathe.profile.size());
                const uint32_t sideCount = 2u * SEG * (nProf - 1u);
                p.patches.push_back(make_patch_range("surface", 0, sideCount));

                const bool hasBottom = lathe.capped && std::abs(lathe.profile.front().x) >= 1e-5f;
                const bool hasTop    = lathe.capped && std::abs(lathe.profile.back().x)  >= 1e-5f;
                if (hasBottom)
                    p.patches.push_back(make_patch_range("cap_start", sideCount, SEG));
                if (hasTop)
                    p.patches.push_back(make_patch_range("cap_end", sideCount + (hasBottom ? SEG : 0u), SEG));

                asm_.parts.push_back(std::move(p));
            }
        }
    }

    // ── igv ──
    if (compressor.has_igv) {
        detail::BladeRowBuildInfo info;
        MeshData mesh = detail::build_blade_row_impl(compressor.igv, compressor.flow_path,
                                                     compressor.revolve_segments, &info);
        if (!mesh.vertices.empty()) {
            Part p = as_part("igv", std::move(mesh));
            add_blade_row_patches(p, info);
            asm_.parts.push_back(std::move(p));
        }
    }

    // ── stages ──
    for (std::size_t i = 0; i < compressor.stages.size(); ++i) {
        const CompressorStage& stage = compressor.stages[i];

        detail::BladeRowBuildInfo rotorInfo;
        MeshData rotorMesh = detail::build_blade_row_impl(stage.rotor, compressor.flow_path,
                                                          compressor.revolve_segments, &rotorInfo);
        if (!rotorMesh.vertices.empty()) {
            Part p = as_part("rotor_" + std::to_string(i), std::move(rotorMesh));
            add_blade_row_patches(p, rotorInfo);
            asm_.parts.push_back(std::move(p));
        }

        detail::BladeRowBuildInfo statorInfo;
        MeshData statorMesh = detail::build_blade_row_impl(stage.stator, compressor.flow_path,
                                                           compressor.revolve_segments, &statorInfo);
        if (!statorMesh.vertices.empty()) {
            Part p = as_part("stator_" + std::to_string(i), std::move(statorMesh));
            add_blade_row_patches(p, statorInfo);
            asm_.parts.push_back(std::move(p));
        }
    }

    // Union of bounds over all part meshes (skip empty meshes; empty → Bounds{}).
    asm_.bounds = {};
    bool have = false;
    for (const Part& part : asm_.parts) {
        if (part.mesh.vertices.empty()) continue;
        const Bounds b = compute_bounds(part.mesh.vertices);
        if (!have) {
            asm_.bounds = b;
            have = true;
        } else {
            asm_.bounds.min.x = std::min(asm_.bounds.min.x, b.min.x);
            asm_.bounds.min.y = std::min(asm_.bounds.min.y, b.min.y);
            asm_.bounds.min.z = std::min(asm_.bounds.min.z, b.min.z);
            asm_.bounds.max.x = std::max(asm_.bounds.max.x, b.max.x);
            asm_.bounds.max.y = std::max(asm_.bounds.max.y, b.max.y);
            asm_.bounds.max.z = std::max(asm_.bounds.max.z, b.max.z);
        }
    }

    return asm_;
}

MeshData generate_compressor_mesh(const CompressorDefinition& compressor)
{
    const Assembly a = generate_compressor_assembly(compressor);
    if (a.parts.empty()) return {};
    return flatten(a).mesh;
}

} // namespace exd::geometry
#pragma once

#include <exd/geometry/part.hpp>
#include <exd/geometry/turbine.hpp>

#include <cstdint>
#include <vector>

namespace exd::geometry
{

/// One compressor stage: a rotor row plus its counter-rotating/stationary
/// stator row, both defined in the turbine blade-row vocabulary (camber,
/// stagger, sweep, lean, section stacks…). For the compressor SENSE, set the
/// rotor's stagger/camber so the suction side faces -theta (fan sense), or
/// spin the rotor in the fan direction — see BladeSection::stagger docs.
struct CompressorStage
{
    BladeRow rotor;    // type = Rotor
    BladeRow stator;   // type = Stator
};

/// Axial compressor machine. Parts produced by generate_compressor_assembly
/// (in order): "spinner" (HubShape != None; patch "surface"), "casing"
/// (shroud meridional lathe; patches "surface", and "cap_start"/"cap_end"
/// annuli when the ends are off-axis), optional "igv", then per stage
/// "rotor_<i>" and "stator_<i>" (patches "blade_surface", "hub_cap",
/// "shroud_cap"). The casing follows the shroud control-point spline, so it
/// mates with the meridional flow path.
struct CompressorDefinition
{
    FlowPath flow_path;
    std::vector<CompressorStage> stages;   // may be empty (flow-only machine)
    bool   has_igv = false;
    BladeRow igv;                          // used when has_igv
    HubDefinition spinner;                 // HubShape::None → no spinner part
    uint32_t revolve_segments = 64;
};

/// Assemble the compressor as a patched Assembly (see CompressorDefinition
/// for part/patch layout). Empty flow path AND no stages → empty Assembly.
Assembly generate_compressor_assembly(const CompressorDefinition& compressor);

/// Convenience: flatten(generate_compressor_assembly(...)).mesh (empty if
/// the assembly is empty).
MeshData generate_compressor_mesh(const CompressorDefinition& compressor);

} // namespace exd::geometry
#pragma once

#include <exd/core/mesh_types.hpp>
#include <exd/math/vec3.hpp>
#include <exd/math/quat.hpp>

#include <cstdint>
#include <vector>

namespace exd::geometry
{

// Re-export core types for backward compatibility.
using Vertex            = exd::core::Vertex;
using PrimitiveTopology = exd::core::PrimitiveTopology;
using Bounds            = exd::core::Bounds;
using MeshData          = exd::core::MeshData;

// ── Vertex semantics (for stream-based / SoA mesh data) ──

enum class VertexSemantic
{
    Position,
    Normal,
    Tangent,
    TexCoord0,
    Color0,
    Custom0
};

enum class ComponentType
{
    Float32,
    Uint32,
    Int32,
    Uint16,
    Uint8
};

enum class IndexType
{
    Uint16,
    Uint32
};

/// A single vertex attribute stream (SoA layout).
struct VertexStream
{
    VertexSemantic semantic;
    ComponentType  componentType;
    uint32_t       componentCount;
    std::vector<std::byte> data;
};

/// Index data for a mesh.
struct IndexData
{
    IndexType type = IndexType::Uint32;
    std::vector<std::byte> data;
};

} // namespace exd::geometry

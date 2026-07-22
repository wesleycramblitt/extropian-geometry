#pragma once

#include <exd/math/vec3.hpp>
#include <exd/math/quat.hpp>

#include <cstdint>
#include <vector>

namespace exd::geometry
{

// ── Vertex ──

/// Canonical interleaved vertex layout.
/// Shared across the ecosystem — both Canvas and Renderer consume this type.
struct Vertex
{
    math::Vec3f position = {0.0f, 0.0f, 0.0f};
    math::Vec3f normal   = {0.0f, 1.0f, 0.0f};
    math::Vec3f uv       = {0.0f, 0.0f, 0.0f};
    math::Quat  tangent  = {1.0f, 0.0f, 0.0f, 1.0f};  // identity
    math::Quat  color    = {0.8f, 0.8f, 0.8f, 1.0f};  // RGBA
};

// ── Topology ──

enum class PrimitiveTopology
{
    Points,
    Lines,
    LineStrip,
    Triangles,
    TriangleStrip
};

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

// ── Bounds ──

struct Bounds
{
    math::Vec3f min = {0.0f, 0.0f, 0.0f};
    math::Vec3f max = {0.0f, 0.0f, 0.0f};
};

// ── MeshData ──

/// Canonical CPU-side mesh data used by geometry generators.
/// Renderer maps this to GPU buffers.
struct MeshData
{
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    PrimitiveTopology topology = PrimitiveTopology::Triangles;
};

} // namespace exd::geometry

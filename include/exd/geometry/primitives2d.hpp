#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

#include <numbers>
#include <vector>

namespace exd::geometry
{

// ── 2D primitive geometry descriptors ──

struct CornerRadii
{
    float topLeft     = 0.0f;
    float topRight    = 0.0f;
    float bottomRight = 0.0f;
    float bottomLeft  = 0.0f;
};

struct RectangleGeometry
{
    math::Vec3f size = {1.0f, 1.0f, 0.0f};
};

struct RoundedRectangleGeometry
{
    math::Vec3f size = {1.0f, 1.0f, 0.0f};
    CornerRadii radii;
    uint32_t cornerSegments = 16;
};

struct CircleGeometry
{
    float radius = 1.0f;
    uint32_t segments = 32;
};

struct EllipseGeometry
{
    float radiusX = 1.0f;
    float radiusY = 0.5f;
    uint32_t segments = 32;
};

struct ArcGeometry
{
    float radius = 1.0f;
    float startAngle = 0.0f;
    float endAngle = std::numbers::pi_v<float> * 1.5f;
    uint32_t segments = 32;
};

struct RingGeometry
{
    float outerRadius = 1.0f;
    float innerRadius = 0.5f;
    uint32_t segments = 32;
};

struct LineGeometry
{
    math::Vec3f start = {0.0f, 0.0f, 0.0f};
    math::Vec3f end   = {1.0f, 0.0f, 0.0f};
    float width = 1.0f;
};

struct PolylineGeometry
{
    std::vector<math::Vec3f> points;
    float width = 1.0f;
    bool closed = false;
};

struct ArrowGeometry
{
    math::Vec3f start = {0.0f, 0.0f, 0.0f};
    math::Vec3f end   = {1.0f, 0.0f, 0.0f};
    float headLength  = 0.2f;
    float headWidth   = 0.1f;
    float shaftWidth  = 0.02f;
};

struct GridGeometry
{
    math::Vec3f size = {1.0f, 1.0f, 0.0f};
    uint32_t rows = 10;
    uint32_t columns = 10;
    float lineWidth = 0.01f;
};

} // namespace exd::geometry

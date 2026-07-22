#pragma once

#include <exd/geometry/types.hpp>
#include <exd/math/vec3.hpp>

#include <memory>
#include <numbers>
#include <vector>

namespace exd::geometry
{

// ── Line join / cap ──

enum class LineJoin { Miter, Round, Bevel };
enum class LineCap  { Butt, Round, Square };
enum class FillRule { NonZero, EvenOdd };

// ── Arc descriptor ──

struct ArcDescriptor
{
    math::Vec3f center     = {0.0f, 0.0f, 0.0f};
    float       radius     = 1.0f;
    float       startAngle = 0.0f;
    float       endAngle   = std::numbers::pi_v<float> * 2.0f;
    bool        counterClockwise = false;
};

// ── Stroke style ──

struct StrokeStyle
{
    float     width      = 1.0f;
    LineJoin  join       = LineJoin::Miter;
    LineCap   cap        = LineCap::Butt;
    float     miterLimit = 4.0f;
    std::vector<float> dashPattern;
    float     dashOffset = 0.0f;
};

// ── Path2D ──

/// Vector path builder (SVG-like API).
/// Produces path commands that can be tessellated into MeshData.
class Path2D
{
public:
    Path2D();
    ~Path2D();

    Path2D(const Path2D&) = delete;
    Path2D& operator=(const Path2D&) = delete;
    Path2D(Path2D&&) noexcept;
    Path2D& operator=(Path2D&&) noexcept;

    Path2D& moveTo(math::Vec3f p);
    Path2D& lineTo(math::Vec3f p);
    Path2D& quadraticTo(math::Vec3f control, math::Vec3f end);
    Path2D& cubicTo(math::Vec3f c0, math::Vec3f c1, math::Vec3f end);
    Path2D& arcTo(const ArcDescriptor& arc);
    Path2D& close();

    MeshData tessellateFill(FillRule rule = FillRule::NonZero,
                            float tolerance = 0.1f) const;
    MeshData tessellateStroke(const StrokeStyle& style,
                              float tolerance = 0.1f) const;

    uint64_t revision() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace exd::geometry

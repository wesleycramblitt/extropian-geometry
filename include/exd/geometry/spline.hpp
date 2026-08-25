#pragma once

#include <cstddef>
#include <vector>

namespace exd::geometry
{

/// Monotone cubic (Fritsch–Carlson) interpolating spline over 1D data.
///
/// Interpolates a set of (x, y) control points with C1 continuity and, unlike
/// a plain cubic spline, never overshoots between points: it preserves the
/// monotonicity of the data. This makes it the right curve for flow-path
/// profiles such as hub/shroud radius r(z), which must stay single-valued and
/// smooth.
///
/// Construct with strictly increasing x values (duplicates are coalesced).
class MonotoneCubicSpline
{
public:
    MonotoneCubicSpline() = default;
    MonotoneCubicSpline(std::vector<float> xs, std::vector<float> ys);

    [[nodiscard]] bool valid() const noexcept { return !x_.empty(); }
    [[nodiscard]] std::size_t point_count() const noexcept { return x_.size(); }

    /// Evaluate the spline at x (clamped to [min_x, max_x]).
    [[nodiscard]] float evaluate(float x) const;

    /// First derivative dy/dx (clamped to [min_x, max_x]).
    [[nodiscard]] float derivative(float x) const;

    [[nodiscard]] float min_x() const noexcept { return x_.empty() ? 0.0f : x_.front(); }
    [[nodiscard]] float max_x() const noexcept { return x_.empty() ? 0.0f : x_.back(); }

private:
    std::vector<float> x_;
    std::vector<float> y_;
    std::vector<float> m_;   // per-node tangent slopes
};

} // namespace exd::geometry

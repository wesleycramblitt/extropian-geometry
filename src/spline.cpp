#include <exd/geometry/spline.hpp>

#include <algorithm>
#include <cassert>

namespace exd::geometry
{

MonotoneCubicSpline::MonotoneCubicSpline(std::vector<float> xs, std::vector<float> ys)
{
    assert(xs.size() == ys.size());
    if (xs.empty()) return;

    // Coalesce duplicate x values (keep the last y).
    x_.push_back(xs.front());
    y_.push_back(ys.front());
    for (std::size_t i = 1; i < xs.size(); ++i) {
        if (xs[i] == x_.back()) {
            y_.back() = ys[i];
        } else {
            x_.push_back(xs[i]);
            y_.push_back(ys[i]);
        }
    }

    const std::size_t n = x_.size();
    m_.resize(n);
    if (n == 1) { m_[0] = 0.0f; return; }
    if (n == 2) {
        const float d = (y_[1] - y_[0]) / (x_[1] - x_[0]);
        m_[0] = d;
        m_[1] = d;
        return;
    }

    // Secant slopes per interval.
    std::vector<float> d(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i)
        d[i] = (y_[i + 1] - y_[i]) / (x_[i + 1] - x_[i]);

    // Interior slopes via Fritsch–Carlson (monotonicity-preserving).
    for (std::size_t i = 1; i + 1 < n; ++i) {
        if (d[i - 1] * d[i] <= 0.0f) {
            m_[i] = 0.0f;                       // local extremum
        } else {
            const float h0 = x_[i] - x_[i - 1];
            const float h1 = x_[i + 1] - x_[i];
            const float w0 = 2.0f * h1 + h0;
            const float w1 = h1 + 2.0f * h0;
            m_[i] = (w0 + w1) / (w0 / d[i - 1] + w1 / d[i]);
        }
    }
    // End slopes: one-sided, then clamp to preserve monotonicity at the ends.
    m_[0] = d[0];
    m_[n - 1] = d[n - 2];
}

float MonotoneCubicSpline::evaluate(float x) const
{
    if (!valid()) return 0.0f;
    if (x_.size() == 1) return y_[0];

    const float clamped = std::clamp(x, x_.front(), x_.back());
    const std::size_t n = x_.size();

    std::size_t i = 0;
    if (clamped >= x_.back()) {
        i = n - 2;
    } else {
        i = static_cast<std::size_t>(
            std::upper_bound(x_.begin(), x_.end(), clamped) - x_.begin());
        if (i == 0) i = 0;
        else if (i >= n) i = n - 2;
        else --i;
    }

    const float h = x_[i + 1] - x_[i];
    const float t = (clamped - x_[i]) / h;
    const float t2 = t * t;
    const float t3 = t2 * t;

    const float h00 =  2.0f * t3 - 3.0f * t2 + 1.0f;
    const float h10 =        t3 - 2.0f * t2 + t;
    const float h01 = -2.0f * t3 + 3.0f * t2;
    const float h11 =        t3 -       t2;

    return h00 * y_[i] + h10 * h * m_[i] + h01 * y_[i + 1] + h11 * h * m_[i + 1];
}

float MonotoneCubicSpline::derivative(float x) const
{
    if (!valid()) return 0.0f;
    if (x_.size() == 1) return 0.0f;

    const float clamped = std::clamp(x, x_.front(), x_.back());
    const std::size_t n = x_.size();

    std::size_t i = 0;
    if (clamped >= x_.back()) {
        i = n - 2;
    } else {
        i = static_cast<std::size_t>(
            std::upper_bound(x_.begin(), x_.end(), clamped) - x_.begin());
        if (i == 0) i = 0;
        else if (i >= n) i = n - 2;
        else --i;
    }

    const float h = x_[i + 1] - x_[i];
    const float t = (clamped - x_[i]) / h;
    const float t2 = t * t;

    const float dh00 =  6.0f * t2 - 6.0f * t;
    const float dh10 =  3.0f * t2 - 4.0f * t + 1.0f;
    const float dh01 = -6.0f * t2 + 6.0f * t;
    const float dh11 =  3.0f * t2 - 2.0f * t;

    return (dh00 * y_[i] + dh10 * h * m_[i] + dh01 * y_[i + 1] + dh11 * h * m_[i + 1]) / h;
}

} // namespace exd::geometry

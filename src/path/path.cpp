#include <exd/geometry/path.hpp>
#include <exd/geometry/mesh_builder.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <utility>
#include <variant>
#include <vector>

namespace exd::geometry
{

// ============================================================================
// SECTION 1: Command Types and Storage
// ============================================================================

namespace
{

struct MoveToCmd  { math::Vec3f p; };
struct LineToCmd  { math::Vec3f p; };
struct QuadToCmd  { math::Vec3f control, end; };
struct CubicToCmd { math::Vec3f c0, c1, end; };
struct ArcToCmd   { ArcDescriptor arc; };
struct CloseCmd   {};

using PathCmd = std::variant<MoveToCmd, LineToCmd, QuadToCmd,
                              CubicToCmd, ArcToCmd, CloseCmd>;

} // anonymous namespace

struct Path2D::Impl
{
    uint64_t revision = 0;
    std::vector<PathCmd> commands;
    math::Vec3f currentPoint{0.0f, 0.0f, 0.0f};
    math::Vec3f subpathStart{0.0f, 0.0f, 0.0f};
};

Path2D::Path2D() : impl_(std::make_unique<Impl>()) {}
Path2D::~Path2D() = default;
Path2D::Path2D(Path2D&&) noexcept = default;
Path2D& Path2D::operator=(Path2D&&) noexcept = default;

Path2D& Path2D::moveTo(math::Vec3f p)
{
    impl_->commands.emplace_back(MoveToCmd{p});
    impl_->currentPoint = p;
    impl_->subpathStart = p;
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::lineTo(math::Vec3f p)
{
    impl_->commands.emplace_back(LineToCmd{p});
    impl_->currentPoint = p;
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::quadraticTo(math::Vec3f control, math::Vec3f end)
{
    impl_->commands.emplace_back(QuadToCmd{control, end});
    impl_->currentPoint = end;
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::cubicTo(math::Vec3f c0, math::Vec3f c1, math::Vec3f end)
{
    impl_->commands.emplace_back(CubicToCmd{c0, c1, end});
    impl_->currentPoint = end;
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::arcTo(const ArcDescriptor& arc)
{
    impl_->commands.emplace_back(ArcToCmd{arc});
    // Update currentPoint to arc endpoint
    impl_->currentPoint = {
        arc.center.x + arc.radius * std::cos(arc.endAngle),
        arc.center.y + arc.radius * std::sin(arc.endAngle),
        arc.center.z
    };
    ++impl_->revision;
    return *this;
}

Path2D& Path2D::close()
{
    impl_->commands.emplace_back(CloseCmd{});
    impl_->currentPoint = impl_->subpathStart;
    ++impl_->revision;
    return *this;
}

uint64_t Path2D::revision() const noexcept
{
    return impl_->revision;
}

// ============================================================================
// SECTION 2: Curve Flattening
// ============================================================================

namespace
{

using FlattenedSeg = std::pair<math::Vec3f, math::Vec3f>;

// ── Point-to-line distance ──────────────────────────────────────────────────

float point_to_line_distance(const math::Vec3f& p,
                             const math::Vec3f& a,
                             const math::Vec3f& b)
{
    math::Vec3f ab = b - a;
    float lenSq = ab.dot(ab);
    if (lenSq < 1e-12f) return (p - a).length();
    float t = std::max(0.0f, std::min(1.0f, (p - a).dot(ab) / lenSq));
    math::Vec3f proj = a + ab * t;
    return (p - proj).length();
}

// ── Quadratic Bezier flattening (de Casteljau subdivision) ──────────────────

void flatten_quadratic(const math::Vec3f& p0,
                       const math::Vec3f& c,
                       const math::Vec3f& p2,
                       float tolerance,
                       std::vector<FlattenedSeg>& out)
{
    // Midpoint of the quadratic at t=0.5: (p0 + 2c + p2) / 4
    math::Vec3f mid = {
        (p0.x + 2.0f * c.x + p2.x) / 4.0f,
        (p0.y + 2.0f * c.y + p2.y) / 4.0f,
        (p0.z + 2.0f * c.z + p2.z) / 4.0f
    };
    float dist = point_to_line_distance(mid, p0, p2);

    if (dist <= tolerance) {
        out.emplace_back(p0, p2);
    } else {
        // De Casteljau at t=0.5
        math::Vec3f q0 = p0;
        math::Vec3f q1 = {(p0.x + c.x) / 2.0f, (p0.y + c.y) / 2.0f, (p0.z + c.z) / 2.0f};
        math::Vec3f q2 = mid;
        math::Vec3f r0 = mid;
        math::Vec3f r1 = {(c.x + p2.x) / 2.0f, (c.y + p2.y) / 2.0f, (c.z + p2.z) / 2.0f};
        math::Vec3f r2 = p2;
        flatten_quadratic(q0, q1, q2, tolerance, out);
        flatten_quadratic(r0, r1, r2, tolerance, out);
    }
}

// ── Cubic Bezier flattening (de Casteljau subdivision) ──────────────────────

void flatten_cubic(const math::Vec3f& p0,
                   const math::Vec3f& c0,
                   const math::Vec3f& c1,
                   const math::Vec3f& p3,
                   float tolerance,
                   std::vector<FlattenedSeg>& out)
{
    float d1 = point_to_line_distance(c0, p0, p3);
    float d2 = point_to_line_distance(c1, p0, p3);

    if (std::max(d1, d2) <= tolerance) {
        out.emplace_back(p0, p3);
    } else {
        // De Casteljau at t=0.5
        math::Vec3f p01  = {(p0.x + c0.x) / 2.0f, (p0.y + c0.y) / 2.0f, (p0.z + c0.z) / 2.0f};
        math::Vec3f p12  = {(c0.x + c1.x) / 2.0f, (c0.y + c1.y) / 2.0f, (c0.z + c1.z) / 2.0f};
        math::Vec3f p23  = {(c1.x + p3.x) / 2.0f, (c1.y + p3.y) / 2.0f, (c1.z + p3.z) / 2.0f};
        math::Vec3f p012 = {(p01.x + p12.x) / 2.0f, (p01.y + p12.y) / 2.0f, (p01.z + p12.z) / 2.0f};
        math::Vec3f p123 = {(p12.x + p23.x) / 2.0f, (p12.y + p23.y) / 2.0f, (p12.z + p23.z) / 2.0f};
        math::Vec3f p0123 = {(p012.x + p123.x) / 2.0f, (p012.y + p123.y) / 2.0f, (p012.z + p123.z) / 2.0f};

        flatten_cubic(p0, p01, p012, p0123, tolerance, out);
        flatten_cubic(p0123, p123, p23, p3, tolerance, out);
    }
}

// ── Arc flattening ──────────────────────────────────────────────────────────

void flatten_arc(const ArcDescriptor& arc,
                 float tolerance,
                 std::vector<FlattenedSeg>& out)
{
    float sweep = arc.endAngle - arc.startAngle;
    if (std::abs(sweep) < 1e-6f) return;

    float absSweep = std::abs(sweep);
    // Number of segments: chord height ~ r * dtheta^2 / 8
    float chordAngle = 2.0f * std::acos(std::max(0.0f, 1.0f - tolerance / arc.radius));
    if (chordAngle < 1e-6f) chordAngle = 0.1f; // fallback
    int segs = std::max(1, (int)std::ceil(absSweep / chordAngle));
    float step = sweep / (float)segs;

    math::Vec3f prev = {
        arc.center.x + arc.radius * std::cos(arc.startAngle),
        arc.center.y + arc.radius * std::sin(arc.startAngle),
        arc.center.z
    };
    for (int i = 1; i <= segs; ++i) {
        float a = arc.startAngle + step * (float)i;
        math::Vec3f next = {
            arc.center.x + arc.radius * std::cos(a),
            arc.center.y + arc.radius * std::sin(a),
            arc.center.z
        };
        out.emplace_back(prev, next);
        prev = next;
    }
}

// ── Main flatten: walk commands and produce linear segments ─────────────────

std::vector<FlattenedSeg> flatten_commands(
    const std::vector<PathCmd>& commands, float tolerance)
{
    std::vector<FlattenedSeg> segments;
    math::Vec3f current{0, 0, 0};

    for (const auto& cmd : commands) {
        std::visit([&](const auto& c) {
            using T = std::decay_t<decltype(c)>;
            if constexpr (std::is_same_v<T, MoveToCmd>) {
                current = c.p;
            } else if constexpr (std::is_same_v<T, LineToCmd>) {
                segments.emplace_back(current, c.p);
                current = c.p;
            } else if constexpr (std::is_same_v<T, QuadToCmd>) {
                flatten_quadratic(current, c.control, c.end, tolerance, segments);
                current = c.end;
            } else if constexpr (std::is_same_v<T, CubicToCmd>) {
                flatten_cubic(current, c.c0, c.c1, c.end, tolerance, segments);
                current = c.end;
            } else if constexpr (std::is_same_v<T, ArcToCmd>) {
                flatten_arc(c.arc, tolerance, segments);
                current = {
                    c.arc.center.x + c.arc.radius * std::cos(c.arc.endAngle),
                    c.arc.center.y + c.arc.radius * std::sin(c.arc.endAngle),
                    c.arc.center.z
                };
            } else if constexpr (std::is_same_v<T, CloseCmd>) {
                // Close is handled at the polygon construction level
            }
        }, cmd);
    }
    return segments;
}

} // anonymous namespace

// ============================================================================
// SECTION 3: Polygon Construction
// ============================================================================

namespace
{

struct Polygon {
    std::vector<math::Vec3f> vertices; // ordered (CCW or CW)
};

std::vector<Polygon> build_polygons(const std::vector<PathCmd>& commands, float tolerance)
{
    std::vector<Polygon> polygons;
    Polygon currentPoly;

    math::Vec3f subpathStart{0, 0, 0};
    math::Vec3f currentPoint{0, 0, 0};
    bool hasSubpath = false;

    for (const auto& cmd : commands) {
        std::visit([&](const auto& c) {
            using T = std::decay_t<decltype(c)>;
            if constexpr (std::is_same_v<T, MoveToCmd>) {
                if (!currentPoly.vertices.empty()) {
                    polygons.push_back(std::move(currentPoly));
                    currentPoly = Polygon{};
                }
                currentPoly.vertices.push_back(c.p);
                currentPoint = c.p;
                subpathStart = c.p;
                hasSubpath = true;
            } else if constexpr (std::is_same_v<T, LineToCmd>) {
                if (!hasSubpath) {
                    currentPoly.vertices.push_back(currentPoint);
                    hasSubpath = true;
                }
                currentPoly.vertices.push_back(c.p);
                currentPoint = c.p;
            } else if constexpr (std::is_same_v<T, QuadToCmd>) {
                if (!hasSubpath) {
                    currentPoly.vertices.push_back(currentPoint);
                    hasSubpath = true;
                }
                // Flatten and add interior points
                std::vector<FlattenedSeg> temp;
                flatten_quadratic(currentPoint, c.control, c.end, tolerance, temp);
                for (const auto& seg : temp) {
                    currentPoly.vertices.push_back(seg.second);
                }
                currentPoint = c.end;
            } else if constexpr (std::is_same_v<T, CubicToCmd>) {
                if (!hasSubpath) {
                    currentPoly.vertices.push_back(currentPoint);
                    hasSubpath = true;
                }
                std::vector<FlattenedSeg> temp;
                flatten_cubic(currentPoint, c.c0, c.c1, c.end, tolerance, temp);
                for (const auto& seg : temp) {
                    currentPoly.vertices.push_back(seg.second);
                }
                currentPoint = c.end;
            } else if constexpr (std::is_same_v<T, ArcToCmd>) {
                if (!hasSubpath) {
                    currentPoly.vertices.push_back(currentPoint);
                    hasSubpath = true;
                }
                std::vector<FlattenedSeg> temp;
                flatten_arc(c.arc, tolerance, temp);
                for (const auto& seg : temp) {
                    currentPoly.vertices.push_back(seg.second);
                }
                currentPoint = {
                    c.arc.center.x + c.arc.radius * std::cos(c.arc.endAngle),
                    c.arc.center.y + c.arc.radius * std::sin(c.arc.endAngle),
                    c.arc.center.z
                };
            } else if constexpr (std::is_same_v<T, CloseCmd>) {
                // Don't add subpathStart as a vertex — ear clipping implicitly
                // closes the loop via modular indexing. Just update currentPoint.
                if (hasSubpath) {
                    currentPoint = subpathStart;
                }
            }
        }, cmd);
    }
    if (!currentPoly.vertices.empty()) {
        polygons.push_back(std::move(currentPoly));
    }

    // Remove duplicate consecutive vertices
    for (auto& poly : polygons) {
        std::vector<math::Vec3f> cleaned;
        for (const auto& v : poly.vertices) {
            if (cleaned.empty()) {
                cleaned.push_back(v);
            } else {
                auto& last = cleaned.back();
                float dx = v.x - last.x;
                float dy = v.y - last.y;
                float dz = v.z - last.z;
                if (dx * dx + dy * dy + dz * dz > 1e-12f) {
                    cleaned.push_back(v);
                }
            }
        }
        // Also remove duplicate last vertex if it equals the first
        if (cleaned.size() >= 2) {
            auto& first = cleaned.front();
            auto& last = cleaned.back();
            float dx = last.x - first.x;
            float dy = last.y - first.y;
            float dz = last.z - first.z;
            if (dx * dx + dy * dy + dz * dz < 1e-12f) {
                cleaned.pop_back();
            }
        }
        poly.vertices = std::move(cleaned);
    }

    // Remove degenerate polygons (< 3 vertices)
    polygons.erase(
        std::remove_if(polygons.begin(), polygons.end(),
            [](const Polygon& p) { return p.vertices.size() < 3; }),
        polygons.end());

    return polygons;
}

} // anonymous namespace

// ============================================================================
// SECTION 4: Ear-Clipping Triangulation
// ============================================================================

namespace
{

// Cross-product Z-component sign test for convexity in 2D
bool is_convex(const math::Vec3f& a, const math::Vec3f& b,
               const math::Vec3f& c, bool ccw)
{
    float cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    return ccw ? (cross > 0.0f) : (cross < 0.0f);
}

// Barycentric point-in-triangle test (2D, XY plane)
bool point_in_triangle(const math::Vec3f& p,
                       const math::Vec3f& a,
                       const math::Vec3f& b,
                       const math::Vec3f& c)
{
    float v0x = c.x - a.x, v0y = c.y - a.y;
    float v1x = b.x - a.x, v1y = b.y - a.y;
    float v2x = p.x - a.x, v2y = p.y - a.y;

    float dot00 = v0x * v0x + v0y * v0y;
    float dot01 = v0x * v1x + v0y * v1y;
    float dot02 = v0x * v2x + v0y * v2y;
    float dot11 = v1x * v1x + v1y * v1y;
    float dot12 = v1x * v2x + v1y * v2y;

    float denom = dot00 * dot11 - dot01 * dot01;
    if (std::abs(denom) < 1e-12f) return false;

    float u = (dot11 * dot02 - dot01 * dot12) / denom;
    float v = (dot00 * dot12 - dot01 * dot02) / denom;

    return (u >= 0.0f) && (v >= 0.0f) && (u + v <= 1.0f);
}

// Signed area test: positive = CCW
bool is_ccw(const std::vector<math::Vec3f>& poly)
{
    float area = 0.0f;
    size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += poly[i].x * poly[j].y - poly[j].x * poly[i].y;
    }
    return area > 0.0f;
}

// Returns triangles as index triples into the original polygon vertex array
std::vector<std::array<size_t, 3>> triangulate_ear_clip(std::vector<math::Vec3f> poly)
{
    std::vector<std::array<size_t, 3>> triangles;
    if (poly.size() < 3) return triangles;
    if (poly.size() == 3) {
        triangles.push_back({0, 1, 2});
        return triangles;
    }

    bool ccw = is_ccw(poly);

    // Working index list — vertices removed as ears are clipped
    std::vector<size_t> indices(poly.size());
    for (size_t i = 0; i < poly.size(); ++i) indices[i] = i;

    size_t safetyCounter = 0;
    const size_t maxIterations = poly.size() * poly.size(); // guard against infinite loops

    while (indices.size() > 3 && safetyCounter < maxIterations) {
        ++safetyCounter;
        bool foundEar = false;
        size_t n = indices.size();

        for (size_t i = 0; i < n; ++i) {
            size_t prevIdx = indices[(i + n - 1) % n];
            size_t currIdx = indices[i];
            size_t nextIdx = indices[(i + 1) % n];

            if (!is_convex(poly[prevIdx], poly[currIdx], poly[nextIdx], ccw))
                continue;

            // Check that no other vertex is inside the candidate ear triangle
            bool ear = true;
            for (size_t j = 0; j < n; ++j) {
                if (j == (i + n - 1) % n || j == i || j == (i + 1) % n) continue;
                if (point_in_triangle(poly[indices[j]],
                                      poly[prevIdx], poly[currIdx], poly[nextIdx])) {
                    ear = false;
                    break;
                }
            }
            if (ear) {
                triangles.push_back({prevIdx, currIdx, nextIdx});
                indices.erase(indices.begin() + (int)i);
                foundEar = true;
                break;
            }
        }
        if (!foundEar) break; // degenerate or self-intersecting polygon
    }

    if (indices.size() == 3) {
        triangles.push_back({indices[0], indices[1], indices[2]});
    }

    return triangles;
}

} // anonymous namespace

// ============================================================================
// SECTION 5: Fill Tessellation
// ============================================================================

MeshData Path2D::tessellateFill(FillRule /*rule*/, float tolerance) const
{
    if (impl_->commands.empty()) return {};

    auto polygons = build_polygons(impl_->commands, tolerance);
    if (polygons.empty()) return {};

    MeshBuilder builder;

    for (const auto& poly : polygons) {
        auto tris = triangulate_ear_clip(poly.vertices);
        if (tris.empty()) continue;

        // Map original polygon vertex indices to MeshBuilder vertex indices
        std::vector<uint32_t> vertIndices(poly.vertices.size());
        for (size_t i = 0; i < poly.vertices.size(); ++i) {
            Vertex v;
            v.position = poly.vertices[i];
            v.normal   = {0.0f, 0.0f, 1.0f}; // fill faces point +Z
            vertIndices[i] = builder.add_vertex(v);
        }

        bool ccw = is_ccw(poly.vertices);
        for (const auto& tri : tris) {
            if (ccw) {
                builder.add_triangle(vertIndices[tri[0]],
                                     vertIndices[tri[1]],
                                     vertIndices[tri[2]]);
            } else {
                // Reverse winding for CW polygons
                builder.add_triangle(vertIndices[tri[0]],
                                     vertIndices[tri[2]],
                                     vertIndices[tri[1]]);
            }
        }
    }

    return builder.build();
}

// ============================================================================
// SECTION 6: Stroke Tessellation
// ============================================================================

namespace
{

struct Subpath {
    std::vector<math::Vec3f> points;
    bool closed = false;
};

std::vector<Subpath> build_subpaths(const std::vector<PathCmd>& commands, float tolerance)
{
    std::vector<Subpath> subpaths;
    Subpath current;
    math::Vec3f subpathStart{0, 0, 0};
    math::Vec3f currentPoint{0, 0, 0};
    bool hasSubpath = false;

    for (const auto& cmd : commands) {
        std::visit([&](const auto& c) {
            using T = std::decay_t<decltype(c)>;
            if constexpr (std::is_same_v<T, MoveToCmd>) {
                if (!current.points.empty()) {
                    subpaths.push_back(std::move(current));
                    current = Subpath{};
                }
                current.points.push_back(c.p);
                currentPoint = c.p;
                subpathStart = c.p;
                hasSubpath = true;
            } else if constexpr (std::is_same_v<T, LineToCmd>) {
                if (!hasSubpath) {
                    current.points.push_back(currentPoint);
                    hasSubpath = true;
                }
                current.points.push_back(c.p);
                currentPoint = c.p;
            } else if constexpr (std::is_same_v<T, QuadToCmd>) {
                if (!hasSubpath) {
                    current.points.push_back(currentPoint);
                    hasSubpath = true;
                }
                std::vector<FlattenedSeg> temp;
                flatten_quadratic(currentPoint, c.control, c.end, tolerance, temp);
                for (const auto& seg : temp) {
                    current.points.push_back(seg.second);
                }
                currentPoint = c.end;
            } else if constexpr (std::is_same_v<T, CubicToCmd>) {
                if (!hasSubpath) {
                    current.points.push_back(currentPoint);
                    hasSubpath = true;
                }
                std::vector<FlattenedSeg> temp;
                flatten_cubic(currentPoint, c.c0, c.c1, c.end, tolerance, temp);
                for (const auto& seg : temp) {
                    current.points.push_back(seg.second);
                }
                currentPoint = c.end;
            } else if constexpr (std::is_same_v<T, ArcToCmd>) {
                if (!hasSubpath) {
                    current.points.push_back(currentPoint);
                    hasSubpath = true;
                }
                std::vector<FlattenedSeg> temp;
                flatten_arc(c.arc, tolerance, temp);
                for (const auto& seg : temp) {
                    current.points.push_back(seg.second);
                }
                currentPoint = current.points.back();
            } else if constexpr (std::is_same_v<T, CloseCmd>) {
                if (hasSubpath && !current.points.empty()) {
                    current.points.push_back(subpathStart);
                    current.closed = true;
                    currentPoint = subpathStart;
                }
            }
        }, cmd);
    }
    if (!current.points.empty()) {
        subpaths.push_back(std::move(current));
    }

    // Remove duplicate consecutive points
    for (auto& sp : subpaths) {
        std::vector<math::Vec3f> cleaned;
        for (const auto& p : sp.points) {
            if (cleaned.empty()) {
                cleaned.push_back(p);
                continue;
            }
            auto& last = cleaned.back();
            float dx = p.x - last.x;
            float dy = p.y - last.y;
            float dz = p.z - last.z;
            if (dx * dx + dy * dy + dz * dz > 1e-12f) {
                cleaned.push_back(p);
            }
        }
        sp.points = std::move(cleaned);
    }

    // Remove subpaths with fewer than 2 points
    subpaths.erase(
        std::remove_if(subpaths.begin(), subpaths.end(),
            [](const Subpath& s) { return s.points.size() < 2; }),
        subpaths.end());

    return subpaths;
}

} // anonymous namespace

MeshData Path2D::tessellateStroke(const StrokeStyle& style, float tolerance) const
{
    if (impl_->commands.empty()) return {};

    auto subpaths = build_subpaths(impl_->commands, tolerance);
    if (subpaths.empty()) return {};

    MeshBuilder builder;
    float hw = style.width * 0.5f;

    for (const auto& sp : subpaths) {
        const auto& pts = sp.points;
        size_t n = pts.size();
        if (n < 2) continue;

        // ── Generate quads for each segment ──────────────────────────────
        for (size_t i = 0; i < n - 1; ++i) {
            const auto& a = pts[i];
            const auto& b = pts[i + 1];

            math::Vec3f dir = b - a;
            float len = dir.length();
            if (len < 1e-6f) continue;
            dir = dir / len;

            // Perpendicular in XY plane (rotate +90 deg around Z)
            math::Vec3f perp = {-dir.y, dir.x, 0.0f};

            Vertex v0, v1, v2, v3;
            v0.position = a - perp * hw;
            v1.position = a + perp * hw;
            v2.position = b + perp * hw;
            v3.position = b - perp * hw;

            v0.normal = v1.normal = v2.normal = v3.normal = {0, 0, 1};

            auto i0 = builder.add_vertex(v0);
            auto i1 = builder.add_vertex(v1);
            auto i2 = builder.add_vertex(v2);
            auto i3 = builder.add_vertex(v3);

            builder.add_triangle(i0, i1, i2);
            builder.add_triangle(i0, i2, i3);
        }

        // ── Caps for open subpaths ───────────────────────────────────────
        if (!sp.closed) {
            // Start cap
            {
                const auto& p = pts.front();
                const auto& q = pts[1];
                math::Vec3f dir = q - p;
                float len = dir.length();
                if (len > 1e-6f) {
                    dir = dir / len;
                    math::Vec3f perp = {-dir.y, dir.x, 0.0f};

                    if (style.cap == LineCap::Square) {
                        Vertex end0, end1, cap0, cap1;
                        end0.position = p - perp * hw;
                        end1.position = p + perp * hw;
                        cap0.position = p - perp * hw - dir * hw;
                        cap1.position = p + perp * hw - dir * hw;
                        end0.normal = end1.normal = cap0.normal = cap1.normal = {0, 0, 1};

                        auto e0 = builder.add_vertex(end0);
                        auto e1 = builder.add_vertex(end1);
                        auto c0 = builder.add_vertex(cap0);
                        auto c1 = builder.add_vertex(cap1);
                        builder.add_triangle(e0, c1, c0);
                        builder.add_triangle(e0, e1, c1);
                    }
                    // Butt cap: the first quad's edge already closes the end
                }
            }

            // End cap
            {
                const auto& p = pts.back();
                const auto& q = pts[n - 2];
                math::Vec3f dir = p - q;
                float len = dir.length();
                if (len > 1e-6f) {
                    dir = dir / len;
                    math::Vec3f perp = {-dir.y, dir.x, 0.0f};

                    if (style.cap == LineCap::Square) {
                        Vertex end0, end1, cap0, cap1;
                        end0.position = p - perp * hw;
                        end1.position = p + perp * hw;
                        cap0.position = p - perp * hw + dir * hw;
                        cap1.position = p + perp * hw + dir * hw;
                        end0.normal = end1.normal = cap0.normal = cap1.normal = {0, 0, 1};

                        auto e0 = builder.add_vertex(end0);
                        auto e1 = builder.add_vertex(end1);
                        auto c0 = builder.add_vertex(cap0);
                        auto c1 = builder.add_vertex(cap1);
                        builder.add_triangle(e0, c0, c1);
                        builder.add_triangle(e0, c1, e1);
                    }
                }
            }
        }
    }

    return builder.build();
}

} // namespace exd::geometry

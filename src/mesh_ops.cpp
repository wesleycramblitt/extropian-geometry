#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace exd::geometry
{

// ============================================================================
// Triangulation of planar polygons (ear clipping) with optional holes
// ============================================================================

namespace
{

// 2D point in the polygon plane.
struct P2
{
    float x = 0.0f;
    float y = 0.0f;
};

// Cross product (b - a) × (c - a) in 2D (z-component).
float cross2(const P2& a, const P2& b, const P2& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

float dist2(const P2& a, const P2& b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

// Twice the signed area of a closed ring (positive = CCW w.r.t. basis).
float signed_area2(const std::vector<P2>& ring)
{
    float s = 0.0f;
    const size_t n = ring.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++)
        s += ring[j].x * ring[i].y - ring[i].x * ring[j].y;
    return s;
}

// Orientation of (a,b,c): +1 CCW, -1 CW, 0 collinear (epsilon 1e-9).
int orient(const P2& a, const P2& b, const P2& c)
{
    constexpr float kEps = 1e-9f;
    const float v = cross2(a, b, c);
    if (v > kEps) return 1;
    if (v < -kEps) return -1;
    return 0;
}

// Strict (proper) crossing: the two segments intersect in their interiors.
// Endpoint touches and collinear overlap do NOT count as crossings.
bool segments_cross(const P2& a, const P2& b, const P2& c, const P2& d)
{
    const int o1 = orient(a, b, c);
    const int o2 = orient(a, b, d);
    const int o3 = orient(c, d, a);
    const int o4 = orient(c, d, b);
    return ((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) &&
           ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0));
}

// Ray-casting point-in-polygon (winding-agnostic).
bool point_in_polygon(const P2& p, const std::vector<P2>& ring)
{
    const size_t n = ring.size();
    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++)
    {
        const P2& a = ring[i];
        const P2& b = ring[j];
        if (((a.y > p.y) != (b.y > p.y)))
        {
            const float xint = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (xint > p.x)
                inside = !inside;
        }
    }
    return inside;
}

// Strict point-in-triangle (for a CCW triangle all three cross products
// must be positive; boundary points are excluded).
bool point_in_triangle_strict(const P2& p, const P2& a, const P2& b, const P2& c)
{
    constexpr float kEps = 1e-9f;
    const float s1 = cross2(a, b, p);
    const float s2 = cross2(b, c, p);
    const float s3 = cross2(c, a, p);
    return (s1 > kEps && s2 > kEps && s3 > kEps) ||
           (s1 < -kEps && s2 < -kEps && s3 < -kEps);
}

// Is segment h→o a valid bridge? It must not cross any outline or hole edge
// and its midpoint must lie in the outline region outside every hole.
bool bridge_ok(const P2& h, const P2& o,
               const std::vector<P2>& outline,
               const std::vector<std::vector<P2>>& holes,
               size_t thisHole)
{
    const size_t mo = outline.size();
    for (size_t i = 0; i < mo; ++i)
    {
        if (segments_cross(h, o, outline[i], outline[(i + 1) % mo]))
            return false;
    }
    for (size_t hh = 0; hh < holes.size(); ++hh)
    {
        const auto& hole = holes[hh];
        const size_t nh = hole.size();
        for (size_t i = 0; i < nh; ++i)
        {
            if (segments_cross(h, o, hole[i], hole[(i + 1) % nh]))
                return false;
        }
    }

    const P2 mid{(h.x + o.x) * 0.5f, (h.y + o.y) * 0.5f};
    if (!point_in_polygon(mid, outline))
        return false;
    for (size_t hh = 0; hh < holes.size(); ++hh)
    {
        if (point_in_polygon(mid, holes[hh]))
            return false;
    }
    return true;
}

// A vertex in the merged (figure-eight) loop being ear-clipped.
struct LoopVtx
{
    P2        p;
    uint32_t  idx;   // index into the combined [outline..., hole...] array
};

} // namespace

std::vector<uint32_t> triangulate_polygon(
    const std::vector<math::Vec3f>& outline,
    const std::vector<std::vector<math::Vec3f>>& holes)
{
    if (outline.size() < 3)
        return {};
    for (const auto& h : holes)
        if (h.size() < 3)
            return {};

    const size_t m = outline.size();

    // ── Plane basis via Newell's normal ──
    math::Vec3f n{0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < m; ++i)
    {
        const math::Vec3f& a = outline[i];
        const math::Vec3f& b = outline[(i + 1) % m];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    float nl = n.length();
    if (nl < 1e-6f)
    {
        // Degenerate Newell (collinear-ish): robust fallback over edge cross.
        n = {};
        for (size_t i = 1; i + 1 < m && nl < 1e-6f; ++i)
        {
            n = (outline[i] - outline[0]).cross(outline[i + 1] - outline[0]);
            nl = n.length();
        }
        if (nl < 1e-6f)
            return {};   // zero area / fully degenerate → empty
    }
    n = n / nl;

    math::Vec3f u{0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < m && u.length() < 1e-9f; ++i)
        u = outline[(i + 1) % m] - outline[i];
    if (u.length() < 1e-9f)
        return {};
    u = u.normalized();

    math::Vec3f v = n.cross(u);
    const float vl = v.length();
    if (vl < 1e-9f)
        return {};
    v = v / vl;

    // ── Project rings into the plane; keep the combined-array mapping ──
    auto project = [&](const std::vector<math::Vec3f>& ring)
    {
        std::vector<P2> pts;
        pts.reserve(ring.size());
        for (const auto& p : ring)
            pts.push_back(P2{p.dot(u), p.dot(v)});
        return pts;
    };

    std::vector<P2> outPts = project(outline);
    if (signed_area2(outPts) < 0.0f)
        std::reverse(outPts.begin(), outPts.end());   // normalize outline to CCW

    std::vector<std::vector<P2>> holePts;
    std::vector<uint32_t>        holeBase;
    holePts.reserve(holes.size());
    holeBase.reserve(holes.size());
    uint32_t hb = static_cast<uint32_t>(m);
    for (const auto& h : holes)
    {
        std::vector<P2> hp = project(h);
        if (signed_area2(hp) > 0.0f)
            std::reverse(hp.begin(), hp.end());       // normalize holes to CW
        holeBase.push_back(hb);
        hb += static_cast<uint32_t>(h.size());
        holePts.push_back(std::move(hp));
    }

    // ── Merge holes into the outline via visible bridges ──
    size_t totalHoleVerts = 0;
    for (const auto& hp : holePts)
        totalHoleVerts += hp.size();
    std::vector<LoopVtx> loop;
    loop.reserve(m + totalHoleVerts + 2 * holes.size());
    for (size_t i = 0; i < m; ++i)
        loop.push_back(LoopVtx{outPts[i], static_cast<uint32_t>(i)});

    for (size_t h = 0; h < holePts.size(); ++h)
    {
        size_t hi = 0;      // hole vertex index
        size_t bo = 0;      // outline vertex index
        float  best = std::numeric_limits<float>::max();
        for (size_t a = 0; a < holePts[h].size(); ++a)
        {
            for (size_t b = 0; b < m; ++b)
            {
                if (!bridge_ok(holePts[h][a], outPts[b], outPts, holePts, h))
                    continue;
                const float d = dist2(holePts[h][a], outPts[b]);
                if (d < best)
                {
                    best = d;
                    hi = a;
                    bo = b;
                }
            }
        }
        if (best == std::numeric_limits<float>::max())
            return {};   // no visible bridge

        // Figure-eight insertion: outline bridge vertex duplicated, the hole
        // ring (CW) inserted between two copies of its bridge vertex.
        loop.push_back(LoopVtx{outPts[bo], static_cast<uint32_t>(bo)});
        const size_t nh = holePts[h].size();
        for (size_t k = 0; k < nh; ++k)
        {
            const size_t j = (hi + k) % nh;
            loop.push_back(LoopVtx{holePts[h][j], holeBase[h] + static_cast<uint32_t>(j)});
        }
        loop.push_back(LoopVtx{holePts[h][hi], holeBase[h] + static_cast<uint32_t>(hi)});
    }

    // ── Ear clipping on the merged ring ──
    std::vector<uint32_t> result;
    if (loop.size() < 3)
        return {};

    constexpr float kEps    = 1e-9f;
    const size_t    maxIter = 4 * loop.size() + 64;
    for (size_t it = 0; loop.size() > 3 && it < maxIter; ++it)
    {
        bool clipped = false;
        const size_t L = loop.size();
        for (size_t i = 0; i < L && !clipped; ++i)
        {
            const LoopVtx& a = loop[(i + L - 1) % L];
            const LoopVtx& b = loop[i];
            const LoopVtx& c = loop[(i + 1) % L];

            // Guard: degenerate ears — zero-length edges / zero area.
            if (dist2(a.p, b.p) < kEps || dist2(b.p, c.p) < kEps || dist2(a.p, c.p) < kEps)
                continue;
            const float area = cross2(a.p, b.p, c.p);
            if (area <= kEps)   // also enforces convexity for a CCW ring
                continue;

            bool ear = true;

            // No merged-ring vertex strictly inside the candidate triangle.
            for (const auto& v : loop)
            {
                const bool isCorner = (&v == &a || &v == &b || &v == &c);
                if (!isCorner && point_in_triangle_strict(v.p, a.p, b.p, c.p))
                {
                    ear = false;
                    break;
                }
            }
            if (!ear)
                continue;

            // Ear edges must not cross any other loop edge.
            const P2 earEdges[3][2] = {{a.p, b.p}, {b.p, c.p}, {c.p, a.p}};
            for (size_t j = 0; j < L && ear; ++j)
            {
                const P2& u2 = loop[j].p;
                const P2& w2 = loop[(j + 1) % L].p;
                for (const auto& ee : earEdges)
                {
                    if (segments_cross(ee[0], ee[1], u2, w2))
                    {
                        ear = false;
                        break;
                    }
                }
            }
            if (!ear)
                continue;

            result.push_back(a.idx);
            result.push_back(b.idx);
            result.push_back(c.idx);
            loop.erase(loop.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
        }
        (void)clipped;
    }

    if (loop.size() == 3)
    {
        result.push_back(loop[0].idx);
        result.push_back(loop[1].idx);
        result.push_back(loop[2].idx);
    }

    return result;
}

// ============================================================================
// Vertex welding (hash grid, first-wins)
// ============================================================================

MeshData weld_vertices(const MeshData& mesh, float epsilon)
{
    if (epsilon <= 0.0f)
        return mesh;

    const size_t n = mesh.vertices.size();
    if (n == 0)
        return mesh;

    const float eps2 = epsilon * epsilon;

    struct CellKey
    {
        int64_t x{}, y{}, z{};
    };
    auto key_of = [&](const math::Vec3f& p)
    {
        return CellKey{static_cast<int64_t>(std::floor(p.x / epsilon)),
                       static_cast<int64_t>(std::floor(p.y / epsilon)),
                       static_cast<int64_t>(std::floor(p.z / epsilon))};
    };
    auto hash_key = [](const CellKey& k)
    {
        size_t h = 14695981039346656037ull;   // FNV-1a basis
        h = (h ^ static_cast<size_t>(k.x)) * 1099511628211ull;
        h = (h ^ static_cast<size_t>(k.y)) * 1099511628211ull;
        h = (h ^ static_cast<size_t>(k.z)) * 1099511628211ull;
        return h;
    };

    std::unordered_map<size_t, std::vector<uint32_t>> grid;

    std::vector<Vertex>   outVerts;
    std::vector<uint32_t> remap(n);
    outVerts.reserve(n);

    for (size_t i = 0; i < n; ++i)
    {
        const math::Vec3f& p = mesh.vertices[i].position;
        const CellKey      c = key_of(p);

        uint32_t keeper = 0;
        bool     found  = false;
        for (int dx = -1; dx <= 1 && !found; ++dx)
            for (int dy = -1; dy <= 1 && !found; ++dy)
                for (int dz = -1; dz <= 1 && !found; ++dz)
                {
                    const CellKey nk{c.x + dx, c.y + dy, c.z + dz};
                    const auto    it = grid.find(hash_key(nk));
                    if (it == grid.end())
                        continue;
                    for (const uint32_t k : it->second)
                    {
                        if ((outVerts[k].position - p).length_sq() <= eps2)
                        {
                            keeper = k;
                            found  = true;
                            break;
                        }
                    }
                }

        if (found)
        {
            remap[i] = keeper;
        }
        else
        {
            remap[i] = static_cast<uint32_t>(outVerts.size());
            outVerts.push_back(mesh.vertices[i]);
            grid[hash_key(c)].push_back(remap[i]);
        }
    }

    MeshData result;
    result.topology = mesh.topology;
    result.vertices = std::move(outVerts);
    result.indices.reserve(mesh.indices.size());
    for (const uint32_t idx : mesh.indices)
        result.indices.push_back(remap[idx]);
    result.bounds = compute_bounds(result.vertices);
    return result;
}

// ============================================================================
// Normal recomputation
// ============================================================================

MeshData recompute_normals(const MeshData& mesh, NormalMode mode)
{
    if (mesh.vertices.empty() || mesh.indices.empty())
        return {};
    const size_t triCount = mesh.indices.size() / 3;
    if (triCount == 0)
        return {};

    if (mode == NormalMode::Flat)
    {
        MeshData result;
        result.topology = PrimitiveTopology::Triangles;
        result.bounds   = mesh.bounds;
        result.vertices.reserve(triCount * 3);
        result.indices.reserve(triCount * 3);

        for (size_t f = 0; f < triCount; ++f)
        {
            const uint32_t ia = mesh.indices[f * 3 + 0];
            const uint32_t ib = mesh.indices[f * 3 + 1];
            const uint32_t ic = mesh.indices[f * 3 + 2];

            const math::Vec3f e1 = mesh.vertices[ib].position - mesh.vertices[ia].position;
            const math::Vec3f e2 = mesh.vertices[ic].position - mesh.vertices[ia].position;
            const math::Vec3f fn = e1.cross(e2);
            const float       fl = fn.length();
            const math::Vec3f nrm =
                fl > 1e-8f ? fn / fl : math::Vec3f{0.0f, 1.0f, 0.0f};

            for (uint32_t k = 0; k < 3; ++k)
            {
                Vertex v = mesh.vertices[mesh.indices[f * 3 + k]];
                v.normal = nrm;
                result.vertices.push_back(v);
            }
            const uint32_t base = static_cast<uint32_t>(f) * 3u;
            result.indices.push_back(base);
            result.indices.push_back(base + 1);
            result.indices.push_back(base + 2);
        }
        return result;
    }

    // ── Smooth: angle-weighted accumulation ──
    std::vector<math::Vec3f> acc(mesh.vertices.size(), math::Vec3f{});

    auto corner_angle = [](const math::Vec3f& u, const math::Vec3f& v) -> float
    {
        return std::atan2(u.cross(v).length(), u.dot(v));
    };

    for (size_t f = 0; f < triCount; ++f)
    {
        const uint32_t ia = mesh.indices[f * 3 + 0];
        const uint32_t ib = mesh.indices[f * 3 + 1];
        const uint32_t ic = mesh.indices[f * 3 + 2];

        const math::Vec3f& pa = mesh.vertices[ia].position;
        const math::Vec3f& pb = mesh.vertices[ib].position;
        const math::Vec3f& pc = mesh.vertices[ic].position;

        const math::Vec3f fn = (pb - pa).cross(pc - pa);
        const float       fl = fn.length();
        if (fl < 1e-8f)
            continue;   // degenerate face contributes nothing
        const math::Vec3f nf = fn / fl;

        const float ta = corner_angle(pb - pa, pc - pa);
        const float tb = corner_angle(pc - pb, pa - pb);
        const float tc = corner_angle(pa - pc, pb - pc);

        acc[ia] += nf * ta;
        acc[ib] += nf * tb;
        acc[ic] += nf * tc;
    }

    MeshData result;
    result.topology = mesh.topology;
    result.bounds   = mesh.bounds;
    result.indices  = mesh.indices;
    result.vertices.reserve(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); ++i)
    {
        Vertex   v = mesh.vertices[i];
        const float l = acc[i].length();
        v.normal = l > 1e-8f ? acc[i] / l : math::Vec3f{0.0f, 1.0f, 0.0f};
        result.vertices.push_back(v);
    }
    return result;
}

MeshData merge_meshes(std::span<const MeshData> meshes)
{
    if (meshes.empty())
    {
        return {};
    }

    // Validate topology consistency
    const auto topology = meshes[0].topology;
    for (size_t i = 1; i < meshes.size(); ++i)
    {
        if (meshes[i].topology != topology)
        {
            // Mismatched topology — fall back to first mesh's topology
            // but still merge the geometry data
        }
    }

    // Count totals
    size_t totalVertices = 0;
    size_t totalIndices  = 0;
    for (const auto& m : meshes)
    {
        totalVertices += m.vertices.size();
        totalIndices  += m.indices.size();
    }

    MeshData result;
    result.topology = topology;
    result.vertices.reserve(totalVertices);
    result.indices.reserve(totalIndices);

    uint32_t baseVertex = 0;
    for (const auto& m : meshes)
    {
        // Append vertices
        result.vertices.insert(result.vertices.end(),
                               m.vertices.begin(), m.vertices.end());

        // Append indices with offset
        for (auto idx : m.indices)
        {
            result.indices.push_back(baseVertex + idx);
        }

        baseVertex += static_cast<uint32_t>(m.vertices.size());
    }

    result.bounds = compute_bounds(result.vertices);
    return result;
}

// Helper: transform a Vec3f point by Mat4 (column-major, w=1 for point)
static math::Vec3f transform_point(const math::Vec3f& v, const math::Mat4& m)
{
    return {
        m.m[0] * v.x + m.m[4] * v.y + m.m[8]  * v.z + m.m[12],
        m.m[1] * v.x + m.m[5] * v.y + m.m[9]  * v.z + m.m[13],
        m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z + m.m[14]
    };
}

// Helper: transform a Vec3f direction by the upper 3x3 (inverse-transpose
// for normals; for rigid transforms this is just the rotation part)
static math::Vec3f transform_direction(const math::Vec3f& v, const math::Mat4& m)
{
    return {
        m.m[0] * v.x + m.m[1] * v.y + m.m[2]  * v.z,
        m.m[4] * v.x + m.m[5] * v.y + m.m[6]  * v.z,
        m.m[8] * v.x + m.m[9] * v.y + m.m[10] * v.z
    };
}

MeshData transform_mesh(const MeshData& mesh,
                        const math::Mat4& transform,
                        bool transformNormals)
{
    MeshData result;
    result.topology = mesh.topology;
    result.vertices.reserve(mesh.vertices.size());
    result.indices = mesh.indices; // indices unchanged

    for (const auto& v : mesh.vertices)
    {
        Vertex tv = v;
        tv.position = transform_point(v.position, transform);
        if (transformNormals)
        {
            tv.normal = transform_direction(v.normal, transform);
        }
        result.vertices.push_back(tv);
    }

    result.bounds = compute_bounds(result.vertices);
    return result;
}

Bounds compute_bounds(std::span<const Vertex> vertices)
{
    if (vertices.empty())
    {
        return {};
    }

    Bounds b;
    b.min = vertices[0].position;
    b.max = vertices[0].position;

    for (const auto& v : vertices)
    {
        b.min.x = std::min(b.min.x, v.position.x);
        b.min.y = std::min(b.min.y, v.position.y);
        b.min.z = std::min(b.min.z, v.position.z);
        b.max.x = std::max(b.max.x, v.position.x);
        b.max.y = std::max(b.max.y, v.position.y);
        b.max.z = std::max(b.max.z, v.position.z);
    }

    return b;
}

} // namespace exd::geometry

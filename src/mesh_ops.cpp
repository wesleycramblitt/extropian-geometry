#include <exd/geometry/mesh_ops.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
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

// ============================================================================
// Boolean (CSG)
// ============================================================================

namespace
{

// ── Position canonicalization ──────────────────────────────────────────────

struct CanonicalMap
{
    std::vector<math::Vec3f> positions;   // canonical position per canonical vertex
    std::vector<uint32_t>    remap;       // original vertex index → canonical index
};

// Maps every position to the FIRST canonical position within posEps (O(V²),
// V is small by design). Canonical positions preserve the first-wins attribute.
CanonicalMap canonicalize_positions(std::span<const math::Vec3f> pos, float posEps)
{
    CanonicalMap out;
    out.remap.resize(pos.size());
    const float eps2 = posEps <= 0.0f ? 0.0f : posEps * posEps;
    for (size_t i = 0; i < pos.size(); ++i)
    {
        uint32_t keeper = UINT32_MAX;
        for (size_t j = 0; j < out.positions.size(); ++j)
        {
            const math::Vec3f d = pos[i] - out.positions[j];
            if (d.length_sq() <= eps2)
            {
                keeper = static_cast<uint32_t>(j);
                break;
            }
        }
        if (keeper == UINT32_MAX)
        {
            keeper = static_cast<uint32_t>(out.positions.size());
            out.positions.push_back(pos[i]);
        }
        out.remap[i] = keeper;
    }
    return out;
}

std::vector<math::Vec3f> extract_positions(const MeshData& m)
{
    std::vector<math::Vec3f> out;
    out.reserve(m.vertices.size());
    for (const Vertex& v : m.vertices)
        out.push_back(v.position);
    return out;
}

Bounds positions_bounds(std::span<const math::Vec3f> pos)
{
    if (pos.empty())
        return {};
    Bounds b;
    b.min = pos[0];
    b.max = pos[0];
    for (const auto& p : pos)
    {
        b.min.x = std::min(b.min.x, p.x);
        b.min.y = std::min(b.min.y, p.y);
        b.min.z = std::min(b.min.z, p.z);
        b.max.x = std::max(b.max.x, p.x);
        b.max.y = std::max(b.max.y, p.y);
        b.max.z = std::max(b.max.z, p.z);
    }
    return b;
}

// ── Helper 1: closed-manifold gate ─────────────────────────────────────────
// Position-canonicalized (so un-welded generator output passes), undirected
// edge count == 2 and directed opposition (directed sign sum == 0). Non-
// manifold VERTICES are not diagnosed (two cubes touching at a point pass —
// accepted V1 limitation).

bool closed_manifold_gate(const MeshData& m, float posEps)
{
    const size_t nv = m.vertices.size();
    const size_t ni = m.indices.size();
    if (nv == 0 || ni == 0 || ni % 3 != 0)
        return false;
    if (!(posEps > 0.0f))
        return false;

    const std::vector<math::Vec3f> positions = extract_positions(m);
    const CanonicalMap cm = canonicalize_positions(positions, posEps);

    struct EdgeKey
    {
        uint32_t a, b;   // a <= b
        bool operator==(const EdgeKey& o) const { return a == o.a && b == o.b; }
    };
    struct EdgeHash
    {
        size_t operator()(const EdgeKey& k) const
        {
            return (static_cast<size_t>(k.a) << 32u) ^ static_cast<size_t>(k.b);
        }
    };
    struct EdgeStat
    {
        int count   = 0;   // total directed occurrences
        int balance = 0;   // +1 per (a<b) occurrence, −1 per (a>b)
    };

    std::unordered_map<EdgeKey, EdgeStat, EdgeHash> edges;
    edges.reserve(ni);

    const size_t triCount = ni / 3;
    for (size_t t = 0; t < triCount; ++t)
    {
        const uint32_t c0 = cm.remap[m.indices[t * 3 + 0]];
        const uint32_t c1 = cm.remap[m.indices[t * 3 + 1]];
        const uint32_t c2 = cm.remap[m.indices[t * 3 + 2]];
        auto add_dir = [&](uint32_t a, uint32_t b)
        {
            const uint32_t lo = std::min(a, b);
            const uint32_t hi = std::max(a, b);
            EdgeStat& e = edges[{lo, hi}];
            e.count += 1;
            e.balance += (a < b) ? 1 : (a > b ? -1 : 0);
        };
        add_dir(c0, c1);
        add_dir(c1, c2);
        add_dir(c2, c0);
    }

    for (const auto& kv : edges)
    {
        if (kv.second.count != 2 || kv.second.balance != 0)
            return false;
    }
    return true;
}

// ── Helper 2: signed volume + winding normalization ────────────────────────

// Divergence theorem over canonicalized triangles; accumulated in double.
double compute_signed_volume(const MeshData& m)
{
    const size_t ni = m.indices.size();
    if (m.vertices.empty() || ni == 0 || ni % 3 != 0)
        return 0.0;
    const std::vector<math::Vec3f> positions = extract_positions(m);
    const Bounds pb = positions_bounds(positions);
    const float diag = (pb.max - pb.min).length();
    const float posEps = diag * 1e-7f;
    const CanonicalMap cm = canonicalize_positions(positions, posEps);
    const std::vector<math::Vec3f>& cp = cm.positions;

    double vol = 0.0;
    for (size_t t = 0; t < ni; t += 3)
    {
        const math::Vec3f& a = cp[cm.remap[m.indices[t + 0]]];
        const math::Vec3f& b = cp[cm.remap[m.indices[t + 1]]];
        const math::Vec3f& c = cp[cm.remap[m.indices[t + 2]]];
        vol += a.dot(b.cross(c)) / 6.0;
    }
    return vol;
}

// Returns a winding-normalized copy (outward) or empty with ok=false when the
// mesh is a zero-volume shell (|volume| < 1e-9·diag³).
MeshData normalize_to_outward(const MeshData& m, bool& ok)
{
    ok = false;
    const double vol = compute_signed_volume(m);
    const Bounds b = compute_bounds(m.vertices);
    const double diag = (b.max - b.min).length();
    const double volMin = 1e-9 * diag * diag * diag;
    if (std::abs(vol) < volMin)
        return {};
    ok = true;
    if (vol < 0.0)
    {
        MeshData r = m;
        const size_t ni = r.indices.size();
        for (size_t t = 0; t < ni; t += 3)
            std::swap(r.indices[t + 1], r.indices[t + 2]);
        return r;
    }
    return m;
}

// ── Plane / projection helpers ─────────────────────────────────────────────

inline P2 project_to_2d(const math::Vec3f& p, const math::Vec3f& origin,
                        const math::Vec3f& u, const math::Vec3f& v)
{
    const math::Vec3f d = p - origin;
    return P2{d.dot(u), d.dot(v)};
}

inline math::Vec3f unproject_from_2d(const P2& p, const math::Vec3f& origin,
                                     const math::Vec3f& u, const math::Vec3f& v)
{
    return origin + u * p.x + v * p.y;
}

// Intersection segment of triangle (a,b,c) with the plane (n, origin).
// `tol` marks on-plane vertices (signed-distance tolerance). Returns false when
// there is no proper crossing (tangent/edge-only contact included).
bool plane_triangle_segment(const math::Vec3f& n, const math::Vec3f& origin,
                            const math::Vec3f& a, const math::Vec3f& b,
                            const math::Vec3f& c, float tol,
                            math::Vec3f& s0, math::Vec3f& s1)
{
    const float da = n.dot(a - origin);
    const float db = n.dot(b - origin);
    const float dc = n.dot(c - origin);

    // Fully one side → no crossing (coplanar/all-in-tol handled by caller).
    if ((da > tol && db > tol && dc > tol) ||
        (da < -tol && db < -tol && dc < -tol))
        return false;

    math::Vec3f pts[2];
    int np = 0;
    const math::Vec3f P[3] = {a, b, c};
    const float     D[3] = {da, db, dc};

    auto push = [&](const math::Vec3f& p)
    {
        if (np >= 2)
            return;
        for (int k = 0; k < np; ++k)
            if ((pts[k] - p).length_sq() <= tol * tol)
                return;
        pts[np++] = p;
    };

    if (std::abs(da) <= tol) push(a);
    if (std::abs(db) <= tol) push(b);
    if (std::abs(dc) <= tol) push(c);

    for (int i = 0; i < 3; ++i)
    {
        const int j = (i + 1) % 3;
        if ((D[i] > tol && D[j] < -tol) || (D[i] < -tol && D[j] > tol))
        {
            const float t = D[i] / (D[i] - D[j]);
            push(P[i] + (P[j] - P[i]) * t);
        }
    }
    if (np < 2)
        return false;
    s0 = pts[0];
    s1 = pts[1];
    return true;
}

// Clip a 2D segment to a CCW convex polygon. `eps` is the boundary tolerance
// (points within eps of an edge count as inside).
bool clip_segment_to_polygon(const P2& a, const P2& b,
                             const std::vector<P2>& ring,
                             float eps, P2& o0, P2& o1)
{
    float l0 = 0.0f, l1 = 1.0f;
    const size_t n = ring.size();
    for (size_t i = 0; i < n; ++i)
    {
        const P2& e0 = ring[i];
        const P2& e1 = ring[(i + 1) % n];
        const float s0 = cross2(e0, e1, a);
        const float s1 = cross2(e0, e1, b);
        const float d = s1 - s0;
        if (s0 >= -eps && s1 >= -eps)
            continue;
        if (std::abs(d) < 1e-12f)
            return false;   // both outside, parallel to the edge
        const float t = (-eps - s0) / d;
        if (s0 < -eps)
            l0 = std::max(l0, t);
        if (s1 < -eps)
            l1 = std::min(l1, t);
        if (l0 > l1)
            return false;
    }
    const P2 q0{a.x + l0 * (b.x - a.x), a.y + l0 * (b.y - a.y)};
    const P2 q1{a.x + l1 * (b.x - a.x), a.y + l1 * (b.y - a.y)};
    if (dist2(q0, q1) < eps * eps)
        return false;
    o0 = q0;
    o1 = q1;
    return true;
}

// Convex hull (Andrew monotone chain) of uniquely-deduplicated 2D points,
// returned CCW without repeating the first point.
std::vector<P2> convex_hull(std::vector<P2> pts)
{
    constexpr float kEps = 1e-9f;
    std::sort(pts.begin(), pts.end(), [](const P2& a, const P2& b)
    {
        return a.x != b.x ? a.x < b.x : a.y < b.y;
    });
    std::vector<P2> uniq;
    for (const P2& p : pts)
    {
        if (uniq.empty() || dist2(uniq.back(), p) > kEps)
            uniq.push_back(p);
    }
    if (uniq.size() <= 3)
        return uniq;
    std::vector<P2> hull;
    hull.reserve(2 * uniq.size());
    for (const P2& p : uniq)
    {
        while (hull.size() >= 2 && cross2(hull[hull.size() - 2], hull.back(), p) <= 0.0f)
            hull.pop_back();
        hull.push_back(p);
    }
    const size_t lower = hull.size() + 1;
    for (size_t i = uniq.size(); i-- > 0;)
    {
        const P2& p = uniq[i];
        while (hull.size() >= lower && cross2(hull[hull.size() - 2], hull.back(), p) <= 0.0f)
            hull.pop_back();
        hull.push_back(p);
    }
    hull.pop_back();   // last point == first point
    return hull;
}

// Coplanar 2D interior-overlap test for two triangles projected onto a shared
// plane. True only when interiors overlap (strict — touching at edges/points
// does not count).
bool triangles_overlap_2d(const P2& a0, const P2& a1, const P2& a2,
                          const P2& b0, const P2& b1, const P2& b2)
{
    const P2* A[3] = {&a0, &a1, &a2};
    const P2* B[3] = {&b0, &b1, &b2};
    for (const P2* q : B)
        if (point_in_triangle_strict(*q, a0, a1, a2))
            return true;
    for (const P2* q : A)
        if (point_in_triangle_strict(*q, b0, b1, b2))
            return true;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (segments_cross(*A[i], *A[(i + 1) % 3], *B[j], *B[(j + 1) % 3]))
                return true;
    return false;
}

// ── Triangle arrangement (splitting) ───────────────────────────────────────

struct Seg2
{
    P2 a;
    P2 b;
};

struct ArrangedPiece
{
    std::vector<P2>        ring2D;   // polygon ring in the triangle's 2D frame (CCW)
    std::vector<math::Vec3f> ring3D; // same ring lifted to 3D (same order)
};

float point_seg_dist2(const P2& p, const P2& a, const P2& b)
{
    const float abx = b.x - a.x, aby = b.y - a.y;
    const float len2 = abx * abx + aby * aby;
    if (len2 < 1e-18f)
        return dist2(p, a);
    const float t = std::clamp(((p.x - a.x) * abx + (p.y - a.y) * aby) / len2, 0.0f, 1.0f);
    const P2 q{a.x + t * abx, a.y + t * aby};
    return dist2(p, q);
}

// Span-aware segment intersection: only true when the crossing lies within BOTH
// segment spans (within tol). Out-of-span line crossings must not be inserted.
bool segments_intersect(const P2& a, const P2& b, const P2& c, const P2& d,
                        P2& out, float tol)
{
    const float r_x = b.x - a.x, r_y = b.y - a.y;
    const float s_x = d.x - c.x, s_y = d.y - c.y;
    const float denom = r_x * s_y - r_y * s_x;
    if (std::abs(denom) < 1e-12f)
        return false;   // parallel
    const float q_x = c.x - a.x, q_y = c.y - a.y;
    const float t = (q_x * s_y - q_y * s_x) / denom;
    const float u = (q_x * r_y - q_y * r_x) / denom;
    const float e = tol / std::max(std::sqrt(r_x * r_x + r_y * r_y),
                                   std::sqrt(s_x * s_x + s_y * s_y));
    if (t < -e || t > 1.0f + e || u < -e || u > 1.0f + e)
        return false;
    out = {a.x + t * r_x, a.y + t * r_y};
    return true;
}

bool insert_point(std::vector<P2>& pts, const P2& p, float snapTol)
{
    const float eps2 = snapTol * snapTol;
    for (const P2& q : pts)
        if (dist2(q, p) <= eps2)
            return false;
    pts.push_back(p);
    return true;
}

// Sorts the per-segment split points along the segment and dedupes.
void sort_segment_points(const P2& a, const P2& b, std::vector<P2>& pts, float snapTol)
{
    const float dx = b.x - a.x, dy = b.y - a.y;
    const bool horizontal = std::abs(dx) >= std::abs(dy);
    std::sort(pts.begin(), pts.end(), [&](const P2& p, const P2& q)
    {
        return horizontal ? (p.x < q.x) : (p.y < q.y);
    });
    std::vector<P2> out;
    const float eps2 = snapTol * snapTol;
    for (const P2& p : pts)
    {
        if (out.empty() || dist2(out.back(), p) > eps2)
            out.push_back(p);
    }
    pts = std::move(out);
}

bool point_strictly_inside_convex(const std::vector<P2>& ring, const P2& p, float areaEps)
{
    const size_t n = ring.size();
    if (n < 3)
        return false;
    for (size_t i = 0; i < n; ++i)
    {
        const P2& a = ring[i];
        const P2& b = ring[(i + 1) % n];
        if (cross2(a, b, p) <= areaEps)
            return false;
    }
    return true;
}

// Full chord of a convex ring cut by the infinite LINE through `seg`. Returns
// false when the line does not properly cross the ring (tangent / parallel).
// Two boundary intersection points are collected; they are guaranteed to lie on
// the ring boundary, so the subsequent chord split always has valid endpoints.
bool ring_line_chord(const std::vector<P2>& ring, const Seg2& seg, float snapTol,
                     P2& c0, P2& c1)
{
    const float dx = seg.b.x - seg.a.x, dy = seg.b.y - seg.a.y;
    const float len2 = dx * dx + dy * dy;
    if (len2 < 1e-18f)
        return false;
    const P2 s0 = seg.a;
    const float tol2 = snapTol * snapTol;

    std::vector<P2> pts;
    const size_t n = ring.size();
    for (size_t i = 0; i < n; ++i)
    {
        const P2& ea = ring[i];
        const P2& eb = ring[(i + 1) % n];
        const float ex = eb.x - ea.x, ey = eb.y - ea.y;
        const float denom = dx * ey - dy * ex;   // (d × e)
        if (std::abs(denom) < 1e-12f)
            continue;   // edge parallel to the line (or collinear) — no proper crossing
        const float rax = ea.x - s0.x, ray = ea.y - s0.y;
        const float u = (rax * dy - ray * dx) / denom;   // (ra × d) / (d × e)
        if (u < -snapTol || u > 1.0f + snapTol)
            continue;   // crossing outside the edge span
        const P2 p{ea.x + u * ex, ea.y + u * ey};
        bool dup = false;
        for (const P2& q : pts)
        {
            if ((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) <= tol2)
            {
                dup = true;
                break;
            }
        }
        if (!dup)
            pts.push_back(p);
        if (pts.size() >= 2)
            break;   // a line crosses a convex ring boundary at most twice
    }
    if (pts.size() < 2)
        return false;
    c0 = pts[0];
    c1 = pts[1];
    return true;
}

// Split a CCW convex ring along a chord with endpoints e1/e2 lying on its
// boundary. Endpoints within snapTol of a ring vertex snap to it; otherwise the
// endpoint is inserted onto the edge containing it. Returns {piece1, piece2};
// a degenerate/un-splittable chord returns an empty first piece.
std::pair<std::vector<P2>, std::vector<P2>> split_convex_ring(
    const std::vector<P2>& ring, const P2& e1, const P2& e2, float snapTol)
{
    std::vector<P2> r = ring;
    const float eps2 = snapTol * snapTol;

    auto resolve = [&](const P2& e) -> size_t
    {
        for (size_t i = 0; i < r.size(); ++i)
            if (dist2(r[i], e) <= eps2)
                return i;
        const size_t n = r.size();
        size_t       bestA = n;
        float        bestD = std::numeric_limits<float>::max();
        for (size_t i = 0; i < n; ++i)
        {
            const float d = point_seg_dist2(e, r[i], r[(i + 1) % n]);
            if (d < bestD)
            {
                bestD = d;
                bestA = i;
            }
        }
        if (bestA == n || bestD > eps2)
            return n;   // not on the boundary
        const P2& a = r[bestA];
        const P2& b = r[(bestA + 1) % n];
        const float abx = b.x - a.x, aby = b.y - a.y;
        const float len2 = abx * abx + aby * aby;
        float t = 0.0f;
        if (len2 > 1e-18f)
            t = std::clamp(((e.x - a.x) * abx + (e.y - a.y) * aby) / len2, 0.0f, 1.0f);
        r.insert(r.begin() + static_cast<std::ptrdiff_t>(bestA) + 1, P2{a.x + t * abx, a.y + t * aby});
        return bestA + 1;
    };

    const size_t i1a = resolve(e1);
    if (i1a == r.size())
        return {};
    const size_t i2a = resolve(e2);
    if (i2a == r.size())
        return {};

    // Re-resolve after BOTH insertions: the second insertion may have shifted
    // the index returned for the first endpoint.
    const size_t i1 = resolve(e1);
    const size_t i2 = resolve(e2);
    if (i1 == r.size() || i2 == r.size())
        return {};
    if (i1 == i2)
        return {};
    const size_t n = r.size();
    if ((i1 + 1) % n == i2 || (i2 + 1) % n == i1)
        return {};   // chord coincides with an existing edge → no split

    std::vector<P2> p1, p2;
    size_t k = i1;
    while (true)
    {
        p1.push_back(r[k]);
        if (k == i2)
            break;
        k = (k + 1) % n;
    }
    k = i2;
    while (true)
    {
        p2.push_back(r[k]);
        if (k == i1)
            break;
        k = (k + 1) % n;
    }
    return {std::move(p1), std::move(p2)};
}

// ── Collinear sub-segment merging ──────────────────────────────────────────
// Adjacent triangles of the SAME B surface met along a shared diagonal produce
// collinear fragments that touch at a point in the INTERIOR of Ta (not on its
// boundary). Chord splitting requires both endpoints on the ring boundary, so
// such fragments are merged into one full chord spanning their union.

bool segments_collinear(const Seg2& p, const Seg2& q, float snapTol)
{
    const float dx1 = q.b.x - q.a.x, dy1 = q.b.y - q.a.y;
    const float len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    if (len1 < 1e-12f)
        return false;
    // Cross of p's direction with q's direction (normalized) ~ 0.
    const float px = p.b.x - p.a.x, py = p.b.y - p.a.y;
    const float pl = std::sqrt(px * px + py * py);
    if (pl < 1e-12f)
        return false;
    const float c = (px / pl) * (dy1 / len1) - (py / pl) * (dx1 / len1);
    const float angTol = snapTol / std::max(pl, len1);
    return std::abs(c) <= std::max(angTol, 1e-9f);
}

bool segs_connected(const Seg2& p, const Seg2& q, float snapTol)
{
    const float e2 = snapTol * snapTol;
    if (dist2(p.a, q.a) <= e2 || dist2(p.a, q.b) <= e2 ||
        dist2(p.b, q.a) <= e2 || dist2(p.b, q.b) <= e2)
        return true;
    if (point_seg_dist2(p.a, q.a, q.b) <= e2 || point_seg_dist2(p.b, q.a, q.b) <= e2)
        return true;
    if (point_seg_dist2(q.a, p.a, p.b) <= e2 || point_seg_dist2(q.b, p.a, p.b) <= e2)
        return true;
    return false;
}

Seg2 merge_seg_union(const Seg2& p, const Seg2& q)
{
    const P2 pts[4] = {p.a, p.b, q.a, q.b};
    const float dx = (p.b.x - p.a.x) + (q.b.x - q.a.x);
    const float dy = (p.b.y - p.a.y) + (q.b.y - q.a.y);
    const bool ax = std::abs(dx) >= std::abs(dy);
    int lo = 0, hi = 0;
    for (int i = 1; i < 4; ++i)
    {
        const float v_i = ax ? pts[i].x : pts[i].y;
        const float v_lo = ax ? pts[lo].x : pts[lo].y;
        const float v_hi = ax ? pts[hi].x : pts[hi].y;
        if (v_i < v_lo) lo = i;
        if (v_i > v_hi) hi = i;
    }
    return Seg2{pts[lo], pts[hi]};
}

// Merge collinear connected sub-segments until a fixed point.
void merge_collinear_segments(std::vector<Seg2>& subs, float snapTol)
{
    const size_t maxPasses = subs.size() + 8;
    for (size_t pass = 0; pass < maxPasses; ++pass)
    {
        std::vector<Seg2> out;
        bool mergedAny = false;
        for (const Seg2& s : subs)
        {
            bool absorbed = false;
            for (Seg2& m : out)
            {
                if (segments_collinear(m, s, snapTol) && segs_connected(m, s, snapTol))
                {
                    m = merge_seg_union(m, s);
                    absorbed = true;
                    mergedAny = true;
                    break;
                }
            }
            if (!absorbed)
                out.push_back(s);
        }
        subs = std::move(out);
        if (!mergedAny)
            break;
    }
}

// Core arrangement subsystem (isolated for later replacement by a
// unified-curve arrangement).
//
// CONTRACT: `ta2D` is the CCW 2D projection of a triangle in the (u,v) basis
// (plane origin + u·x + v·y is the lift to 3D); `segments2D` are cut segments
// already clipped to the triangle interior (each is one B-surface crossing of
// the triangle's plane). The function splits the triangle along those segments
// into a set of convex polygons (progressively "cut by chord"), dropping
// slivers below `minArea`. It returns the pieces as polygon rings plus their 3D
// lifts.
std::vector<ArrangedPiece> arrange_triangle(
    const std::vector<P2>& ta2D,
    const std::vector<Seg2>& segments2D,
    const math::Vec3f& planeOrigin,
    const math::Vec3f& uAxis,
    const math::Vec3f& vAxis,
    float snapTol,
    float minArea)
{
    // 1. Crossing/T-junction splitting of the input segments.
    struct SegPts
    {
        P2                a, b;
        std::vector<P2>   pts;
    };
    std::vector<SegPts> seps;
    seps.reserve(segments2D.size());
    for (const Seg2& s : segments2D)
        seps.push_back(SegPts{s.a, s.b, {s.a, s.b}});

    for (size_t i = 0; i < seps.size(); ++i)
    {
        for (size_t j = i + 1; j < seps.size(); ++j)
        {
            P2 xi;
            if (segments_intersect(seps[i].a, seps[i].b, seps[j].a, seps[j].b, xi, snapTol))
            {
                insert_point(seps[i].pts, xi, snapTol);
                insert_point(seps[j].pts, xi, snapTol);
            }
            else
            {
                // T-junctions: endpoint of one on the interior of the other.
                for (const P2& ep : {seps[i].a, seps[i].b})
                    if (point_seg_dist2(ep, seps[j].a, seps[j].b) <= snapTol * snapTol)
                        insert_point(seps[j].pts, ep, snapTol);
                for (const P2& ep : {seps[j].a, seps[j].b})
                    if (point_seg_dist2(ep, seps[i].a, seps[i].b) <= snapTol * snapTol)
                        insert_point(seps[i].pts, ep, snapTol);
            }
        }
    }

    std::vector<Seg2> subs;
    for (SegPts& sp : seps)
    {
        sort_segment_points(sp.a, sp.b, sp.pts, snapTol);
        for (size_t k = 0; k + 1 < sp.pts.size(); ++k)
        {
            if (dist2(sp.pts[k], sp.pts[k + 1]) >= snapTol * snapTol)
                subs.push_back(Seg2{sp.pts[k], sp.pts[k + 1]});
        }
    }

    // Merge collinear fragments (adjacent B-triangle crossings of one surface
    // line) into full chords spanning the triangle.
    merge_collinear_segments(subs, snapTol);

    // 2. Progressive chord splitting.
    std::vector<std::vector<P2>> rings;
    rings.push_back(ta2D);
    const float areaEps = std::max(minArea, 1e-12f);
    const float twoMin  = 2.0f * minArea;

    for (const Seg2& seg : subs)
    {
        const P2 mid{(seg.a.x + seg.b.x) * 0.5f, (seg.a.y + seg.b.y) * 0.5f};
        std::vector<std::vector<P2>> next;
        next.reserve(rings.size() + 1);
        for (const std::vector<P2>& ring : rings)
        {
            if (!point_strictly_inside_convex(ring, mid, areaEps))
            {
                next.push_back(ring);
                continue;
            }
            // Cut the ring along the FULL chord of the infinite line through
            // `seg` (endpoints guaranteed on the ring boundary). Splitting along
            // the sub-segment alone would leave interior endpoints (crossing
            // points / collinear fragments) that are not on any boundary.
            P2 ca, cb;
            if (!ring_line_chord(ring, seg, snapTol, ca, cb))
            {
                next.push_back(ring);
                continue;
            }
            const auto pieces = split_convex_ring(ring, ca, cb, snapTol);
            if (pieces.first.empty())
            {
                next.push_back(ring);
                continue;
            }
            const float a1 = std::abs(signed_area2(pieces.first));
            const float a2 = std::abs(signed_area2(pieces.second));
            bool added = false;
            if (a1 >= twoMin)
            {
                next.push_back(std::move(pieces.first));
                added = true;
            }
            if (a2 >= twoMin)
            {
                next.push_back(std::move(pieces.second));
                added = true;
            }
            if (!added)
                next.push_back(ring);   // both slivers dropped → keep original
        }
        rings = std::move(next);
    }

    // 3. Lift to 3D.
    std::vector<ArrangedPiece> out;
    out.reserve(rings.size());
    for (const std::vector<P2>& ring : rings)
    {
        ArrangedPiece piece;
        piece.ring2D = ring;
        piece.ring3D.reserve(ring.size());
        for (const P2& p : ring)
            piece.ring3D.push_back(unproject_from_2d(p, planeOrigin, uAxis, vAxis));
        out.push_back(std::move(piece));
    }
    return out;
}

// ── Point-in-mesh classification (parity ray) ──────────────────────────────

bool point_on_triangle(const math::Vec3f& p,
                       const math::Vec3f& a, const math::Vec3f& b, const math::Vec3f& c,
                       float eps)
{
    const math::Vec3f e1 = b - a;
    const math::Vec3f e2 = c - a;
    const math::Vec3f n = e1.cross(e2);
    const float nl = n.length();
    if (nl < 1e-12f)
        return false;
    const math::Vec3f rel = p - a;
    if (std::abs(n.dot(rel)) > eps * nl)
        return false;   // not on the triangle plane (within eps)
    const float ax = std::abs(n.x), ay = std::abs(n.y), az = std::abs(n.z);
    P2 a2, b2, c2, p2;
    if (az >= ax && az >= ay)
    {
        a2 = {a.x, a.y}; b2 = {b.x, b.y}; c2 = {c.x, c.y}; p2 = {p.x, p.y};
    }
    else if (ay >= ax)
    {
        a2 = {a.x, a.z}; b2 = {b.x, b.z}; c2 = {c.x, c.z}; p2 = {p.x, p.z};
    }
    else
    {
        a2 = {a.y, a.z}; b2 = {b.y, b.z}; c2 = {c.y, c.z}; p2 = {p.y, p.z};
    }
    const float area2 = cross2(a2, b2, c2);
    if (std::abs(area2) < 1e-12f)
        return false;
    const float sign = area2 > 0.0f ? 1.0f : -1.0f;
    const float thr = -eps * std::sqrt(std::abs(area2));
    return cross2(a2, b2, p2) * sign >= thr &&
           cross2(b2, c2, p2) * sign >= thr &&
           cross2(c2, a2, p2) * sign >= thr;
}

bool ray_hits_triangle(const math::Vec3f& origin, const math::Vec3f& dir,
                       const math::Vec3f& a, const math::Vec3f& b, const math::Vec3f& c,
                       float baryEps)
{
    const math::Vec3f e1 = b - a;
    const math::Vec3f e2 = c - a;
    const math::Vec3f pv = dir.cross(e2);
    const float det = e1.dot(pv);
    if (std::abs(det) < 1e-12f)
        return false;
    const float inv = 1.0f / det;
    const math::Vec3f sv = origin - a;
    const float u2 = sv.dot(pv) * inv;
    if (u2 < -baryEps || u2 > 1.0f + baryEps)
        return false;
    const math::Vec3f qv = sv.cross(e1);
    const float v2 = dir.dot(qv) * inv;
    if (v2 < -baryEps || u2 + v2 > 1.0f + baryEps)
        return false;
    const float t = e2.dot(qv) * inv;
    if (!std::isfinite(t))
        return false;
    return t >= 1e-9f;   // forward only
}

// Deterministic jitter of the ray origin perpendicular to the ray, so the
// parity ray does not graze vertices/edges.
math::Vec3f jitter_ray_origin(const math::Vec3f& p, const math::Vec3f& dir, float scale)
{
    uint64_t h = 14695981039346656037ull;
    auto mix = [&h](uint32_t bits)
    {
        h = (h ^ static_cast<uint64_t>(bits)) * 1099511628211ull;
    };
    mix(std::bit_cast<uint32_t>(std::round(p.x * 1e4f)));
    mix(std::bit_cast<uint32_t>(std::round(p.y * 1e4f)));
    mix(std::bit_cast<uint32_t>(std::round(p.z * 1e4f)));

    math::Vec3f ref{0.0f, 0.0f, 1.0f};
    if (std::abs(dir.dot(ref)) > 0.99f)
        ref = math::Vec3f{1.0f, 0.0f, 0.0f};
    math::Vec3f b1 = ref.cross(dir);
    const float b1l = b1.length();
    if (b1l < 1e-12f)
        return p;
    b1 = b1 / b1l;
    const math::Vec3f b2 = dir.cross(b1);

    const float u = ((h & 0xffffu) * (1.0f / 65535.0f) - 0.5f) * scale;
    const float v = (((h >> 16u) & 0xffffu) * (1.0f / 65535.0f) - 0.5f) * scale;
    return p + b1 * u + b2 * v;
}

// Parity point-in-mesh: odd crossings = inside. `eps` is the on-surface
// tolerance (triangles containing p within eps are skipped).
bool point_inside_mesh(const math::Vec3f& p,
                       const std::vector<math::Vec3f>& positions,
                       const std::vector<uint32_t>& indices,
                       float eps, float jitterScale)
{
    if (positions.empty() || indices.size() % 3 != 0)
        return false;
    // Fixed, normalized query direction (not axis-aligned).
    const math::Vec3f dir = math::Vec3f{0.6273812f, 0.2992783f, 0.7198231f}.normalized();
    if (dir.length_sq() <= 1e-12f)
        return false;
    const math::Vec3f origin = jitter_ray_origin(p, dir, jitterScale);
    if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z))
        return false;

    int count = 0;
    const size_t nt = indices.size() / 3;
    for (size_t t = 0; t < nt; ++t)
    {
        const math::Vec3f& a = positions[indices[t * 3 + 0]];
        const math::Vec3f& b = positions[indices[t * 3 + 1]];
        const math::Vec3f& c = positions[indices[t * 3 + 2]];
        if (point_on_triangle(p, a, b, c, eps))
            continue;   // on-surface → skip (no ambiguous crossing)
        if (ray_hits_triangle(origin, dir, a, b, c, eps))
            ++count;
    }
    return (count % 2) == 1;
}

// ── Triangle classification + splitting driver ─────────────────────────────

struct TriGeom
{
    std::array<uint32_t, 3> ci;             // canonical vertex indices
    std::array<uint32_t, 3> oi;             // original (mesh) vertex indices
    math::Vec3f             p0;             // canonical corner 0 (plane origin)
    math::Vec3f             n;              // unit geometric normal
    math::Vec3f             u, v;           // orthonormal 2D frame (u×v = n)
    std::array<P2, 3>       t2d;            // corners projected into the frame
};

bool make_tri_geom(const MeshData& m, const std::vector<math::Vec3f>& canonPos,
                   const std::vector<uint32_t>& remap, size_t tri,
                   float minArea, TriGeom& g)
{
    const size_t i = tri * 3;
    g.oi = {m.indices[i], m.indices[i + 1], m.indices[i + 2]};
    g.ci = {remap[g.oi[0]], remap[g.oi[1]], remap[g.oi[2]]};
    g.p0 = canonPos[g.ci[0]];
    const math::Vec3f& p1 = canonPos[g.ci[1]];
    const math::Vec3f& p2 = canonPos[g.ci[2]];
    const math::Vec3f e0 = p1 - g.p0;
    const math::Vec3f e1 = p2 - g.p0;
    const math::Vec3f nrm = e0.cross(e1);
    const float nl = nrm.length();
    if (nl < minArea)   // degenerate area → drop the triangle entirely
        return false;
    g.n = nrm / nl;
    g.u = e0.normalized();
    g.v = g.n.cross(g.u);
    g.t2d[0] = P2{0.0f, 0.0f};
    g.t2d[1] = P2{e0.length(), 0.0f};
    g.t2d[2] = P2{e1.dot(g.u), e1.dot(g.v)};
    return true;
}

// Appends the kept triangles of `m` (the "split" mesh) to outVerts/outIdx.
// Classification samples the OTHER mesh's canonical positions/indices.
// `keepInside` selects the keep rule; `flip` (subtract B-parts) flips triangle
// winding and negates stored vertex normals. Returns false to abort the whole
// op on a coplanar-overlap hit (V1 limitation).
//
// Processing is FACE-LOCAL: triangles are grouped into coplanar faces and each
// face is cut/classified as a whole. This keeps the subdivision of shared seam
// edges identical between adjacent coplanar triangles and between the two input
// meshes' complementary pieces, which is what lets the assembled result pass the
// watertight gate.
bool collect_kept_triangles(
    const MeshData& m,
    const std::vector<math::Vec3f>& canonPos,
    const std::vector<uint32_t>& remap,
    const std::vector<math::Vec3f>& otherPos,
    const std::vector<uint32_t>& otherIdx,
    bool keepInside, bool flip,
    float planeTol, float snapTol, float minArea, float interEps,
    float epsClass,
    std::vector<Vertex>& outVerts,
    std::vector<uint32_t>& outIdx)
{
    // ── Group triangles into coplanar faces ──
    struct Face
    {
        math::Vec3f n;                       // unit outward face normal
        math::Vec3f p0, u, v;                // frame (u×v = n)
        std::array<P2, 3>      firstT2D;     // first triangle corners (face frame)
        std::array<uint32_t, 3> firstOi;     // first triangle original ordinals
        std::vector<size_t>    tris;
        std::vector<std::array<P2, 3>> tris2D;
        std::vector<P2>        outline2D;    // CCW convex outline (hull)
        std::vector<uint32_t>  outlineOi;    // original vertex index per outline point
    };

    const size_t nt = m.indices.size() / 3;
    std::vector<Face> faces;
    for (size_t t = 0; t < nt; ++t)
    {
        TriGeom tg;
        if (!make_tri_geom(m, canonPos, remap, t, minArea, tg))
            continue;   // degenerate triangle: dropped entirely
        Face* face = nullptr;
        for (Face& f : faces)
        {
            bool same = f.n.dot(tg.n) > 0.99f;
            if (same)
            {
                for (int k = 0; k < 3; ++k)
                {
                    if (std::abs(f.n.dot(canonPos[tg.ci[k]] - f.p0)) > planeTol)
                    {
                        same = false;
                        break;
                    }
                }
            }
            if (same)
            {
                face = &f;
                break;
            }
        }
        if (!face)
        {
            faces.push_back(Face{});
            face = &faces.back();
            face->n = tg.n;
            face->p0 = tg.p0;
            face->u = tg.u;
            face->v = tg.v;
            face->firstT2D = tg.t2d;
            face->firstOi = tg.oi;
        }
        face->tris.push_back(t);
        face->tris2D.push_back(tg.t2d);
    }

    // Build convex face outlines + original-vertex mapping.
    const float snap2 = snapTol * snapTol;
    for (Face& f : faces)
    {
        std::vector<std::pair<P2, uint32_t>> pts;
        for (size_t ti = 0; ti < f.tris.size(); ++ti)
        {
            const size_t t = f.tris[ti];
            for (int k = 0; k < 3; ++k)
            {
                const uint32_t oi = m.indices[t * 3 + k];
                const P2 p = project_to_2d(canonPos[remap[oi]], f.p0, f.u, f.v);
                bool dup = false;
                for (const auto& q : pts)
                {
                    if (dist2(q.first, p) <= snap2)
                    {
                        dup = true;
                        break;
                    }
                }
                if (!dup)
                    pts.push_back({p, oi});
            }
        }
        if (pts.size() < 3)
            continue;
        std::vector<P2> raw;
        raw.reserve(pts.size());
        for (const auto& q : pts)
            raw.push_back(q.first);
        f.outline2D = convex_hull(raw);
        for (const P2& hp : f.outline2D)
        {
            uint32_t oi = 0u;
            float    best = std::numeric_limits<float>::max();
            for (const auto& q : pts)
            {
                const float d = dist2(q.first, hp);
                if (d < best)
                {
                    best = d;
                    oi = q.second;
                }
            }
            f.outlineOi.push_back(oi);
        }
    }

    const float jitterScale = epsClass;

    auto push_vertex = [&outVerts, flip](Vertex v) -> uint32_t
    {
        if (flip)
            v.normal = math::Vec3f{} - v.normal;
        outVerts.push_back(v);
        return static_cast<uint32_t>(outVerts.size() - 1);
    };

    // Emit a kept piece (face-level polygon) with per-vertex attributes.
    auto emit_piece = [&](const Face& f, const ArrangedPiece& piece)
    {
        if (piece.ring2D.size() < 3)
            return;
        math::Vec3f centroid{0.0f, 0.0f, 0.0f};
        for (const auto& p3 : piece.ring3D)
            centroid += p3;
        centroid = centroid * (1.0f / static_cast<float>(piece.ring3D.size()));
        const math::Vec3f pt = centroid + f.n * epsClass;
        const bool inside = point_inside_mesh(pt, otherPos, otherIdx, planeTol, jitterScale);
        const bool keep = keepInside ? inside : !inside;
        if (!keep)
            return;

        std::vector<uint32_t> vi;
        vi.reserve(piece.ring2D.size());
        const float area2 = cross2(f.firstT2D[0], f.firstT2D[1], f.firstT2D[2]);
        for (size_t k = 0; k < piece.ring2D.size(); ++k)
        {
            int match = -1;
            for (size_t i = 0; i < f.outline2D.size(); ++i)
            {
                if (dist2(piece.ring2D[k], f.outline2D[i]) <= snap2)
                {
                    match = static_cast<int>(i);
                    break;
                }
            }
            if (match >= 0)
            {
                if (!flip)
                    vi.push_back(f.outlineOi[static_cast<size_t>(match)]); // reuse pool vertex
                else
                    vi.push_back(push_vertex(m.vertices[f.outlineOi[static_cast<size_t>(match)]]));
            }
            else
            {
                Vertex nv;
                nv.position = piece.ring3D[k];
                nv.normal   = f.n;
                if (std::abs(area2) > 1e-12f)
                {
                    const float w0 = cross2(piece.ring2D[k], f.firstT2D[1], f.firstT2D[2]) / area2;
                    const float w1 = cross2(f.firstT2D[0], piece.ring2D[k], f.firstT2D[2]) / area2;
                    const float w2 = 1.0f - w0 - w1;
                    const Vertex& v0 = m.vertices[f.firstOi[0]];
                    const Vertex& v1 = m.vertices[f.firstOi[1]];
                    const Vertex& v2 = m.vertices[f.firstOi[2]];
                    nv.uv = v0.uv * w0 + v1.uv * w1 + v2.uv * w2;
                }
                nv.tangent = math::Quat{1.0f, 0.0f, 0.0f, 1.0f};
                nv.color   = m.vertices[f.firstOi[0]].color;
                vi.push_back(push_vertex(std::move(nv)));
            }
        }

        const std::vector<uint32_t> tris = triangulate_polygon(piece.ring3D);
        if (tris.size() % 3 != 0)
            return;   // triangulation failure → skip the piece
        if (!flip)
        {
            for (uint32_t idx : tris)
                outIdx.push_back(vi[idx]);
        }
        else
        {
            for (size_t k = 0; k < tris.size(); k += 3)
            {
                outIdx.push_back(vi[tris[k]]);
                outIdx.push_back(vi[tris[k + 2]]);
                outIdx.push_back(vi[tris[k + 1]]);   // reversed winding
            }
        }
    };

    // Emit a kept verbatim triangle (original ordinals in original order when
    // non-flip — preserves identity for disjoint/untouched meshes).
    auto emit_verbatim = [&](const TriGeom& tg)
    {
        if (!flip)
        {
            outIdx.push_back(tg.oi[0]);
            outIdx.push_back(tg.oi[1]);
            outIdx.push_back(tg.oi[2]);
        }
        else
        {
            outIdx.push_back(push_vertex(m.vertices[tg.oi[0]]));
            outIdx.push_back(push_vertex(m.vertices[tg.oi[2]]));
            outIdx.push_back(push_vertex(m.vertices[tg.oi[1]]));
        }
    };

    const size_t on = otherIdx.size() / 3;

    for (Face& f : faces)
    {
        if (f.outline2D.size() < 3)
            continue;

        // ── Gather cut chords against the OTHER mesh, clipped to the face ──
        std::vector<Seg2> segs;
        for (size_t tb = 0; tb < on; ++tb)
        {
            const math::Vec3f& q0 = otherPos[otherIdx[tb * 3 + 0]];
            const math::Vec3f& q1 = otherPos[otherIdx[tb * 3 + 1]];
            const math::Vec3f& q2 = otherPos[otherIdx[tb * 3 + 2]];

            const float d0 = f.n.dot(q0 - f.p0);
            const float d1 = f.n.dot(q1 - f.p0);
            const float d2 = f.n.dot(q2 - f.p0);

            // Coplanar pair.
            if (std::abs(d0) <= planeTol && std::abs(d1) <= planeTol && std::abs(d2) <= planeTol)
            {
                const P2 qa = project_to_2d(q0, f.p0, f.u, f.v);
                const P2 qb = project_to_2d(q1, f.p0, f.u, f.v);
                const P2 qc = project_to_2d(q2, f.p0, f.u, f.v);
                for (const auto& t2d : f.tris2D)
                {
                    if (triangles_overlap_2d(t2d[0], t2d[1], t2d[2], qa, qb, qc))
                        return false;   // coplanar overlap → V1 limitation → {}
                }
                continue;   // coplanar, no interior overlap → no split
            }

            // All on one side (within tolerance) → no crossing.
            if ((d0 >= -planeTol && d1 >= -planeTol && d2 >= -planeTol) ||
                (d0 <= planeTol && d1 <= planeTol && d2 <= planeTol))
                continue;

            math::Vec3f s0, s1;
            if (!plane_triangle_segment(f.n, f.p0, q0, q1, q2, planeTol, s0, s1))
                continue;
            const P2 c0 = project_to_2d(s0, f.p0, f.u, f.v);
            const P2 c1 = project_to_2d(s1, f.p0, f.u, f.v);
            P2 o0, o1;
            if (!clip_segment_to_polygon(c0, c1, f.outline2D, interEps, o0, o1))
                continue;
            segs.push_back(Seg2{o0, o1});
        }

        // ── Short-circuit: no interior chords → per-triangle verbatim ──
        if (segs.empty())
        {
            for (const size_t t : f.tris)
            {
                TriGeom tg;
                if (!make_tri_geom(m, canonPos, remap, t, minArea, tg))
                    continue;
                const math::Vec3f centroid =
                    (canonPos[tg.ci[0]] + canonPos[tg.ci[1]] + canonPos[tg.ci[2]]) * (1.0f / 3.0f);
                const math::Vec3f pt = centroid + tg.n * epsClass;
                const bool inside = point_inside_mesh(pt, otherPos, otherIdx, planeTol, jitterScale);
                const bool keep = keepInside ? inside : !inside;
                if (!keep)
                    continue;
                emit_verbatim(tg);
            }
            continue;
        }

        // ── Arrange the whole face, then classify/keep each piece ──
        const std::vector<ArrangedPiece> pieces =
            arrange_triangle(f.outline2D, segs, f.p0, f.u, f.v, snapTol, minArea);
        for (const ArrangedPiece& piece : pieces)
            emit_piece(f, piece);
    }
    return true;
}

// ── T-junction resolution (stitching) ─────────────────────────────────────
// The per-face arrangement subdivides cut faces independently, so a face that
// is cut may carry subdivision vertices on a shared edge while its neighbour
// is not. The watertight gate requires every edge to be shared by exactly two
// triangles, so we SPLIT every edge (a,b) whose interior is touched by an
// existing vertex v, subdividing the two triangles sharing that edge at v.
// Iterated to a fixed point (splitting may expose new T-junctions).
void resolve_t_junctions(MeshData& mesh, float tol)
{
    if (mesh.vertices.empty() || tol <= 0.0f)
        return;
    const float tol2 = tol * tol;

    struct EdgeKey
    {
        uint32_t a, b;   // a <= b
        bool operator==(const EdgeKey& o) const { return a == o.a && b == o.b; }
    };
    struct EdgeHash
    {
        size_t operator()(const EdgeKey& k) const
        {
            return (static_cast<size_t>(k.a) << 32u) ^ static_cast<size_t>(k.b);
        }
    };

    for (size_t iter = 0; iter < 16; ++iter)
    {
        const size_t nTris = mesh.indices.size() / 3;
        std::unordered_map<EdgeKey, std::array<uint32_t, 2>, EdgeHash> edgeTris;
        for (size_t t = 0; t < nTris; ++t)
        {
            const uint32_t i0 = mesh.indices[t * 3 + 0];
            const uint32_t i1 = mesh.indices[t * 3 + 1];
            const uint32_t i2 = mesh.indices[t * 3 + 2];
            auto add = [&](uint32_t a, uint32_t b, uint32_t slot)
            {
                const EdgeKey k{std::min(a, b), std::max(a, b)};
                auto& arr = edgeTris[k];
                if (arr[0] == 0 && arr[1] == 0)
                    arr[0] = slot;
                else if (arr[1] == 0)
                    arr[1] = slot;   // third+ incident triangles left alone
            };
            add(i0, i1, static_cast<uint32_t>(t * 3 + 0));
            add(i1, i2, static_cast<uint32_t>(t * 3 + 1));
            add(i2, i0, static_cast<uint32_t>(t * 3 + 2));
        }

        std::vector<uint32_t> kept;
        kept.reserve(mesh.indices.size());
        bool any = false;
        for (size_t t = 0; t < nTris; ++t)
        {
            const uint32_t i0 = mesh.indices[t * 3 + 0];
            const uint32_t i1 = mesh.indices[t * 3 + 1];
            const uint32_t i2 = mesh.indices[t * 3 + 2];

            // Find a vertex (with that triangle index ≠ any of its corners)
            // that lies strictly inside one of the triangle's edges.
            uint32_t vi = UINT32_MAX;
            for (int c = 0; c < 3 && vi == UINT32_MAX; ++c)
            {
                const uint32_t aC = mesh.indices[t * 3 + c];
                const uint32_t bC = mesh.indices[t * 3 + ((c + 1) % 3)];
                const math::Vec3f& pA = mesh.vertices[aC].position;
                const math::Vec3f& pB = mesh.vertices[bC].position;
                const math::Vec3f ab = pB - pA;
                const float abLen2 = ab.length_sq();
                if (abLen2 <= tol2)
                    continue;
                for (uint32_t v = 0; v < mesh.vertices.size(); ++v)
                {
                    const math::Vec3f& pv = mesh.vertices[v].position;
                    const float tv = (pv - pA).dot(ab) / abLen2;
                    if (tv <= 1e-6f || tv >= 1.0f - 1e-6f)
                        continue;   // endpoint, not a T-vertex
                    const math::Vec3f closest = pA + ab * tv;
                    if ((pv - closest).length_sq() <= tol2)
                    {
                        vi = v;
                        break;
                    }
                }
                if (vi != UINT32_MAX)
                {
                    const uint32_t oC = mesh.indices[t * 3 + ((c + 2) % 3)];
                    kept.push_back(aC);
                    kept.push_back(vi);
                    kept.push_back(oC);
                    kept.push_back(vi);
                    kept.push_back(bC);
                    kept.push_back(oC);
                    any = true;
                    break;
                }
            }
            if (vi == UINT32_MAX)
            {
                kept.push_back(i0);
                kept.push_back(i1);
                kept.push_back(i2);
            }
        }

        if (!any)
            break;
        mesh.indices = std::move(kept);
    }
}

MeshData assemble_side(std::vector<Vertex>& verts, std::vector<uint32_t>& idx)
{
    MeshData m;
    m.topology = PrimitiveTopology::Triangles;
    m.vertices = std::move(verts);
    m.indices  = std::move(idx);
    return m;
}

// Drop vertices not referenced by any surviving triangle (pool copies of
// dropped faces / unused corners). Keeps the result mesh compact so bounds and
// gates reflect the actual retained geometry.
void prune_vertices(MeshData& mesh)
{
    std::vector<uint32_t> remap(mesh.vertices.size(), UINT32_MAX);
    std::vector<Vertex>   out;
    out.reserve(mesh.vertices.size());
    for (const uint32_t idx : mesh.indices)
    {
        if (remap[idx] == UINT32_MAX)
        {
            remap[idx] = static_cast<uint32_t>(out.size());
            out.push_back(mesh.vertices[idx]);
        }
    }
    for (uint32_t& idx : mesh.indices)
        idx = remap[idx];
    mesh.vertices = std::move(out);
    mesh.bounds   = compute_bounds(mesh.vertices);
}

// Remap a mesh index buffer through a canonical map (used for the "other mesh"
// parity queries, which index into the canonical position array).
std::vector<uint32_t> remap_indices(const MeshData& m, const CanonicalMap& cm)
{
    std::vector<uint32_t> out;
    out.reserve(m.indices.size());
    for (size_t i = 0; i < m.indices.size(); ++i)
        out.push_back(cm.remap[m.indices[i]]);
    return out;
}

} // namespace

MeshData boolean_mesh(const MeshData& a, const MeshData& b, BooleanOp op,
                      float weldEpsilon)
{
    // ── Guards ──
    if (a.vertices.empty() || a.indices.empty() ||
        b.vertices.empty() || b.indices.empty())
        return {};
    if (a.indices.size() % 3 != 0 || b.indices.size() % 3 != 0)
        return {};

    const Bounds ba = compute_bounds(a.vertices);
    const Bounds bb = compute_bounds(b.vertices);
    const math::Vec3f da = ba.max - ba.min;
    const math::Vec3f db = bb.max - bb.min;
    if (da.length_sq() <= 0.0f || db.length_sq() <= 0.0f)
        return {};
    const float diagA = da.length();
    const float diagB = db.length();
    const float diag  = std::max(diagA, diagB);

    // ── Tolerances (scale-relative to max input diagonal) ──
    const float weld      = (weldEpsilon > 0.0f) ? weldEpsilon : 1e-4f * diag;
    const float epsClass  = weld * 0.5f;
    const float planeTol  = 1e-6f * diag;
    const float posEpsA   = 1e-7f * diagA;
    const float posEpsB   = 1e-7f * diagB;
    const float snapTol   = planeTol;
    const float interEps  = 1e-7f * diag;
    const float minArea   = 1e-9f * diag * diag;

    // ── Winding normalization (helper 2) + gates (helper 1) ──
    bool volAOk = false, volBOk = false;
    const MeshData mA = normalize_to_outward(a, volAOk);
    const MeshData mB = normalize_to_outward(b, volBOk);
    if (!volAOk || !volBOk)
        return {};
    if (!closed_manifold_gate(mA, posEpsA) || !closed_manifold_gate(mB, posEpsB))
        return {};

    // ── Canonical maps for the split loop ──
    const std::vector<math::Vec3f> posA = extract_positions(mA);
    const std::vector<math::Vec3f> posB = extract_positions(mB);
    const CanonicalMap cmA = canonicalize_positions(posA, posEpsA);
    const CanonicalMap cmB = canonicalize_positions(posB, posEpsB);

    // ── Keep rules ──
    const bool keepAInside = (op == BooleanOp::Intersect);
    const bool keepBInside = (op != BooleanOp::Union);   // subtract & intersect
    const bool flipB       = (op == BooleanOp::Subtract);

    // A-side pool keeps the original vertex array (identity preserved when the
    // whole side survives with no splits). B-side pool: only when non-flip,
    // since subtract pushes negated copies.
    std::vector<Vertex>   aVerts = mA.vertices;
    std::vector<uint32_t> aIdx;
    aIdx.reserve(mA.indices.size());
    std::vector<Vertex>   bVerts = flipB ? std::vector<Vertex>{} : mB.vertices;
    std::vector<uint32_t> bIdx;
    bIdx.reserve(mB.indices.size());

    const bool okA = collect_kept_triangles(
        mA, cmA.positions, cmA.remap, cmB.positions, remap_indices(mB, cmB),
        keepAInside, /*flip=*/false, planeTol, snapTol, minArea, interEps, epsClass,
        aVerts, aIdx);
    if (!okA)
        return {};
    const bool okB = collect_kept_triangles(
        mB, cmB.positions, cmB.remap, cmA.positions, remap_indices(mA, cmA),
        keepBInside, flipB, planeTol, snapTol, minArea, interEps, epsClass,
        bVerts, bIdx);
    if (!okB)
        return {};

    // ── Assembly ──
    if (aIdx.empty() && bIdx.empty())
        return {};
    std::vector<MeshData> parts;
    parts.reserve(2);
    if (!aIdx.empty())
        parts.push_back(assemble_side(aVerts, aIdx));
    if (!bIdx.empty())
        parts.push_back(assemble_side(bVerts, bIdx));
    MeshData assembled = merge_meshes(parts);

    // ── Weld, stitch T-junctions, drop post-weld degenerates, re-gate ──
    MeshData result = weld_vertices(assembled, weld);
    resolve_t_junctions(result, weld);

    const Bounds ob = compute_bounds(result.vertices);
    const float od = (ob.max - ob.min).length();
    if (od <= 0.0f)
        return {};
    const float outMinArea = 1e-9f * od * od;
    std::vector<uint32_t> outIdx;
    outIdx.reserve(result.indices.size());
    const size_t wn = result.indices.size() / 3;
    for (size_t t = 0; t < wn; ++t)
    {
        const math::Vec3f& pa = result.vertices[result.indices[t * 3 + 0]].position;
        const math::Vec3f& pb = result.vertices[result.indices[t * 3 + 1]].position;
        const math::Vec3f& pc = result.vertices[result.indices[t * 3 + 2]].position;
        if ((pb - pa).cross(pc - pa).length() < 2.0f * outMinArea)
            continue;   // degenerate triangle dropped post-weld
        outIdx.push_back(result.indices[t * 3 + 0]);
        outIdx.push_back(result.indices[t * 3 + 1]);
        outIdx.push_back(result.indices[t * 3 + 2]);
    }
    result.indices = std::move(outIdx);
    prune_vertices(result);

    const Bounds ob2 = compute_bounds(result.vertices);
    const float od2 = (ob2.max - ob2.min).length();
    if (od2 <= 0.0f)
        return {};
    if (!closed_manifold_gate(result, 1e-7f * od2))
        return {};

    result.bounds = compute_bounds(result.vertices);
    return result;
}

} // namespace exd::geometry

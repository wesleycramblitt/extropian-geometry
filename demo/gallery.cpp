/// ═══════════════════════════════════════════════════════════════════════
///  extropian-geometry · gallery demo
///
///  A 3D, tab-through showcase of every use case in extropian-geometry,
///  rendered by extropian-render (OpenGL). One ImGui tab per capability:
///
///     0  2D primitives        5  SDF blend          10 turbine
///     1  3D primitives        6  workspace ops      11 steam engine
///     2  paths & splines      7  terrain/heightmap  12 mechanisms & FK
///     3  loft                  8  deformation        13 import/export
///     4  extrusion               9  compressor       14 gizmos
///
///  Navigation: orbit camera (left-drag orbit, middle-drag pan, scroll
///  zoom), Esc quits, Tab toggles UI/free-fly input. Sliders drive the
///  animated tabs (blend radius, crank angle, gear ratio).
///
///  Build: ./demo.sh (or cmake -DBUILD_DEMO=ON ...).
/// ═══════════════════════════════════════════════════════════════════════

#include <exd/app/window.hpp>
#include <exd/app/input_mode.hpp>
#include <exd/ecs/registry.hpp>
#include <exd/math/mat4.hpp>
#include <exd/render/graphics/gl_loader.hpp>
#include <exd/render/graphics/graphics_context.hpp>
#include <exd/render/components/transform.hpp>
#include <exd/render/components/camera_component.hpp>
#include <exd/render/components/camera_mode.hpp>
#include <exd/render/components/material.hpp>
#include <exd/render/components/renderable.hpp>
#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/grid.hpp>
#include <exd/render/components/gradient_sky.hpp>
#include <exd/render/components/environment.hpp>
#include <exd/render/systems/camera_mode_system.hpp>
#include <exd/render/systems/grid_system.hpp>
#include <exd/render/systems/gradient_sky_system.hpp>
#include <exd/render/systems/render_system.hpp>
#include <exd/render/systems/imgui_system.hpp>
#include <exd/geometry/geometry.hpp>

#include <SDL3/SDL.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace exd;

static math::Quat col(float r, float g_, float b) { return math::Quat{r, g_, b, 1.0f}; }

// ═══════════════════════════════════════════════════════════════════════
// Gallery state
// ═══════════════════════════════════════════════════════════════════════

struct Gallery
{
    ecs::Entity cam_entity{};
    int   current_tab = 0;
    bool  build_now   = true;      // rebuild scene (tab change / slider)
    std::string info;

    std::vector<uint32_t> meshes;    // GPU mesh handles owned by the gallery
    std::vector<uint32_t> entities;  // ECS entities owned by the gallery

    // animated tab state
    int   steam_angle  = 0;
    float mech_state   = 0.0f;
    float blend_radius = 0.20f;

    long frame = 0;
};

struct TabDef
{
    const char* name;
    const char* info;
    math::Vec3f target;
    float dist, azim, elev;
};

// 1:1 order with build_tab() below.
static const TabDef kTabs[] = {
    {"2D primitives",
     "Rect, rounded rect, circle, ellipse, arc, ring, line, polyline, arrow, "
     "star, regular polygon, grid — each descriptor + generator -> MeshData.",
     {0, 0, 0}, 7.5f, -0.7f, 0.72f},
    {"3D primitives",
     "Sphere, box, cylinder, cone, capsule, torus, tube, disk, ellipsoid, "
     "icosahedron, arrow3d, axes, billboard.",
     {0, 0, 0}, 8.5f, 0.6f, 0.55f},
    {"Paths & splines",
     "Path2D fill + stroke tessellation and a MonotoneCubicSpline driving a "
     "tube sweep.",
     {0, 1, 0}, 6.5f, -0.4f, 0.6f},
    {"Loft",
     "generate_loft_mesh skins equal-count rings (circle -> star -> circle -> "
     "teardrop) with smooth normals.",
     {0, 1.8f, 0}, 6.0f, 0.5f, 0.5f},
    {"Extrusion",
     "Profile extrusion, lathe (revolve sweeps a profile about an axis), and "
     "a helix sweep.",
     {0, 1.2f, 0}, 8.5f, -0.5f, 0.55f},
    {"SDF blend",
     "Marching-cubes isosurface extraction over combined SDF fields: smooth "
     "union and a subtract pocket. Drag the blend radius.",
     {0, 0, 0}, 6.5f, 0.6f, 0.5f},
    {"Workspace ops",
     "Exact CSG boolean (box - cylinder bore), transform, weld, normal "
     "recompute, and Mirtich mass properties.",
     {0, 0, 0}, 6.5f, -0.6f, 0.52f},
    {"Terrain",
     "Procedural terrain presets (Mountains) plus a Noise2D-driven heightmap "
     "mesh.",
     {0, 0.5f, 0}, 13.0f, 0.3f, 0.65f},
    {"Deformation",
     "deform_mesh bend / twist / taper + noise applied to a cylinder.",
     {0, 0.5f, 0}, 7.0f, -0.5f, 0.5f},
    {"Compressor",
     "Axial compressor recipe: spinner, casing, IGV + one rotor/stator stage "
     "as a patched Assembly (flattened for display).",
     {0, 0, 2}, 8.0f, 0.8f, 0.45f},
    {"Turbine",
     "Turbine recipe: spinner hub, flow path, stator + rotor blade rows with "
     "stacked airfoil sections.",
     {0, 0, 1.2f}, 8.0f, -0.8f, 0.45f},
    {"Steam engine",
     "Single-cylinder steam engine (crank-slider): cylinder, steam chest, "
     "piston, crosshead, conrod, flywheel, crankshaft. Drag the crank angle.",
     {0, 0, 0.7f}, 4.5f, 0.5f, 0.35f},
    {"Mechanisms & FK",
     "Motion-graph contract: two gears coupled at ratio -2, posed by "
     "evaluate_poses()/apply_poses(). Drag the drive angle.",
     {0, 0, 0}, 6.0f, -0.5f, 0.5f},
    {"Import / export",
     "CADModel -> STL / Gmsh / faceted STEP -> reimport into CADModel: the "
     "unified mesh round-trip.",
     {0, 0, 0}, 7.0f, 0.4f, 0.5f},
    {"Gizmos",
     "Translation / rotation / scale interaction handles and a lattice cage.",
     {0, 0, 0}, 8.0f, 0.5f, 0.5f},
};
constexpr int kTabCount = static_cast<int>(sizeof(kTabs) / sizeof(kTabs[0]));

// ═══════════════════════════════════════════════════════════════════════
// Scene builder
// ═══════════════════════════════════════════════════════════════════════

static std::vector<math::Vec3f> ring_circle(float r, float z, uint32_t n, float rot = 0.0f)
{
    std::vector<math::Vec3f> out;
    out.reserve(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        const float a = rot + 6.283185307179586f * static_cast<float>(i) / static_cast<float>(n);
        out.push_back({std::cos(a) * r, std::sin(a) * r, z});
    }
    return out;
}

static std::vector<math::Vec3f> ring_star(float r0, float r1, float z, uint32_t points)
{
    std::vector<math::Vec3f> out;
    out.reserve(points * 2);
    for (uint32_t i = 0; i < points; ++i)
    {
        const float a0 = 6.283185307179586f * static_cast<float>(i) / static_cast<float>(points);
        const float a1 = a0 + 6.283185307179586f / (2.0f * static_cast<float>(points));
        out.push_back({std::cos(a0) * r0, std::sin(a0) * r0, z});
        out.push_back({std::cos(a1) * r1, std::sin(a1) * r1, z});
    }
    return out;
}

static void build_tab(Gallery& g, ecs::Registry& reg, render::GraphicsContext& ctx)
{
    using namespace geometry;

    // ── shared spawn helpers ───────────────────────────────────────────
    auto spawn_e = [&](uint32_t h, math::Vec3f pos, math::Quat tint,
                       math::Vec3f scale = {1, 1, 1}, math::Quat rot = {1, 0, 0, 0}) {
        auto e = reg.create("G");
        reg.emplace<render::Transform>(e, pos, rot, scale);
        reg.emplace<render::RenderableComponent>(e, h);
        reg.emplace<render::RenderTechnique_Lambertian>(e);
        reg.emplace<render::Material>(e, tint);
        g.entities.push_back(e.id);
        return e.id;
    };
    auto show_mesh = [&](const MeshData& m, math::Vec3f pos, math::Quat tint,
                         math::Vec3f scale = {1, 1, 1}, math::Quat rot = {1, 0, 0, 0}) {
        const uint32_t h = ctx.mesh_manager.create(m);
        g.meshes.push_back(h);
        spawn_e(h, pos, tint, scale, rot);
    };
    auto show_mesh_vc = [&](const MeshData& m, math::Vec3f pos, math::Vec3f scale = {1, 1, 1}) {
        // vertex colors carry the tint (2D shapes bake colors into vertices)
        show_mesh(m, pos, math::Quat{1, 1, 1, 1}, scale);
    };

    if (g.current_tab == 0)   // ── 2D primitives ─────────────────────
    {
        show_mesh_vc(generate_rect_mesh(RectangleGeometry{{1.6f, 1.0f, 0}, col(0.90f, 0.35f, 0.35f)}), {-3.1f, 2.2f, 0});
        show_mesh_vc(generate_rounded_rect_mesh(RoundedRectangleGeometry{{1.6f, 1.0f, 0}, {0.3f, 0.3f, 0.3f, 0.3f}, 32, col(0.95f, 0.60f, 0.25f)}), {-1.05f, 2.2f, 0});
        show_mesh_vc(generate_circle_mesh(CircleGeometry{0.75f, 64, col(0.55f, 0.75f, 0.95f)}), {1.05f, 2.2f, 0});
        show_mesh_vc(generate_ellipse_mesh(EllipseGeometry{0.85f, 0.45f, 64, col(0.70f, 0.45f, 0.90f)}), {3.1f, 2.2f, 0});
        show_mesh_vc(generate_arc_mesh(ArcGeometry{0.7f, 0.0f, 4.71f, 64, col(0.40f, 0.85f, 0.70f)}), {-3.1f, 0.0f, 0});
        show_mesh_vc(generate_ring_mesh(RingGeometry{0.85f, 0.45f, 64, col(0.95f, 0.80f, 0.35f)}), {-1.05f, 0.0f, 0});
        show_mesh_vc(generate_line_mesh(LineGeometry{{-0.7f, -0.4f, 0}, {0.7f, 0.4f, 0}, 0.06f, col(0.45f, 0.70f, 0.60f)}), {1.05f, 0.0f, 0});
        show_mesh_vc(generate_polyline_mesh(PolylineGeometry{{{-2.0f, -0.4f, 0}, {-1.0f, 0.5f, 0}, {0.0f, -0.2f, 0}, {1.0f, 0.4f, 0}, {2.0f, -0.3f, 0}}, 0.05f, false, col(0.35f, 0.60f, 0.95f)}), {3.1f, 0.0f, 0});
        show_mesh_vc(generate_arrow_mesh(ArrowGeometry{{0, 0, 0}, {1.4f, 0, 0}, 0.35f, 0.18f, 0.05f, col(0.95f, 0.40f, 0.55f)}), {-2.15f, -2.2f, 0});
        show_mesh_vc(generate_star_mesh(StarGeometry{0.8f, 0.3f, 5, col(0.95f, 0.75f, 0.30f)}), {-0.35f, -2.2f, 0});
        show_mesh_vc(generate_regular_polygon_mesh(RegularPolygonGeometry{0.7f, 6, col(0.55f, 0.80f, 0.50f)}), {1.45f, -2.2f, 0});
        show_mesh_vc(generate_grid_mesh(GridGeometry{{1.6f, 1.1f, 0}, 9, 9, 0.015f, col(0.75f, 0.55f, 0.90f)}), {3.15f, -2.2f, 0});
    }
    else if (g.current_tab == 1)   // ── 3D primitives ─────────────────
    {
        std::vector<math::Vec3f> cells;
        for (int iy = 0; iy < 3; ++iy)
            for (int ix = 0; ix < 5; ++ix)
                cells.push_back({-4.4f + ix * 2.2f, 1.2f, -1.0f + iy * 2.2f});
        int i = 0;
        auto at = [&]() { return cells[std::min(i, static_cast<int>(cells.size()) - 1)]; };
        show_mesh(generate_sphere_mesh(SphereGeometry{0.8f, 24, 48}), at(), col(0.95f, 0.40f, 0.40f)); ++i;
        show_mesh(generate_box_mesh(BoxGeometry{{1.3f, 1.3f, 1.3f}}), at(), col(0.95f, 0.65f, 0.30f)); ++i;
        show_mesh(generate_cylinder_mesh(CylinderGeometry{0.65f, 1.5f, 48}), at(), col(0.45f, 0.75f, 0.95f)); ++i;
        show_mesh(generate_cone_mesh(ConeGeometry{0.7f, 1.6f, 48}), at(), col(0.65f, 0.85f, 0.45f)); ++i;
        show_mesh(generate_capsule_mesh(CapsuleGeometry{0.42f, 0.9f, 48, 12}), at(), col(0.90f, 0.50f, 0.85f)); ++i;
        show_mesh(generate_torus_mesh(TorusGeometry{0.8f, 0.3f, 64, 24}), at(), col(0.40f, 0.80f, 0.80f)); ++i;
        show_mesh(generate_disk_mesh(DiskGeometry{0.9f, 0.0f, 64}), at(), col(0.95f, 0.85f, 0.40f)); ++i;
        show_mesh(generate_ellipsoid_mesh(EllipsoidGeometry{{0.55f, 0.9f, 0.55f}, 24, 48}), at(), col(0.55f, 0.55f, 0.95f)); ++i;
        show_mesh(generate_icosahedron_mesh(0.8f, 1), at(), col(0.75f, 0.45f, 0.35f)); ++i;
        show_mesh(generate_arrow3d_mesh(Arrow3DGeometry{{0, 0, 0}, {0, 1.1f, 0}, 0.14f, 0.30f, 0.05f, 24}), at(), col(0.95f, 0.60f, 0.45f)); ++i;
        show_mesh(generate_axes_mesh(AxesGeometry{1.1f}), at(), math::Quat{1, 1, 1, 1}); ++i;
        show_mesh(generate_billboard_mesh(BillboardGeometry{{1.2f, 0.8f, 0}}), at(), col(0.90f, 0.40f, 0.60f)); ++i;
        MeshData tube = generate_tube_mesh(TubeGeometry{ring_circle(0.8f, 0, 48), 0.1f, 24, true});
        show_mesh(tube, at(), col(0.40f, 0.85f, 0.55f)); ++i;
    }
    else if (g.current_tab == 2)   // ── paths & splines ───────────────
    {
        Path2D heart;
        heart.moveTo({0.0f, 0.30f, 0});
        heart.cubicTo({0.40f, 0.70f, 0}, {0.85f, 0.25f, 0}, {0.0f, 0.0f, 0});
        heart.cubicTo({-0.85f, 0.25f, 0}, {-0.40f, 0.70f, 0}, {0.0f, 0.30f, 0});
        heart.close();
        show_mesh_vc(heart.tessellateFill(FillRule::NonZero, 0.05f), {-2.4f, 0.0f, 0});
        show_mesh_vc(heart.tessellateStroke(StrokeStyle{0.045f, LineJoin::Round, LineCap::Round}, 0.05f), {-2.4f, 0.0f, 0.02f});

        // star path stroke
        Path2D star;
        float prev = 0.0f;
        for (int k = 0; k <= 20; ++k) { (void)prev;
            const float a = 6.283185307179586f * static_cast<float>(k) / 20.0f;
            const float r = (k % 2 == 0) ? 1.2f : 0.55f;
            if (k == 0) star.moveTo({std::cos(a) * r, std::sin(a) * r, 0});
            else        star.lineTo({std::cos(a) * r, std::sin(a) * r, 0});
        }
        star.close();
        show_mesh_vc(star.tessellateStroke(StrokeStyle{0.06f, LineJoin::Miter, LineCap::Round}, 0.05f), {1.6f, 3.0f, 0});

        // spline → tube sweep
        MonotoneCubicSpline spl({0, 1, 2, 3, 4, 5, 6},
                                {0.0f, 0.9f, 0.25f, 1.2f, -0.1f, 0.8f, 0.0f});
        std::vector<math::Vec3f> path;
        constexpr int kSamples = 48;
        for (int k = 0; k < kSamples; ++k)
        {
            const float x = 6.0f * static_cast<float>(k) / static_cast<float>(kSamples - 1);
            path.push_back({x, spl.evaluate(x), 0});
        }
        show_mesh(generate_tube_mesh(TubeGeometry{path, 0.09f, 24, true}),
                  {-2.0f, 0.0f, 0.6f}, col(0.30f, 0.70f, 0.90f));
    }
    else if (g.current_tab == 3)   // ── loft ──────────────────────────
    {
        std::vector<std::vector<math::Vec3f>> sections;
        sections.push_back(ring_circle(0.9f, 0.0f, 24));
        sections.push_back(ring_star(0.55f, 0.75f, 1.2f, 12));
        sections.push_back(ring_circle(1.0f, 2.4f, 24, 0.3f));
        sections.push_back(ring_circle(0.45f, 3.6f, 24, 0.9f));
        show_mesh(generate_loft_mesh(LoftGeometry{sections, true, col(0.80f, 0.50f, 0.30f)}),
                  {0, 0, 0}, math::Quat{1, 1, 1, 1}, {0.8f, 0.8f, 0.8f});
    }
    else if (g.current_tab == 4)   // ── extrusion / lathe / helix ─────
    {
        std::vector<math::Vec3f> starProfile = ring_star(0.6f, 0.45f, 0, 5);
        show_mesh(generate_extrusion_mesh(ExtrusionGeometry{starProfile, 2.2f, true, col(0.55f, 0.80f, 0.50f)}),
                  {-2.6f, 1.1f, 0}, math::Quat{1, 1, 1, 1});

        std::vector<math::Vec3f> vase = {{0.15f, 0.0f, 0}, {0.45f, 0.0f, 0}, {0.62f, 0.20f, 0},
                                         {0.55f, 0.60f, 0}, {0.30f, 0.80f, 0}, {0.12f, 0.95f, 0},
                                         {0.28f, 1.10f, 0}, {0.55f, 1.10f, 0}, {0.42f, 1.30f, 0}};
        show_mesh(generate_lathe_mesh(LatheGeometry{vase}), {0.0f, 0.7f, 0}, col(0.85f, 0.55f, 0.40f), {0.5f, 0.8f, 0.5f});

        std::vector<math::Vec3f> coilProfile = ring_circle(0.08f, 0, 8);
        show_mesh(generate_helix_mesh(HelixGeometry{coilProfile, 1.0f, 2.2f, 4.0f, 96}),
                  {2.7f, 1.1f, 0}, col(0.70f, 0.70f, 0.90f));
    }
    else if (g.current_tab == 5)   // ── SDF blend ─────────────────────
    {
        BlendGeometry unionBlend;
        unionBlend.op = BlendOp::Union;
        unionBlend.blendRadius = g.blend_radius;
        unionBlend.cellSize = 0.035f;
        unionBlend.primitives = {
            {BlendPrimitiveKind::Sphere, {-0.55f, 0.0f, 0.0f}, {}, {1, 1, 1}, 0.5f, 0.0f, 0.0f, 0.0f, 0},
            {BlendPrimitiveKind::Sphere, { 0.55f, 0.0f, 0.0f}, {}, {1, 1, 1}, 0.5f, 0.0f, 0.0f, 0.0f, 0},
            {BlendPrimitiveKind::Sphere, { 0.0f, 0.55f, 0.0f}, {}, {1, 1, 1}, 0.5f, 0.0f, 0.0f, 0.0f, 0},
            {BlendPrimitiveKind::Box,    { 0.0f, 0.0f, 0.5f}, {}, {1, 1, 1}, 0.0f, 0.0f, 0.0f, 0.4f, 0},
        };
        MeshData unionMesh = generate_blend_mesh(unionBlend);
        if (!unionMesh.vertices.empty())
            show_mesh(unionMesh, {-2.0f, 0.0f, 0}, col(0.45f, 0.80f, 0.95f));

        BlendGeometry minusBlend;
        minusBlend.op = BlendOp::Subtract;
        minusBlend.blendRadius = g.blend_radius;
        minusBlend.cellSize = 0.035f;
        minusBlend.primitives = {
            {BlendPrimitiveKind::Sphere, {0.0f, 0.0f, 0.0f}, {}, {1, 1, 1}, 0.75f, 0.0f, 0.0f, 0.0f, 0},
            {BlendPrimitiveKind::Capsule, {0.0f, 0.35f, 0.0f}, {}, {1, 1, 1}, 0.35f, 0.0f, 0.9f, 0.0f, 0},
        };
        MeshData minusMesh = generate_blend_mesh(minusBlend);
        if (!minusMesh.vertices.empty())
            show_mesh(minusMesh, {2.1f, 0.0f, 0}, col(0.90f, 0.65f, 0.35f));
    }
    else if (g.current_tab == 6)   // ── workspace ops ─────────────────
    {
        MeshData housing = boolean_mesh(
            generate_box_mesh(BoxGeometry{{2.1f, 1.5f, 1.5f}, col(0.80f, 0.58f, 0.36f)}),
            generate_cylinder_mesh(CylinderGeometry{0.34f, 8.0f, 48}),
            BooleanOp::Subtract);
        if (!housing.vertices.empty())
            show_mesh(housing, {-1.9f, 0.0f, 0}, math::Quat{1, 1, 1, 1});

        MeshData rotated = transform_mesh(
            generate_box_mesh(BoxGeometry{{1.1f, 1.1f, 1.1f}}),
            math::Mat4::trs({1.9f, 0.4f, 0}, math::Quat::from_axis_angle({0, 1, 0}, 0.9f), {1, 1, 1}));
        show_mesh(rotated, {0, 0, 0}, col(0.50f, 0.70f, 0.95f));
        MeshData welded = weld_vertices(rotated, 1e-4f);
        if (!welded.vertices.empty())
            show_mesh(welded, {3.6f, 0.0f, 0}, col(0.85f, 0.75f, 0.40f));
        if (!rotated.vertices.empty())
            g.info = "weld: " + std::to_string(rotated.vertices.size()) + " -> " +
                     std::to_string(welded.vertices.size()) + " verts (eps 1e-4)";
        if (!housing.vertices.empty())
        {
            const MassProperties mp = mesh_properties(housing, 2700.0f);
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "boolean housing | V=%.4f m3  A=%.3f m2  mass=%.2f kg  centroid=(%.2f,%.2f,%.2f)",
                          mp.volume, mp.surface_area, mp.mass, mp.centroid.x, mp.centroid.y, mp.centroid.z);
            g.info += std::string("\n") + buf;
        }
    }
    else if (g.current_tab == 7)   // ── terrain & heightmap ───────────
    {
        TerrainConfig mountains;
        mountains.kind = TerrainKind::Mountains;
        mountains.seed = 7;
        mountains.width = 96;
        mountains.height = 96;
        mountains.size = {14.0f, 3.6f, 14.0f};
        show_mesh(generate_terrain_mesh(mountains), {0, 0, 0}, math::Quat{1, 1, 1, 1});

        Noise2D noise(42);
        constexpr int kN = 64;
        std::vector<float> data(static_cast<size_t>(kN) * kN);
        for (int y = 0; y < kN; ++y)
            for (int x = 0; x < kN; ++x)
            {
                const float u = static_cast<float>(x) / static_cast<float>(kN);
                const float v = static_cast<float>(y) / static_cast<float>(kN);
                data[static_cast<size_t>(y) * kN + x] = noise.fbm(u * 3.0f, v * 3.0f, 4) * 0.5f + 0.5f;
            }
        Heightmap hm;
        hm.heightData = std::move(data);
        hm.width = kN;
        hm.height = kN;
        hm.size = {8.0f, 1.6f, 8.0f};
        show_mesh(generate_heightmap_mesh(hm), {9.5f, 0.0f, -1.0f}, math::Quat{1, 1, 1, 1});
    }
    else if (g.current_tab == 8)   // ── deformation ───────────────────
    {
        MeshData cyl = generate_cylinder_mesh(CylinderGeometry{0.34f, 2.2f, 48, true});
        show_mesh(cyl, {-2.6f, 1.1f, 0}, col(0.80f, 0.60f, 0.40f));

        DeformDescriptor bend;
        bend.bend = true; bend.bendAngle = 1.15f; bend.bendRadius = 1.1f; bend.bendAxis = {0, 0, 1};
        show_mesh(deform_mesh(cyl, bend), {-0.9f, 1.2f, 0}, col(0.45f, 0.75f, 0.90f));

        DeformDescriptor twist;
        twist.twist = true; twist.twistAngle = 2.6f; twist.twistAxis = {0, 1, 0};
        show_mesh(deform_mesh(cyl, twist), {0.9f, 1.1f, 0}, col(0.70f, 0.55f, 0.90f));

        DeformDescriptor taper;
        taper.taper = true; taper.taperStart = 1.0f; taper.taperEnd = 0.3f; taper.taperAxis = {0, 1, 0};
        taper.noise = true; taper.noiseAmplitude = 0.06f; taper.noiseFrequency = 8.0f; taper.noiseSeed = 11;
        show_mesh(deform_mesh(cyl, taper), {2.7f, 1.1f, 0}, col(0.60f, 0.85f, 0.50f));
    }
    else if (g.current_tab == 9)   // ── compressor ────────────────────
    {
        FlowPath fp;
        fp.hub_points    = {{0.0f, 0.34f}, {1.2f, 0.36f}, {2.4f, 0.42f}};
        fp.shroud_points = {{0.0f, 0.62f}, {1.2f, 0.66f}, {2.4f, 0.72f}};

        BladeRow igv; igv.type = BladeRowType::Stator;
        igv.leading_edge_hub = {0.15f, 0.35f};    igv.leading_edge_shroud = {0.15f, 0.63f};
        igv.trailing_edge_hub = {0.45f, 0.35f};   igv.trailing_edge_shroud = {0.45f, 0.63f};
        igv.blade_count.value = 16;

        BladeRow rotor; rotor.type = BladeRowType::Rotor;
        rotor.leading_edge_hub = {0.85f, 0.37f};  rotor.leading_edge_shroud = {0.85f, 0.65f};
        rotor.trailing_edge_hub = {1.25f, 0.38f}; rotor.trailing_edge_shroud = {1.25f, 0.66f};
        rotor.blade_count.value = 12;

        BladeRow stator; stator.type = BladeRowType::Stator;
        stator.leading_edge_hub = {1.55f, 0.39f}; stator.leading_edge_shroud = {1.55f, 0.67f};
        stator.trailing_edge_hub = {1.95f, 0.40f}; stator.trailing_edge_shroud = {1.95f, 0.68f};
        stator.blade_count.value = 14;

        CompressorDefinition cd;
        cd.flow_path = fp;
        cd.has_igv = true;
        cd.igv = igv;
        cd.stages = {{rotor, stator}};
        cd.revolve_segments = 48;
        MeshData comp = generate_compressor_mesh(cd);
        if (!comp.vertices.empty())
            show_mesh(comp, {0, 0, -0.5f}, col(0.85f, 0.70f, 0.50f));
    }
    else if (g.current_tab == 10)   // ── turbine ──────────────────────
    {
        TurbineDefinition td;
        td.flow_path.hub_points    = {{0, 0.35f}, {1, 0.38f}, {2, 0.44f}};
        td.flow_path.shroud_points = {{0, 0.72f}, {1, 0.78f}, {2, 0.86f}};
        td.hub.shape = HubShape::Spinner;
        td.hub.root_radius = 0.35f;
        td.hub.front_length = 0.45f;

        BladeRow st; st.type = BladeRowType::Stator;
        st.leading_edge_hub = {0.30f, 0.40f};   st.leading_edge_shroud = {0.30f, 0.76f};
        st.trailing_edge_hub = {0.75f, 0.42f};  st.trailing_edge_shroud = {0.75f, 0.78f};
        st.blade_count.value = 16;

        BladeRow rt; rt.type = BladeRowType::Rotor;
        rt.leading_edge_hub = {1.15f, 0.45f};   rt.leading_edge_shroud = {1.15f, 0.80f};
        rt.trailing_edge_hub = {1.65f, 0.47f};  rt.trailing_edge_shroud = {1.65f, 0.81f};
        rt.blade_count.value = 18;
        rt.rotational_speed.value = 3000.0f;

        td.blade_rows = {st, rt};
        td.revolve_segments = 48;
        MeshData turb = generate_turbine_mesh(td);
        if (!turb.vertices.empty())
            show_mesh(turb, {0, 0, -0.5f}, col(0.55f, 0.75f, 0.55f));
    }
    else if (g.current_tab == 11)   // ── steam engine ─────────────────
    {
        SteamEngineDefinition def;
        def.crank_angle_deg = static_cast<float>(g.steam_angle);
        const SteamEngineResult res = generate_steam_engine(def);
        g.info = "parts: " + std::to_string(res.assembly.parts.size()) +
                 " | joints: " + std::to_string(res.mechanism.joints.size()) +
                 " | crank: " + std::to_string(g.steam_angle) + " deg | FK rest == recipe rest";
        for (const Part& p : res.assembly.parts)
        {
            math::Quat tint = (p.name.find("flywheel") != std::string::npos ||
                               p.name.find("crank")    != std::string::npos ||
                               p.name.find("shaft")    != std::string::npos)
                                ? col(0.70f, 0.72f, 0.80f) : col(0.85f, 0.60f, 0.32f);
            show_mesh(p.mesh, {0, 0, 0}, tint);
        }
    }
    else if (g.current_tab == 12)   // ── mechanisms & FK ──────────────
    {
        Part gearA = as_part("gear_a", generate_cylinder_mesh(CylinderGeometry{0.55f, 0.22f, 48, true}));
        Part gearB = as_part("gear_b", generate_cylinder_mesh(CylinderGeometry{0.38f, 0.22f, 48, true}));
        std::vector<Part> gears{gearA, gearB};
        Mechanism mech;
        mech.joints.push_back({"j_a", JointKind::Continuous, "", "gear_a",
                               {0, 0, 0}, {0, 1, 0}, -1e30f, 1e30f, 10.0f, 20.0f});
        mech.joints.push_back({"j_b", JointKind::Continuous, "", "gear_b",
                               {0.72f, 0, 0}, {0, 1, 0}, -1e30f, 1e30f, 10.0f, 20.0f});
        mech.couplings.push_back({"g", CouplingKind::Gear, "j_a", "j_b", -0.7f});
        mech.driver_joint = "j_a";

        const auto poses = evaluate_poses(mech, g.mech_state);
        const Assembly a = apply_poses(mech, gears, poses);
        for (const Part& p : a.parts)
            show_mesh(p.mesh, {0, 0, 0},
                      p.name == "gear_a" ? col(0.50f, 0.75f, 0.95f) : col(0.95f, 0.55f, 0.45f));
        g.info = "driving j_a = " + std::to_string(g.mech_state) + " rad -> j_b = " +
                 std::to_string(-0.7f * g.mech_state) + " rad (ratio -0.7)";
    }
    else if (g.current_tab == 13)   // ── import / export ──────────────
    {
        Part casing = generate_box_part(BoxGeometry{{1.8f, 1.0f, 1.0f}, col(0.85f, 0.55f, 0.30f)});
        casing.name = "casing";
        Part rotor = generate_cylinder_part(CylinderGeometry{0.32f, 2.2f, 48, true});
        rotor.name = "rotor";
        rotor.meta.material = "steel-1045";
        CADModel model = make_cad_model("pump", std::vector<Part>{casing, rotor});
        model.materials = MaterialDB::defaults();

        show_mesh(casing.mesh, {-3.4f, 0, 0}, col(0.85f, 0.55f, 0.30f));
        show_mesh(rotor.mesh,  {-3.4f, 0, 0}, col(0.60f, 0.65f, 0.75f));

        const std::string stl   = to_stl_binary(model);
        const std::string msh   = to_msh(model);
        const std::string step  = to_step_faceted(model);
        const CADModel backM   = import_msh(msh);
        const CADModel backS   = import_stl(stl);
        const MeshData stepParse = parse_obj(to_obj(model));

        for (const Part& p : backM.parts)
            show_mesh(p.mesh, {0.6f, 0, 0},
                      p.name.find("casing") != std::string::npos ? col(0.35f, 0.75f, 0.55f)
                                                                 : col(0.60f, 0.55f, 0.85f));
        if (!backS.parts.empty())
            show_mesh(backS.parts[0].mesh, {4.2f, 0, 0}, col(0.90f, 0.70f, 0.40f));
        if (!stepParse.vertices.empty())
            show_mesh(stepParse, {4.2f, 2.2f, 0}, col(0.80f, 0.40f, 0.60f));

        std::vector<std::string> errM, errS;
        const bool okM = backM.validate(errM);
        const bool okS = backS.validate(errS);
        g.info = "stl " + std::to_string(stl.size()) + " B | msh " +
                 std::to_string(msh.size()) + " B | step " +
                 std::to_string(step.size()) + " B | obj " +
                 std::to_string(to_obj(model).size()) + " B\nmsh reimport parts=" +
                 std::to_string(backM.parts.size()) + " validate=" + (okM ? "ok" : "FAIL") +
                 " | stl reimport parts=" + std::to_string(backS.parts.size()) +
                 " validate=" + (okS ? "ok" : "FAIL");
    }
    else if (g.current_tab == 14)   // ── gizmos ───────────────────────
    {
        auto show_gizmo = [&](const GizmoParts& parts, math::Vec3f pos) {
            const MeshData m = merge_gizmo_parts(parts);
            if (!m.vertices.empty())
                show_mesh(m, pos, math::Quat{1, 1, 1, 1});
        };
        show_gizmo(generate_translation_gizmo(TranslationGizmoGeometry{1.2f}), {-2.8f, 0, 0});
        show_gizmo(generate_rotation_gizmo(RotationGizmoGeometry{1.0f}),        {0.0f, 0, 0});
        show_gizmo(generate_scale_gizmo(ScaleGizmoGeometry{1.2f}),             {2.8f, 0, 0});
        show_gizmo(generate_lattice_gizmo(LatticeCageGeometry{{3, 3, 3}, {1.2f, 1.2f, 1.2f}}), {5.6f, 0.6f, 0});
    }
}

// ═══════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════

static void clear_scene(Gallery& g, ecs::Registry& reg, render::GraphicsContext& ctx)
{
    for (uint32_t h : g.meshes)
        if (h != 0)
            ctx.mesh_manager.destroy(h);
    g.meshes.clear();
    for (uint32_t e : g.entities)
        if (reg.valid(ecs::Entity{e}))
            reg.destroy(ecs::Entity{e});
    g.entities.clear();
    g.info.clear();
}

static void frame_camera(Gallery& g, ecs::Registry& reg, const TabDef& t)
{
    auto& ctl = reg.get<render::CameraModeController>(g.cam_entity);
    ctl.mode = render::CameraMode::Orbit;
    ctl.orbit_target = t.target;
    ctl.orbit_distance = t.dist;
    ctl.azimuth = t.azim;
    ctl.elevation = t.elev;
}

static void print_help()
{
    std::printf("\n=== extropian-geometry gallery — controls ===\n");
    std::printf("  Esc     quit\n");
    std::printf("  Tab     toggle UI / FPS input mode\n");
    std::printf("  Orbit:  left-drag orbit · middle-drag pan · scroll zoom\n");
    std::printf("  FPS:    WASD/QE fly, mouse look\n\n");
}

int main()
{
    app::WindowDesc desc;
    desc.title = "extropian-geometry · gallery";
    desc.width = 1280;
    desc.height = 720;
    app::Window window(desc);
    if (!window.is_valid())
    {
        std::fprintf(stderr, "FATAL: window creation failed\n");
        return 1;
    }

    int win_w = 0, win_h = 0;
    float aspect = 1.0f;
    window.get_dimensions(win_w, win_h, aspect);
    window.input_mode = app::InputMode::UI;
    window.set_input_mode(window.input_mode);

    ecs::Registry reg;
    render::GraphicsContext ctx;
    Gallery g;

    // ── systems ────────────────────────────────────────────────────
    render::CameraModeSystem  cam_sys(&window);
    render::GridSystem        grid_sys(ctx, &window);
    render::GradientSkySystem sky_sys(ctx, &window);
    render::RenderSystem      render_sys(ctx, &window);
    render::ImGuiSystem       imgui(ctx, &window);
    imgui.init();
    render_sys.set_clear_color(0.09f, 0.11f, 0.16f, 1.0f);

    // ── camera ▸ only the 3D camera; orbit by default ───────────────
    g.cam_entity = reg.create("Camera");
    reg.emplace<render::Transform>(g.cam_entity, math::Vec3f{0.0f, 4.0f, 14.0f});
    reg.emplace<render::CameraComponent>(g.cam_entity);
    reg.emplace<render::CameraModeController>(g.cam_entity);
    auto& ctl = reg.get<render::CameraModeController>(g.cam_entity);
    ctl.mode = render::CameraMode::Orbit;

    // ── permanent scene ▸ light + grid + procedural sky ─────────────
    auto light = reg.create("Lighting");
    reg.emplace<render::SceneLighting>(light,
        math::Vec3f{0.20f, 0.22f, 0.24f},        // ambient
        math::Vec3f{0.40f, 1.00f, 0.30f},        // sun direction
        math::Vec3f{1.00f, 0.98f, 0.92f});       // sun color
    auto fog = reg.create("Fog");
    reg.emplace<render::FogComponent>(fog, math::Vec3f{0.30f, 0.36f, 0.44f}, 0.0035f);
    auto grid = reg.create("Grid");
    reg.emplace<render::GridComponent>(grid, 1.0f);
    reg.emplace<render::Transform>(grid);
    auto sky = reg.create("SkyGradient");
    auto& skyc = reg.emplace<render::GradientSkyComponent>(sky);
    skyc.zenith        = {0.10f, 0.24f, 0.50f};
    skyc.horizon       = {0.48f, 0.58f, 0.75f};
    skyc.ground        = {0.16f, 0.19f, 0.24f};
    skyc.sun_direction = {0.30f, 0.80f, 0.60f};
    skyc.sun_color     = {1.0f, 0.95f, 0.82f};
    skyc.sun_size      = 0.06f;
    skyc.gradient_power = 2.0f;
    reg.emplace<render::RenderTechnique_Gradient>(sky);

    // ── ImGui host panel with the gallery tab bar ───────────────────
    std::string infoBuf;              // stable storage for the panel's text
    auto panel = reg.create("GalleryPanel");
    reg.emplace<render::ImGuiPanelComponent>(panel,
        "extropian-geometry gallery",
        [&]() {
            ImGui::TextWrapped("%s", kTabs[g.current_tab].info);
            ImGui::Separator();
            ImGui::TextWrapped("%s", infoBuf.c_str());
            ImGui::Separator();

            if (ImGui::BeginTabBar("gallery_tabs"))
            {
                for (int i = 0; i < kTabCount; ++i)
                {
                    bool open = true;
                    if (ImGui::BeginTabItem(kTabs[i].name, &open))   // only when selected
                    {
                        ImGui::EndTabItem();
                        if (g.current_tab != i)
                        {
                            g.current_tab = i;
                            g.build_now = true;
                        }
                    }
                    (void)open;
                }
                ImGui::EndTabBar();
            }

            // animated tab controls
            if (g.current_tab == 5)   // SDF blend
            {
                if (ImGui::SliderFloat("blend radius", &g.blend_radius, 0.0f, 0.5f, "%.2f"))
                    g.build_now = true;
            }
            else if (g.current_tab == 11)   // steam engine
            {
                if (ImGui::SliderInt("crank angle (deg)", &g.steam_angle, 0, 360))
                    g.build_now = true;
            }
            else if (g.current_tab == 12)   // mechanisms
            {
                if (ImGui::SliderFloat("drive angle (rad)", &g.mech_state, 0.0f, 6.2831853f))
                    g.build_now = true;
            }
        },
        true);

    // ── initial tab ─────────────────────────────────────────────────
    g.current_tab = 0;
    g.build_now = true;
    print_help();

    // ═══════════════════════════════════════════════════════════════
    // main loop
    // ═══════════════════════════════════════════════════════════════
    uint64_t last_ticks = SDL_GetTicks();

    while (!window.should_close())
    {
        const uint64_t now = SDL_GetTicks();
        const double dt = (now - last_ticks) / 1000.0;
        last_ticks = now;
        ++g.frame;

        window.poll_events();
        const auto& win_events = window.events();
        for (int i = 0; i < win_events.num_events; ++i)
            imgui.process_event(win_events.events[i]);

        const bool want_kb = imgui.want_capture_keyboard();
        if (window.was_key_released(SDL_SCANCODE_ESCAPE))
            window.close();
        if (window.was_key_released(SDL_SCANCODE_TAB))
        {
            window.input_mode = (window.input_mode == app::InputMode::FPS)
                                ? app::InputMode::UI : app::InputMode::FPS;
            window.set_input_mode(window.input_mode);
        }
        (void)want_kb;

        // keep orbit from fighting the ImGui window
        if (reg.valid(g.cam_entity))
        {
            auto& cur = reg.get<render::CameraModeController>(g.cam_entity);
            cur.lock_movement = imgui.want_capture_mouse();
        }

        // rebuild scene on tab change / slider drag
        if (g.build_now)
        {
            clear_scene(g, reg, ctx);
            build_tab(g, reg, ctx);
            infoBuf = g.info;
            frame_camera(g, reg, kTabs[g.current_tab]);
            g.build_now = false;
        }

        cam_sys.update(reg, dt);
        grid_sys.update(reg, dt);
        sky_sys.update(reg, dt);
        render_sys.update(reg, dt);
        imgui.update(reg, dt);

        window.swap_buffers();
        window.get_dimensions(win_w, win_h, aspect);
    }

    std::printf("[gallery] shutdown — %ld frames rendered\n", g.frame);
    return 0;
}

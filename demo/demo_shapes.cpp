/// extropian-geometry — advanced shapes visual demo
/// Exercises extrusion, lathe, helix, heightmap, deformation, SDF boolean blending.
/// Links to extropian-render for window/GL/camera.  The geometry library does not.

#include <exd/app/window.hpp>
#include <exd/ecs/registry.hpp>
#include <exd/render/graphics/graphics_context.hpp>
#include <exd/render/systems/render_system.hpp>
#include <exd/render/systems/camera_system.hpp>
#include <exd/render/systems/primitive_mesh_system.hpp>
#include <exd/render/systems/grid_system.hpp>
#include <exd/render/systems/polygon_mode_system.hpp>
#include <exd/render/components/transform.hpp>
#include <exd/render/components/camera_component.hpp>
#include <exd/render/components/camera_controller.hpp>
#include <exd/render/components/renderable.hpp>
#include <exd/render/components/render_technique_tags.hpp>
#include <exd/render/components/grid.hpp>
#include <exd/geometry/geometry.hpp>
#include <SDL3/SDL.h>
#include <glad/gl.h>
#include <cstdio>
#include <cmath>

using namespace exd;

// ── Profile generators ─────────────────────────────────────────────────────
namespace {

std::vector<math::Vec3f> star_profile(int points, float outer, float inner) {
    std::vector<math::Vec3f> v(points * 2);
    float step = 3.14159265358979323846f / points;
    for (int i = 0; i < points; ++i) {
        float ao = 1.57079632679f + i * 2 * step;
        float ai = ao + step;
        v[i*2]   = {outer * std::cos(ao), outer * std::sin(ao), 0};
        v[i*2+1] = {inner * std::cos(ai), inner * std::sin(ai), 0};
    }
    return v;
}

std::vector<math::Vec3f> vase_profile() {
    return {
        {0.15f, -1.0f, 0}, {0.30f, -0.5f, 0}, {0.25f,  0.0f, 0},
        {0.40f,  0.4f, 0}, {0.18f,  0.7f, 0}, {0.22f,  0.9f, 0},
        {0.20f,  1.0f, 0},
    };
}

} // anonymous namespace

int main() {
    app::WindowDesc desc;
    desc.title = "extropian-geometry | advanced shapes";
    app::Window window(desc);
    if (!window.is_valid()) {
        std::fprintf(stderr, "FATAL: window creation failed\n");
        return 1;
    }
    std::printf("OpenGL %s | %s\n",
        glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

    int w, h; float aspect;
    window.get_dimensions(w, h, aspect);

    ecs::Registry reg;
    render::GraphicsContext ctx;

    render::CameraSystem       cam_sys(&window);
    render::PrimitiveMeshSystem mesh_sys(ctx, &window);
    render::GridSystem         grid_sys(ctx, &window);
    render::PolygonModeSystem  poly_sys(&window);
    render::RenderSystem       render_sys(ctx, &window);

    // ── Camera ─────────────────────────────────────
    auto cam = reg.create("Camera");
    reg.emplace<render::Transform>(cam, math::Vec3f{0, 6, 20});
    reg.emplace<render::CameraComponent>(cam);
    reg.emplace<render::CameraController>(cam);

    // ── Ground grid ────────────────────────────────
    auto grid = reg.create("Grid");
    reg.emplace<render::GridComponent>(grid, 50.0f);
    reg.emplace<render::Transform>(grid);

    auto add_entity = [&](const char* name, float x, float y, float z, uint32_t mesh) {
        auto e = reg.create(name);
        reg.emplace<render::Transform>(e, math::Vec3f{x, y, z});
        reg.emplace<render::RenderableComponent>(e, mesh);
        reg.emplace<render::RenderTechnique_Lambertian>(e);
    };

    using namespace exd::geometry;

    // Convenience: Quat colors: {R, G, B, A}  (w=R, x=G, y=B, z=A)
    auto rgb = [](float r, float g, float b) {
        return math::Quat{r, g, b, 1.0f};
    };

    // ── 1. Extrusion — star profile extruded (gold) ──
    {
        auto prof = star_profile(5, 0.8f, 0.35f);
        uint32_t mesh = ctx.mesh_manager.create(
            generate_extrusion_mesh({.profile = prof, .depth = 1.2f,
                                      .color = rgb(0.9f, 0.7f, 0.1f)}));
        add_entity("Extrusion_Star", -8, 2.5f, -6, mesh);
    }

    // ── 2. Extrusion — hex nut (steel) ──────────
    {
        auto prof = star_profile(6, 0.7f, 0.55f);
        uint32_t mesh = ctx.mesh_manager.create(
            generate_extrusion_mesh({.profile = prof, .depth = 0.4f,
                                      .color = rgb(0.5f, 0.55f, 0.6f)}));
        add_entity("Extrusion_HexNut", -8, 2.5f, -3, mesh);
    }

    // ── 3. Lathe — vase (terracotta) ────────────
    {
        auto prof = vase_profile();
        uint32_t mesh = ctx.mesh_manager.create(
            generate_lathe_mesh({.profile = prof, .segments = 64,
                                  .color = rgb(0.8f, 0.4f, 0.2f)}));
        add_entity("Lathe_Vase", -4, 2.5f, -6, mesh);
    }

    // ── 4. Lathe — half-vase 180° (copper) ──────
    {
        auto prof = vase_profile();
        for (auto& p : prof) p.x *= 0.7f;
        uint32_t mesh = ctx.mesh_manager.create(
            generate_lathe_mesh({.profile = prof, .segments = 48,
                                  .endAngle = 3.14159265358979323846f,
                                  .color = rgb(0.85f, 0.5f, 0.3f)}));
        add_entity("Lathe_HalfVase", -4, 2.5f, -3, mesh);
    }

    // ── 5. Helix — spring (silver) ──────────────
    {
        auto prof = star_profile(4, 0.15f, 0.15f);
        uint32_t mesh = ctx.mesh_manager.create(
            generate_helix_mesh({.profile = prof, .radius = 0.7f,
                                  .height = 3.5f, .turns = 6.0f, .pathSteps = 128,
                                  .color = rgb(0.7f, 0.7f, 0.75f)}));
        add_entity("Helix_Spring", 0, 2.5f, -6, mesh);
    }

    // ── 6. Helix — DNA coil (cyan) ──────────────
    {
        auto prof = star_profile(6, 0.13f, 0.13f);
        uint32_t mesh = ctx.mesh_manager.create(
            generate_helix_mesh({.profile = prof, .radius = 0.6f,
                                  .height = 3.0f, .turns = 4.0f, .pathSteps = 96,
                                  .color = rgb(0.1f, 0.7f, 0.8f)}));
        add_entity("Helix_DNA", 0, 2.5f, -3, mesh);
    }

    // ── 7. Heightmap — rolling hills (green) ────
    {
        const int W = 64, H = 64;
        std::vector<float> data(W * H);
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x)
                data[y * W + x] = std::sin(x * 0.15f) * std::cos(y * 0.15f) * 0.4f
                                + std::sin(x * 0.3f + 2.0f) * std::cos(y * 0.25f) * 0.2f;
        uint32_t mesh = ctx.mesh_manager.create(
            generate_heightmap_mesh({.heightData = data, .width = W, .height = H,
                                      .size = {5, 1.5f, 5},
                                      .color = rgb(0.2f, 0.6f, 0.15f)}));
        add_entity("Heightmap", 5, 1.0f, -5, mesh);
    }

    // ── 8. Deformation — bent+twisted cylinder (purple) ──
    {
        auto cyl = generate_cylinder_mesh({.radius = 0.2f, .height = 3.0f, .slices = 32,
                                            .color = rgb(0.5f, 0.2f, 0.7f)});
        DeformDescriptor d;
        d.bend = true;  d.bendAngle = 1.2f;  d.bendRadius = 1.5f;
        d.bendAxis = {0, 0, 1};
        d.twist = true; d.twistAngle = 0.6f;
        d.twistAxis = {0, 1, 0};
        uint32_t mesh = ctx.mesh_manager.create(deform_mesh(cyl, d));
        add_entity("Deform_BendTwist", 9, 2.5f, -6, mesh);
    }

    // ── 9. Deformation — tapered+noisy box (lime) ──
    {
        auto box = generate_box_mesh({.size = {1.2f, 2.5f, 1.2f},
                                       .color = rgb(0.3f, 0.8f, 0.1f)});
        DeformDescriptor d;
        d.taper = true; d.taperStart = 1.0f; d.taperEnd = 0.3f;
        d.noise = true; d.noiseAmplitude = 0.15f; d.noiseFrequency = 3.0f; d.noiseSeed = 42;
        uint32_t mesh = ctx.mesh_manager.create(deform_mesh(box, d));
        add_entity("Deform_TaperNoise", 9, 2.5f, -3, mesh);
    }

    // ── 10. SDF Blend — organic blob (coral) ────
    {
        BlendGeometry bg;
        bg.blendRadius = 0.15f;
        bg.cellSize = 0.06f;
        bg.op = BlendOp::Union;
        bg.primitives.push_back({.kind = BlendPrimitiveKind::Sphere,
                                  .position = {0, 0, 0}, .radius = 0.8f});
        bg.primitives.push_back({.kind = BlendPrimitiveKind::Sphere,
                                  .position = {0.6f, 0.4f, 0.3f}, .radius = 0.5f});
        bg.primitives.push_back({.kind = BlendPrimitiveKind::Sphere,
                                  .position = {-0.5f, -0.3f, 0.4f}, .radius = 0.45f});
        bg.primitives.push_back({.kind = BlendPrimitiveKind::Capsule,
                                  .position = {0.2f, -0.6f, -0.2f}, .radius = 0.3f, .height = 0.8f});
        uint32_t mesh = ctx.mesh_manager.create(generate_blend_mesh(bg));
        add_entity("Blend_Blob", -12, 2.0f, -6, mesh);
    }

    // ── 11. SDF Subtract — box with hole ────────
    {
        BlendGeometry bg;
        bg.cellSize = 0.06f;
        bg.op = BlendOp::Subtract;
        bg.primitives.push_back({.kind = BlendPrimitiveKind::Box, .halfExtent = 0.8f});
        bg.primitives.push_back({.kind = BlendPrimitiveKind::Sphere,
                                  .position = {0, 0, 0}, .radius = 0.5f});
        uint32_t mesh = ctx.mesh_manager.create(generate_blend_mesh(bg));
        add_entity("Blend_Subtract", -12, 2.0f, -3, mesh);
    }

    // ── 12. SDF Intersect — torus ∩ sphere ──────
    {
        BlendGeometry bg;
        bg.cellSize = 0.06f;
        bg.op = BlendOp::Intersect;
        bg.primitives.push_back({.kind = BlendPrimitiveKind::Torus,
                                  .radius = 0.35f, .radius2 = 0.7f});
        bg.primitives.push_back({.kind = BlendPrimitiveKind::Sphere, .radius = 0.65f});
        uint32_t mesh = ctx.mesh_manager.create(generate_blend_mesh(bg));
        add_entity("Blend_Intersect", -12, 2.0f, 0, mesh);
    }

    // ═══════════════════════════════════════════════
    // Complex models: Brain & Heart (SDF blending)
    // ═══════════════════════════════════════════════

    // ── Brain ────────────────────────────────────
    {
        BlendGeometry brain;
        brain.blendRadius = 0.08f;
        brain.cellSize = 0.05f;
        brain.op = BlendOp::Union;

        // Two hemispheres
        auto add_hemisphere = [&](float cx, float scale) {
            for (int row = 0; row < 8; ++row) {
                float ny = (row - 3.5f) * 0.3f;
                float cosRow = std::sqrt(1.0f - (ny / 1.5f) * (ny / 1.5f));
                if (cosRow < 0.1f) continue;
                for (int col = 0; col < 6; ++col) {
                    float angle = col * 1.047f + row * 0.2f;
                    float rx = std::cos(angle) * cosRow * 1.3f * scale;
                    float rz = std::sin(angle) * cosRow * 0.9f * scale;
                    brain.primitives.push_back({
                        .kind = BlendPrimitiveKind::Sphere,
                        .position = {cx + rx, ny, rz},
                        .radius = 0.22f + row * 0.01f
                    });
                }
            }
            // Deeper structures (brain stem area)
            for (int i = 0; i < 5; ++i) {
                brain.primitives.push_back({
                    .kind = BlendPrimitiveKind::Sphere,
                    .position = {cx, -1.0f - i * 0.15f, 0},
                    .radius = 0.18f - i * 0.02f
                });
            }
        };

        add_hemisphere(-0.55f, 1.0f);   // left
        add_hemisphere(0.55f, 1.0f);    // right

        // Central connecting ridge
        for (int i = 0; i < 5; ++i) {
            float t = (i - 2.0f) * 0.35f;
            brain.primitives.push_back({
                .kind = BlendPrimitiveKind::Sphere,
                .position = {0, t, 0},
                .radius = 0.28f
            });
        }

        uint32_t mesh = ctx.mesh_manager.create(generate_blend_mesh(brain));
        add_entity("Brain", 14, 3.5f, -5, mesh);
    }

    // ── Heart ────────────────────────────────────
    {
        BlendGeometry heart;
        heart.blendRadius = 0.12f;
        heart.cellSize = 0.04f;
        heart.op = BlendOp::Union;

        // Left ventricle (larger bulge)
        heart.primitives.push_back({
            .kind = BlendPrimitiveKind::Sphere,
            .position = {-0.4f, 0.35f, 0.0f},
            .scale = {1.0f, 1.1f, 0.8f},
            .radius = 0.85f
        });
        // Right ventricle
        heart.primitives.push_back({
            .kind = BlendPrimitiveKind::Sphere,
            .position = {0.4f, 0.35f, 0.0f},
            .scale = {1.0f, 1.1f, 0.8f},
            .radius = 0.85f
        });
        // Atria (upper chambers, smaller)
        heart.primitives.push_back({
            .kind = BlendPrimitiveKind::Sphere,
            .position = {-0.3f, 1.0f, 0.0f},
            .scale = {0.8f, 0.9f, 0.7f},
            .radius = 0.55f
        });
        heart.primitives.push_back({
            .kind = BlendPrimitiveKind::Sphere,
            .position = {0.3f, 1.0f, 0.0f},
            .scale = {0.8f, 0.9f, 0.7f},
            .radius = 0.55f
        });
        // Tapered bottom (apex)
        heart.primitives.push_back({
            .kind = BlendPrimitiveKind::Cone,
            .position = {0, -0.4f, 0},
            .radius = 0.7f,
            .height = 1.2f
        });
        // Aorta (tube coming out top)
        heart.primitives.push_back({
            .kind = BlendPrimitiveKind::Capsule,
            .position = {0.05f, 1.5f, 0.1f},
            .radius = 0.25f,
            .height = 0.8f
        });
        // Pulmonary artery
        heart.primitives.push_back({
            .kind = BlendPrimitiveKind::Capsule,
            .position = {-0.1f, 1.4f, -0.3f},
            .rotation = math::Quat::from_axis_angle({0,0,1}, 0.4f),
            .radius = 0.18f,
            .height = 0.6f
        });
        // Septum ridge (center line)
        heart.primitives.push_back({
            .kind = BlendPrimitiveKind::Capsule,
            .position = {0, 0.3f, 0},
            .radius = 0.12f,
            .height = 1.5f
        });

        uint32_t mesh = ctx.mesh_manager.create(generate_blend_mesh(heart));
        add_entity("Heart", 14, 3.5f, 2, mesh);
    }

    mesh_sys.update(reg, 0.0);

    render_sys.set_clear_color(0.06f, 0.06f, 0.18f, 1.0f); // dark navy

    std::printf("\n=== Advanced Geometry Demo ===\n");
    std::printf("  WASD fly, mouse look, X = wireframe, Esc = quit\n");
    std::printf("  Brain + Heart models use SDF blending\n\n");

    uint64_t last = SDL_GetTicks();
    while (!window.should_close()) {
        uint64_t now = SDL_GetTicks();
        double dt = (now - last) / 1000.0;
        last = now;

        window.poll_events();
        if (window.was_key_released(SDL_SCANCODE_ESCAPE))
            window.close();

        cam_sys.update(reg, dt);
        window.reset_mouse_delta();
        grid_sys.update(reg, dt);
        poly_sys.update(reg, dt);
        mesh_sys.update(reg, dt);

        render_sys.update(reg, dt);

        window.swap_buffers();
        window.get_dimensions(w, h, aspect);
    }

    return 0;
}

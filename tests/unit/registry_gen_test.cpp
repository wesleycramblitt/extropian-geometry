// Regression test: the gallery demo rebuild pattern (clear_scene + rebuild
// every tab switch) must not leak renderable entities across rebuilds.
//
// Bug history: the gallery stored bare entity IDs (uint32_t) and rebuilt
// ecs::Entity{id} with generation 0 when destroying. After the first rebuild,
// recycled IDs carry generation >= 1, so Registry::valid()/destroy() silently
// no-oped and stale renderables kept referencing mesh handles that the scene
// teardown HAD destroyed -> RenderSystem threw "Mesh handle does not exist"
// and the demo aborted on the second tab cycle.
//
// This test models the fix: scene teardown must use the stored Entity
// (id + generation), never a reconstructed {id, 0}.
#include <doctest/doctest.h>
#include <exd/ecs/registry.hpp>
#include <exd/ecs/view.hpp>

#include <cstdint>
#include <vector>

namespace {

// Stand-ins for render::RenderableComponent / render::RenderTechnique_*
// (the test only exercises exd::core, no GL/render dependency).
struct MeshRef { std::uint32_t handle = 0; };
struct Lambertian {};

// Models MeshManager: monotonically increasing handles, destroy() erases.
struct FakeMeshManager {
    std::uint32_t next = 1;
    std::vector<std::uint32_t> live;
    std::uint32_t create() { live.push_back(next); return next++; }
    void destroy(std::uint32_t h) {
        for (auto it = live.begin(); it != live.end(); ++it)
            if (*it == h) { live.erase(it); return; }
    }
};

// The FIXED scene pattern (what demo/gallery.cpp does now).
struct Scene {
    exd::ecs::Registry reg;
    FakeMeshManager meshes;
    std::vector<std::uint32_t> mesh_handles;
    std::vector<exd::ecs::Entity> entities;   // full Entity: id + generation

    // Use after the object is fully constructed (Registry has a deleted copy).
    void rebuild(std::size_t count) {
        for (std::uint32_t h : mesh_handles) meshes.destroy(h);
        mesh_handles.clear();
        for (exd::ecs::Entity e : entities)
            if (reg.valid(e)) reg.destroy(e);
        entities.clear();
        for (std::size_t i = 0; i < count; ++i) {
            const auto e = reg.create("G");
            const std::uint32_t h = meshes.create();
            reg.emplace<MeshRef>(e, h);
            reg.emplace<Lambertian>(e);
            mesh_handles.push_back(h);
            entities.push_back(e);
        }
    }

    std::size_t alive_renderables() const {
        std::size_t n = 0;
        for (auto e : reg.view<MeshRef, Lambertian>()) { (void)e; ++n; }
        return n;
    }
};

} // namespace

TEST_CASE("scene rebuild: full entities are destroyed across many rebuilds")
{
    Scene s;
    for (std::size_t i = 0; i < 6; ++i) {   // six rebuilds, varying sizes
        s.rebuild(1 + i * 3);
        // Every rebuild must leave exactly its own renderables alive.
        CHECK(s.alive_renderables() == 1 + i * 3);
        // Every alive renderable must reference a live mesh handle.
        for (auto e : s.reg.view<MeshRef>()) {
            const auto& mr = s.reg.get<MeshRef>(e);
            bool live = false;
            for (std::uint32_t h : s.meshes.live) if (h == mr.handle) { live = true; break; }
            CHECK(live);
        }
    }
    // The final scene is exactly the last rebuild's renderables.
    CHECK(s.alive_renderables() == 16);
    CHECK(s.meshes.live.size() == 16);
}

TEST_CASE("scene rebuild: buggy id-only handles leak renderables")
{
    // Document the pitfall: storing only the id and reconstructing
    // Entity{id} (gen 0) makes destroy() a silent no-op after recycling.
    exd::ecs::Registry reg;
    FakeMeshManager meshes;
    std::vector<std::uint32_t> mesh_handles;
    std::vector<std::uint32_t> ids_only;   // the buggy pattern

    auto rebuild_id_only = [&](std::size_t count) {
        if (!ids_only.empty()) {
            // NOTE: this mirrors the old demo bug — valid()/destroy() with
            // the reconstructed Entity will no-op for recycled generations.
            for (std::uint32_t id : ids_only)
                if (reg.valid(exd::ecs::Entity{id})) reg.destroy(exd::ecs::Entity{id});
        }
        mesh_handles.clear();
        ids_only.clear();
        for (std::size_t i = 0; i < count; ++i) {
            const auto e = reg.create("G");
            const std::uint32_t h = meshes.create();
            reg.emplace<MeshRef>(e, h);
            reg.emplace<Lambertian>(e);
            mesh_handles.push_back(h);
            ids_only.push_back(e.id);
        }
    };

    rebuild_id_only(3);
    rebuild_id_only(2);          // entity ids get recycled -> gen mismatch
    rebuild_id_only(2);          // clear now fails to destroy the 2nd set
    std::size_t leaked = 0;
    for (auto e : reg.view<MeshRef, Lambertian>()) { (void)e; ++leaked; }
    // After the third rebuild the scene should hold 2 renderables; with the
    // buggy pattern the second rebuild's entities leak (2 extra).
    CHECK(leaked > 2);
    CHECK(leaked == 4);
}
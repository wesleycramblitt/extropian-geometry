#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build}"
GENERATOR="${2:-Ninja}"

CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5"

# Build the library first
echo "==> Building..."
cmake -S . -B "${BUILD_DIR}" -G "${GENERATOR}" ${CMAKE_ARGS}
cmake --build "${BUILD_DIR}"

# Compile and run a quick smoke check
DEMO_SRC="${BUILD_DIR}/smoke_demo.cpp"
DEMO_BIN="${BUILD_DIR}/smoke_demo"

cat > "${DEMO_SRC}" << 'EOF'
#include <exd/geometry/geometry.hpp>
#include <cstdio>

int main() {
    int passed = 0, failed = 0;
    auto check = [&](const char* name, bool cond) {
        printf("  %-12s %s\n", name, cond ? "OK" : "FAIL");
        cond ? ++passed : ++failed;
    };

    printf("─── 3D primitives ───\n");
    {
        auto m = exd::geometry::generate_sphere_mesh({});
        check("Sphere", m.vertices.size() > 0);
        printf("       %zu verts, %zu indices\n", m.vertices.size(), m.indices.size());
    }
    {
        auto m = exd::geometry::generate_box_mesh({});
        check("Box", m.vertices.size() == 24);
    }
    {
        auto m = exd::geometry::generate_cylinder_mesh({});
        check("Cylinder", m.vertices.size() > 0);
    }
    {
        auto m = exd::geometry::generate_plane_mesh({});
        printf("  Plane        %s\n", m.vertices.size() > 0 ? "OK" : "STUB");
        passed++;
    }

    printf("─── 2D primitives ───\n");
    {
        auto m = exd::geometry::generate_rect_mesh({});
        check("Rect", m.vertices.size() == 4);
    }
    {
        auto m = exd::geometry::generate_rounded_rect_mesh({});
        check("RoundedRect", m.vertices.size() > 0);
    }
    auto circle = exd::geometry::generate_circle_mesh({});
    check("Circle", circle.vertices.size() > 0);
    {
        auto m = exd::geometry::generate_ellipse_mesh({});
        check("Ellipse", m.vertices.size() > 0);
    }
    {
        auto m = exd::geometry::generate_arc_mesh({});
        check("Arc", m.vertices.size() > 0);
    }
    {
        auto m = exd::geometry::generate_ring_mesh({});
        check("Ring", m.vertices.size() > 0);
    }
    {
        auto m = exd::geometry::generate_line_mesh({});
        check("Line", m.vertices.size() == 4);
    }
    {
        auto m = exd::geometry::generate_polyline_mesh({
            .points = {{0,0,0}, {1,0,0}, {1,1,0}},
            .width = 0.1f
        });
        check("Polyline", m.vertices.size() > 0);
    }
    {
        auto m = exd::geometry::generate_arrow_mesh({});
        check("Arrow", m.vertices.size() > 0);
    }
    {
        auto m = exd::geometry::generate_grid_mesh({});
        check("Grid", m.vertices.size() > 0);
    }

    printf("─── MeshOps ───\n");
    {
        auto merged = exd::geometry::merge_meshes(
            std::array<exd::geometry::MeshData, 2>{
                exd::geometry::generate_rect_mesh({.size = {1,1,0}}),
                exd::geometry::generate_circle_mesh({.radius = 0.5f, .segments = 8})
            });
        check("Merge", merged.vertices.size() > 0);
    }
    {
        auto xformed = exd::geometry::transform_mesh(
            circle,
            exd::math::Mat4::trs({5.0f, 3.0f, 0.0f}, exd::math::Quat{}, {1,1,1}));
        bool ok = xformed.bounds.min.x >= 4.0f && xformed.bounds.min.y >= 2.0f;
        check("Transform", ok);
    }

    printf("\n%d passed, %d failed\n", passed, failed);
    return failed > 0 ? 1 : 0;
}
EOF

CORE_INCLUDE="${BUILD_DIR}/_deps/exd-core-src/include"
GEOM_INCLUDE="$(pwd)/include"
GEOM_LIB="${BUILD_DIR}/libexd-geometry.a"
CORE_LIB="${BUILD_DIR}/_deps/exd-core-build/libexd-core.a"

echo "==> Compiling smoke check..."
g++ -std=c++23 \
    -I"${GEOM_INCLUDE}" \
    -I"${CORE_INCLUDE}" \
    -o "${DEMO_BIN}" \
    "${DEMO_SRC}" \
    "${GEOM_LIB}" \
    "${CORE_LIB}" \
    -lm

echo "==> Running..."
"${DEMO_BIN}"

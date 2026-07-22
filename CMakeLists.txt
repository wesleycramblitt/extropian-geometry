cmake_minimum_required(VERSION 3.21)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(extropian-geometry VERSION 0.1.0 LANGUAGES CXX
    DESCRIPTION "Extropian Geometry: renderer-independent mesh generation and geometric computation"
)

# ── Dependencies ──
include(FetchContent)

if(NOT TARGET exd::core)
    FetchContent_Declare(exd-core
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-core.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-core)
endif()

# ── Text support (FreeType + HarfBuzz) ──
option(ENABLE_TEXT "Enable text shaping and font rendering support" ON)

if(ENABLE_TEXT)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FREETYPE REQUIRED freetype2)
    pkg_check_modules(HARFBUZZ REQUIRED harfbuzz)
endif()

# ── Library target (STATIC) ──
add_library(exd-geometry STATIC
    src/mesh_builder.cpp
    src/mesh_ops.cpp
    src/primitives3d/sphere.cpp
    src/primitives3d/box.cpp
    src/primitives3d/cylinder.cpp
    src/primitives3d/plane.cpp
    src/primitives3d/capsule.cpp
    src/primitives3d/icosahedron.cpp
    src/primitives3d/torus.cpp
    src/primitives3d/cone.cpp
    src/primitives3d/tube.cpp
    src/primitives3d/disk.cpp
    src/primitives3d/arrow3d.cpp
    src/primitives3d/axes.cpp
    src/primitives3d/billboard.cpp
    src/primitives2d/rounded_rect.cpp
    src/primitives2d/circle.cpp
    src/primitives2d/line.cpp
    src/primitives2d/rect.cpp
    src/primitives2d/ellipse.cpp
    src/primitives2d/arc.cpp
    src/primitives2d/ring.cpp
    src/primitives2d/polyline.cpp
    src/primitives2d/arrow.cpp
    src/primitives2d/grid.cpp
    src/path/path.cpp
    src/text/text.cpp
    src/text/font.cpp
    src/text/glyph_mesh.cpp
    src/blend/blend.cpp
    src/blend/sdf_primitives.cpp
    src/blend/marching_cubes.cpp
)

add_library(exd::geometry ALIAS exd-geometry)

# ── Include paths ──
target_include_directories(exd-geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ── Link dependencies ──
target_link_libraries(exd-geometry PUBLIC
    exd::core
)

# ── Text support ──
if(ENABLE_TEXT)
    target_compile_definitions(exd-geometry PUBLIC EXD_GEOMETRY_HAS_TEXT=1)
    target_include_directories(exd-geometry PRIVATE ${FREETYPE_INCLUDE_DIRS} ${HARFBUZZ_INCLUDE_DIRS})
    target_link_libraries(exd-geometry PUBLIC ${FREETYPE_LIBRARIES} ${HARFBUZZ_LIBRARIES})
endif()

# ── Tests ──
option(BUILD_TESTS "Build Geometry tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

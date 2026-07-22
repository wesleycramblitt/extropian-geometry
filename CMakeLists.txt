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

# ── Library target (STATIC) ──
add_library(exd-geometry STATIC
    src/mesh_builder.cpp
    src/primitives3d/sphere.cpp
    src/primitives3d/box.cpp
    src/primitives3d/cylinder.cpp
    src/primitives3d/plane.cpp
    src/primitives3d/capsule.cpp
    src/primitives3d/icosahedron.cpp
    src/primitives2d/primitives2d.cpp
    src/path/path.cpp
    src/text/text.cpp
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

# ── Tests ──
option(BUILD_TESTS "Build Geometry tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

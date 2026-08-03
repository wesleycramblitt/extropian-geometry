# ── Advanced geometry visual demo ──────────────────────────────────────────
# This executable links to extropian-render for windowing/GL/camera.
# The geometry library itself does NOT depend on render.
# Gate behind an option so CI / library-only builds skip it.

option(BUILD_DEMO "Build the advanced geometry visual demo" OFF)

if(NOT BUILD_DEMO)
    return()
endif()

# ── Fetch extropian-render ──────────────────────
if(NOT TARGET exd::render)
    include(FetchContent)
    FetchContent_Declare(exd-render
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-render.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-render)
endif()

# ── Copy shaders to build dir (needed at runtime) ──
file(COPY "${exd-render_SOURCE_DIR}/shaders" DESTINATION "${CMAKE_BINARY_DIR}")

# ── Demo executable ─────────────────────────────
add_executable(extropian-geometry-shapes demo_shapes.cpp)
target_link_libraries(extropian-geometry-shapes PRIVATE exd::geometry exd::render)
target_compile_features(extropian-geometry-shapes PRIVATE cxx_std_23)

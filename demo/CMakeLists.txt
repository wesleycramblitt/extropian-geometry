# ── extropian-geometry gallery demo (3D viewer via extropian-render) ──
#
#   ./demo.sh            # builds + runs
#
# Overrides for local development (like EXD_CORE_DIR):
#   -DEXD_RENDER_DIR=../extropian-render
#   -DEXD_APP_DIR=../extropian-app
#
# extropian-render is a leaf consumer of exd::geometry; the renderer owns the
# window/app/GL/ImGui stack, this demo owns the content.

include(FetchContent)

# extropian-app is fetched BY extropian-render, but we pre-fetch it here so a
# local EXD_APP_DIR override works (render's `if(NOT TARGET exd::app)` skips).
if(NOT TARGET exd::app)
    if(DEFINED EXD_APP_DIR)
        FetchContent_Declare(exd-app SOURCE_DIR "${EXD_APP_DIR}")
    else()
        FetchContent_Declare(exd-app
            GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-app.git
            GIT_TAG main
        )
    endif()
    FetchContent_MakeAvailable(exd-app)
endif()

if(NOT TARGET exd::render)
    if(DEFINED EXD_RENDER_DIR)
        FetchContent_Declare(exd-render SOURCE_DIR "${EXD_RENDER_DIR}")
    else()
        FetchContent_Declare(exd-render
            GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-render.git
            GIT_TAG main
        )
    endif()
    # We only want the render library, not its own demo/tests.
    set(EXD_RENDER_BUILD_TESTS OFF CACHE BOOL "Build render tests" FORCE)
    set(EXD_RENDER_BUILD_DEMO  OFF CACHE BOOL "Build render demo" FORCE)
    FetchContent_MakeAvailable(exd-render)
endif()

add_executable(extropian-geometry-gallery gallery.cpp)
target_link_libraries(extropian-geometry-gallery PRIVATE
    exd::render exd::app exd::geometry)
target_compile_features(extropian-geometry-gallery PRIVATE cxx_std_23)

# Shaders are needed at runtime (relative to the working directory).
file(COPY ${exd-render_SOURCE_DIR}/shaders DESTINATION ${CMAKE_BINARY_DIR})

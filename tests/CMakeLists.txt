# ── Unit tests ──
add_executable(test_2d_primitives  unit/primitives2d_test.cpp)
add_executable(test_3d_primitives  unit/primitives3d_test.cpp)
add_executable(test_mesh_ops       unit/mesh_ops_test.cpp)
add_executable(test_mesh_builder   unit/mesh_builder_test.cpp)
add_executable(test_path           unit/path_test.cpp)
add_executable(test_blend           unit/blend_test.cpp)
add_executable(test_advanced        unit/advanced_test.cpp)
add_executable(test_parts           unit/part_test.cpp)
add_executable(test_gizmos          unit/gizmo_test.cpp)
add_executable(test_loft            unit/loft_test.cpp)
add_executable(test_compressor      unit/compressor_test.cpp)
add_executable(test_mechanism        unit/mechanism_test.cpp)
add_executable(test_export           unit/export_test.cpp)
add_executable(test_cad_model         unit/cad_model_test.cpp)
add_executable(test_cae_export         unit/export_cae_test.cpp)
add_executable(test_steam_engine    unit/steam_engine_test.cpp)

set(TEST_TARGETS
    test_2d_primitives
    test_3d_primitives
    test_mesh_ops
    test_mesh_builder
    test_path
    test_blend
    test_advanced
    test_parts
    test_gizmos
    test_loft
    test_compressor
    test_mechanism
    test_export
    test_steam_engine
    test_cad_model
    test_cae_export
)

foreach(target ${TEST_TARGETS})
    target_link_libraries(${target} PRIVATE exd-geometry exd::core doctest::doctest_with_main)
    target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
    add_test(NAME ${target} COMMAND ${target})
endforeach()

add_executable(test_spline  unit/spline_test.cpp)
target_link_libraries(test_spline PRIVATE exd-geometry exd::core doctest::doctest_with_main)
add_test(NAME test_spline COMMAND test_spline)

add_executable(test_turbine unit/turbine_test.cpp)
target_link_libraries(test_turbine PRIVATE exd-geometry exd::core doctest::doctest_with_main)
add_test(NAME test_turbine COMMAND test_turbine)

add_executable(test_noise unit/noise_test.cpp)
target_link_libraries(test_noise PRIVATE exd-geometry exd::core doctest::doctest_with_main)
add_test(NAME test_noise COMMAND test_noise)

add_executable(test_terrain unit/terrain_test.cpp)
target_link_libraries(test_terrain PRIVATE exd-geometry exd::core doctest::doctest_with_main)
add_test(NAME test_terrain COMMAND test_terrain)

# ── Text tests ──
if(ENABLE_TEXT)
    add_executable(test_text unit/text_test.cpp)
    target_link_libraries(test_text PRIVATE exd-geometry exd::core doctest::doctest_with_main)
    target_include_directories(test_text PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${FREETYPE_INCLUDE_DIRS}
        ${HARFBUZZ_INCLUDE_DIRS}
    )
    target_link_libraries(test_text PRIVATE ${FREETYPE_LIBRARIES} ${HARFBUZZ_LIBRARIES})
    add_test(NAME test_text COMMAND test_text)

    add_executable(test_sdf_atlas unit/sdf_atlas_test.cpp)
    target_link_libraries(test_sdf_atlas PRIVATE exd-geometry exd::core doctest::doctest_with_main)
    target_include_directories(test_sdf_atlas PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${FREETYPE_INCLUDE_DIRS}
        ${HARFBUZZ_INCLUDE_DIRS}
    )
    target_link_libraries(test_sdf_atlas PRIVATE ${FREETYPE_LIBRARIES} ${HARFBUZZ_LIBRARIES})
    add_test(NAME test_sdf_atlas COMMAND test_sdf_atlas)
endif()

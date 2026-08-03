# ── Unit tests ──
add_executable(test_2d_primitives  unit/primitives2d_test.cpp)
add_executable(test_3d_primitives  unit/primitives3d_test.cpp)
add_executable(test_mesh_ops       unit/mesh_ops_test.cpp)
add_executable(test_mesh_builder   unit/mesh_builder_test.cpp)
add_executable(test_path           unit/path_test.cpp)
add_executable(test_blend           unit/blend_test.cpp)
add_executable(test_advanced        unit/advanced_test.cpp)

set(TEST_TARGETS
    test_2d_primitives
    test_3d_primitives
    test_mesh_ops
    test_mesh_builder
    test_path
    test_blend
    test_advanced
)

foreach(target ${TEST_TARGETS})
    target_link_libraries(${target} PRIVATE exd-geometry exd::core doctest::doctest_with_main)
    target_include_directories(${target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
    add_test(NAME ${target} COMMAND ${target})
endforeach()

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
endif()

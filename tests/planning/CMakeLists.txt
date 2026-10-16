find_package(nlohmann_json CONFIG REQUIRED)

add_executable(grid_map_test grid_map_test.cc)
target_link_libraries(grid_map_test
    PRIVATE implementations::planning_base nlohmann_json::nlohmann_json
)
add_test(
    NAME planning.grid_map_json
    COMMAND grid_map_test "${PROJECT_SOURCE_DIR}/../data"
        "${CMAKE_CURRENT_BINARY_DIR}/grid_map_test_input.json"
)

add_executable(later_boundary_test
    later_boundary_test.cc
    ${PROJECT_SOURCE_DIR}/experiments/path_orientation/later_boundary_query.cc
)
target_include_directories(later_boundary_test PRIVATE
    ${PROJECT_SOURCE_DIR}/experiments/path_orientation
)
target_link_libraries(later_boundary_test PRIVATE implementations::planning_base)
add_test(NAME planning.lateral_boundary COMMAND later_boundary_test)

add_executable(local_escape_planner_test
    local_escape_planner_test.cc
    ${PROJECT_SOURCE_DIR}/experiments/path_orientation/later_boundary_query.cc
    ${PROJECT_SOURCE_DIR}/experiments/path_orientation/local_escape_planner.cc
)
target_include_directories(local_escape_planner_test PRIVATE
    ${PROJECT_SOURCE_DIR}/experiments/path_orientation
)
target_link_libraries(local_escape_planner_test PRIVATE implementations::planning_base)
add_test(NAME planning.local_escape COMMAND local_escape_planner_test)

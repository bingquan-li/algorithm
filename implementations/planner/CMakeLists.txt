add_library(planner
    SHARED
        local_escape.cc
        debug_visualization.cc
)
target_include_directories(planner
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(planner
    PUBLIC
        algorithms::eigen
        implementations::planning_base
        implementations::lateral_boundary
    PRIVATE
        algorithms::visualization
)

add_library(implementations::planner ALIAS planner)

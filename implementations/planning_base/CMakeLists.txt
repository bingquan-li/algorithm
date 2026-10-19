add_library(planning_base
    SHARED
        grid_map.cc
        footprint.cc
)
target_include_directories(planning_base
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(planning_base
    PUBLIC
        algorithms::eigen
    PRIVATE
        algorithms::json
)

add_library(implementations::planning_base ALIAS planning_base)

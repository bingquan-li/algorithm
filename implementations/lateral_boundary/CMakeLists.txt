add_library(lateral_boundary SHARED
    lateral_boundary.cc
)
target_include_directories(lateral_boundary
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(lateral_boundary
    PUBLIC
        implementations::planning_base
)
add_library(implementations::lateral_boundary ALIAS lateral_boundary)
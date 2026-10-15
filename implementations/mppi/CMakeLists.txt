add_library(mppi SHARED
    mppi.cc
)
target_include_directories(mppi
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(mppi
    PUBLIC
        implementations::planning_base
)
add_library(implementations::mppi ALIAS mppi)
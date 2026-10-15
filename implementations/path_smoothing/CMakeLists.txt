add_library(path_smoothing SHARED
    path_smoothing.cc
    backend/osqp.cc
    backend/projected_gradient.cc
)
target_include_directories(path_smoothing
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(path_smoothing
    PUBLIC
        algorithms::eigen
)
add_library(implementations::path_smoothing ALIAS path_smoothing)
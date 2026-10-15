add_executable(pg_smooth
    pg_smooth.cc
)
target_link_libraries(pg_smooth
    PRIVATE
        algorithms::common
        implementations::path_smoothing
)
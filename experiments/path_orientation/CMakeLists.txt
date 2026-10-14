add_executable(path_orientation
    main.cc
)

target_link_libraries(path_orientation
    PRIVATE
        straight_prefix
)
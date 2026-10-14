add_executable(ransac_line_fitting
    main.cc
)

target_link_libraries(ransac_line_fitting
    PRIVATE
        straight_prefix
)
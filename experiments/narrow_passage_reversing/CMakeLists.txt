add_executable(narrow_passage_reversing
    main.cc
)

target_link_libraries(narrow_passage_reversing
    PRIVATE
        implementations::straight_prefix
        algorithms::common
        algorithms::json
        algorithms::eigen
        implementations::planning_base
        implementations::lateral_boundary
)

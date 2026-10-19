add_executable(narrow_passage_reversing
    main.cc
    scenario.cc
    footprint_io.cc
    visualization.cc
)

target_compile_definitions(narrow_passage_reversing PRIVATE
    NARROW_PASSAGE_DEFAULT_FOOTPRINT="${PROJECT_SOURCE_DIR}/../data/footprint.json"
    NARROW_PASSAGE_DEFAULT_INPUT="${PROJECT_SOURCE_DIR}/../data/narrow_passage_reverseing2/frame.json"
)

target_link_libraries(narrow_passage_reversing PRIVATE
    implementations::planning_base
    implementations::planner
    algorithms::rerun
    algorithms::json
)

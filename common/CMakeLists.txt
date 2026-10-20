add_library(common 
    SHARED
        file.cc
)
target_include_directories(common
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(common
    PUBLIC
        algorithms::eigen
)

add_library(algorithms::common ALIAS common)

add_subdirectory(rerun)

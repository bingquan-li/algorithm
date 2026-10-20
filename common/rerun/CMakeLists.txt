add_library(visualization SHARED recording.cc debug.cc)
target_include_directories(visualization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(visualization PUBLIC algorithms::rerun algorithms::eigen)
add_library(algorithms::visualization ALIAS visualization)

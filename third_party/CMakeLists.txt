add_subdirectory(rerun)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(algorithms_eigen INTERFACE)
target_link_libraries(algorithms_eigen
    INTERFACE
        Eigen3::Eigen
)
add_library(algorithms::eigen ALIAS algorithms_eigen)
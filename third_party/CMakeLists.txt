add_subdirectory(rerun)
add_subdirectory(mujoco)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_library(algorithms_eigen INTERFACE)
target_link_libraries(algorithms_eigen
    INTERFACE
        Eigen3::Eigen
)
add_library(algorithms::eigen ALIAS algorithms_eigen)

find_package(nlohmann_json CONFIG REQUIRED)
add_library(algorithms_json INTERFACE)
target_link_libraries(algorithms_json
    INTERFACE
        nlohmann_json::nlohmann_json
)
add_library(algorithms::json ALIAS algorithms_json)
set(MUJOCO_ROOT "/opt/mujoco" CACHE PATH "MuJoCo installation directory")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    HINTS "${MUJOCO_ROOT}/include" REQUIRED)
find_library(MUJOCO_LIBRARY NAMES mujoco
    HINTS "${MUJOCO_ROOT}/lib" REQUIRED)

add_library(mujoco_sdk UNKNOWN IMPORTED)
set_target_properties(mujoco_sdk PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)
add_library(mujoco_dependency INTERFACE)
target_link_libraries(mujoco_dependency INTERFACE mujoco_sdk)
add_library(algorithms::mujoco ALIAS mujoco_dependency)

find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
add_library(mujoco_viewer_dependency INTERFACE)
target_link_libraries(mujoco_viewer_dependency
    INTERFACE
        algorithms::mujoco
        glfw
        OpenGL::GL
)
add_library(algorithms::mujoco_viewer ALIAS mujoco_viewer_dependency)

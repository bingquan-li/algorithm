add_executable(mujoco_pendulum
    main.cc
    viewer.cc
)

target_link_libraries(mujoco_pendulum
    PRIVATE
        algorithms::mujoco_viewer
)

configure_file(pendulum.xml pendulum.xml COPYONLY)
target_compile_definitions(mujoco_pendulum PRIVATE
    MUJOCO_PENDULUM_MODEL="${CMAKE_CURRENT_BINARY_DIR}/pendulum.xml"
)

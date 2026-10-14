# Compiled
# cmake --install build --config Release
# cmake --build build --config Release --target rerun_sdk -j16
# cmake -S . -B build \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DCMAKE_INSTALL_PREFIX=/opt/sdk/rerun \
#     -DRERUN_DOWNLOAD_AND_BUILD_ARROW=ON \
#     -DRERUN_ARROW_LINK_SHARED=OFF
#     -DCMAKE_POSITION_INDEPENDENT_CODE=ON

# Respect rerun_sdk_DIR and CMAKE_PREFIX_PATH before the local fallback.
find_package(
    rerun_sdk
    CONFIG
    REQUIRED
    PATHS /opt/sdk/rerun
)

# 创建项目自己的依赖包装 target
add_library(coverage_rerun INTERFACE)

# 在能访问 rerun_sdk 的当前目录中建立依赖关系
target_link_libraries(coverage_rerun
    INTERFACE
        rerun_sdk
)

# 提供统一的命名空间
add_library(
    coverage_path_planning::rerun
    ALIAS coverage_rerun
)
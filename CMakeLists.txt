cmake_minimum_required(VERSION 3.28)
project(Algorithm VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 整个项目统一 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(third_party)
add_subdirectory(common)
add_subdirectory(implementations)


add_subdirectory(experiments)
add_subdirectory(exercise)

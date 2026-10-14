add_library(straight_prefix SHARED
    straight_prefix.cc
)

target_include_directories(straight_prefix
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(implementations::straight_prefix ALIAS straight_prefix)
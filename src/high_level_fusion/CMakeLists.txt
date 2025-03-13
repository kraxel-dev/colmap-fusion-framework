cmake_minimum_required(VERSION 3.16)

# define this high level fusion subdirectory as library
set(HIGH_LVL_FUSION_SOURCES
    fusion_graph_interface.cpp
)
add_library(high_level_fusion SHARED ${HIGH_LVL_FUSION_SOURCES})
target_include_directories(high_level_fusion PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(high_level_fusion PRIVATE ${CMAKE_PROJECT_NAME};fusion_helper) # add dependency found in toplevel cmake

# add executable for high level fusion
add_executable(metric_odom_bundle_adjust metric_odom_bundle_adjust.cpp)
target_link_libraries(metric_odom_bundle_adjust ${CMAKE_PROJECT_NAME};fusion_helper;high_level_fusion)

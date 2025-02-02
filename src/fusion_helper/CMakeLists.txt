cmake_minimum_required(VERSION 3.16)

set(FUSION_HELPER_SOURCES
    io.cpp
    col_utils.cpp
)

add_library(fusion_helper SHARED ${FUSION_HELPER_SOURCES})
target_include_directories(fusion_helper PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(fusion_helper PRIVATE ${CMAKE_PROJECT_NAME})

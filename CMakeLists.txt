cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_MAKEFILE ON)

project(colmap_fusion_framework)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include cmake to build third-party dependencies
include(cmake/thirdparty_colmap3.11.cmake)
ExternalProject_Get_Property(colmap_3.11.1 SOURCE_DIR BINARY_DIR INSTALL_DIR)

message(STATUS "Locaition for cloning original colmap 3.11.1 as 3rd party package: ${SOURCE_DIR}")
message(STATUS "Build space will be: ${BINARY_DIR}")
message(STATUS "Local install space will be: ${INSTALL_DIR}")

# Notify cmake where to find locally installed colmap
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${INSTALL_DIR})

# locally installed colmap from cmake file of this repo
find_package(colmap 3.11.1 QUIET)
if(NOT colmap_FOUND)
    message(WARNING "First run: Original colmap not yet built and installed locally as thirdparty module. CMake will stop here but has gathered info to fetch and install colmap. Run `make` to install colmap, then re-run this CMake again to build the actual repo.")
    return() # gracefully exit the CMake script
endif()

find_package(PkgConfig REQUIRED)
# apt installed Google Logger
pkg_check_modules(glog REQUIRED libglog)
# apt installed libceres-dev 
find_package(Ceres REQUIRED)
# apt installed eigen3
find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(colmap::colmap)

set(SOURCES
    src/fusion_helper/io.cpp
)

# Define our repo as custom library
add_library(${PROJECT_NAME} SHARED ${SOURCES})
# link external colmap to our custom library
target_link_libraries(${PROJECT_NAME} PUBLIC colmap::colmap)

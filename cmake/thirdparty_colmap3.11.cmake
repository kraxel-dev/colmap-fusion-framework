# --------------------------------------------------------
# Cmake submodule to clone and locally install colmap 3.11.1 into the thirdparty folder. 
# Makes colmap available as package for the main project.
# --------------------------------------------------------

message(STATUS "Configuring external_project_add for Colmap!")
include(ExternalProject)

# set folder to place 3rd party libs as general path prefix
set(PREFIX_3RD_PARTY ${CMAKE_SOURCE_DIR}/thirdparty)
set(PREFIX_3RD_PARTY_MISC ${PREFIX_3RD_PARTY}/misc)

# set install space for external libraries to be in locally in this repo
set(INSTALL_PREFIX ${PREFIX_3RD_PARTY}/install)

# define name and version for this 3rd party project
set(COLMAP_PROJECT_NAME colmap)
set(COLMAP_VERSION 3.11.1)
set(PROJECT_NAME_AND_VERSION ${COLMAP_PROJECT_NAME}_${COLMAP_VERSION})

# workaround to check for static vs shared library setting
message(STATUS "Build shared libs is: " ${BUILD_SHARED_LIBS})

# Fetch and install colmap 3.11.1 locally
ExternalProject_Add(
    ${PROJECT_NAME_AND_VERSION}
    GIT_REPOSITORY https://github.com/colmap/colmap.git
    GIT_TAG ${COLMAP_VERSION} # Fix to release version 3.11.1

    SOURCE_DIR ${PREFIX_3RD_PARTY}/${PROJECT_NAME_AND_VERSION} # where repo will be cloned to
    BINARY_DIR ${PREFIX_3RD_PARTY}/${PROJECT_NAME_AND_VERSION}_build # build files for 3rd party colmap
    INSTALL_DIR ${INSTALL_PREFIX} # local install space
    LOG_CONFIG ON
    CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX};-DCMAKE_EXPORT_COMPILE_COMMANDS=ON;-DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD};-DBUILD_SHARED_LIBS=${BUILD_SHARED_LIBS}" # pass on build options
    BUILD_ALWAYS OFF # prevent rebuilding unless necessary
    UPDATE_COMMAND "" # also prevents rebuilding unless necessary

    DOWNLOAD_DIR ${PREFIX_3RD_PARTY_MISC}/download # not really used in git clone
    STAMP_DIR ${PREFIX_3RD_PARTY_MISC}/stamps
    LOG_DIR ${PREFIX_3RD_PARTY_MISC}/logs
    TMP_DIR ${PREFIX_3RD_PARTY_MISC}/tmp
)

message(STATUS "Done Configuring external_project_add for Colmap!")

# --------------------------------------------------------
# Cmake submodule to fetch and locally install the rerun SDK locally into this repo. 
# Makes rerun SDK available as package for the main project.
# --------------------------------------------------------

include(FetchContent)

if(BUILD_SHARED_LIBS)
    set(RERUN_ARROW_LINK_SHARED ON)
endif()

message(STATUS "Fetching rerun sdk for C++ as third party module!")
# https://github.com/rerun-io/rerun/releases/tag/0.22.0
FetchContent_Declare(
    rerun_sdk URL https://github.com/rerun-io/rerun/releases/download/0.22.0/rerun_cpp_sdk.zip
)

FetchContent_MakeAvailable(rerun_sdk)
message(STATUS "All done configuring rerun_sdk. Will be fetched and installed once you run `make`.")
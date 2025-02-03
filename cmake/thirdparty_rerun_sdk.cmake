include(FetchContent)
message(STATUS "Fetching rerun sdk for C++ as third party module!")
# https://github.com/rerun-io/rerun/releases/tag/0.21.0
FetchContent_Declare(
    rerun_sdk URL https://github.com/rerun-io/rerun/releases/download/0.21.0/rerun_cpp_sdk.zip
)

FetchContent_MakeAvailable(rerun_sdk)
message(STATUS "All done configuring rerun_sdk. Will be fetched and installed once you run `make`.")
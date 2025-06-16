cmake_minimum_required(VERSION 3.16)

# define this fusion helper subdirectory as library
set(FUSION_HELPER_SOURCES
    io.cpp
    col_utils.cpp
    cov_utils.cpp
    ceres_eval_utils.cpp
    fusion_residuals_tracker.cpp
    fusion_evaluation_callback.cpp
    frame_align_utils.cpp
    rr_utils.cpp
    rr_sfm_logger.cpp
    odom_edges_manager.cpp
)
add_library(fusion_helper ${FUSION_HELPER_SOURCES})  # automatically exposed for other subdirs to use
target_include_directories(fusion_helper PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(fusion_helper PUBLIC  ${PROJECT_DEPENDENCIES}) # add dependency found in toplevel cmake

add_executable(sparsfiy_model colmap_model_manipulation/sparsify_model.cpp)
target_link_libraries(sparsfiy_model fusion_helper)
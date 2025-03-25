cmake_minimum_required(VERSION 3.16)

# define this tightly_coupled_fusion subdirectory as library
set(TIGHTLY_COUPLED_FUSION_SOURCES
    estimators/bundle_adjustment_original.cc
    estimators/fusion_bundle_adjustment.cc
    sfm/incremental_mapper.cc
    controllers/incremental_fusion_pipeline.cc
    ui/main_window.cc
    exe/gui.cc
)
add_library(tightly_coupled_fusion ${TIGHTLY_COUPLED_FUSION_SOURCES})
target_include_directories(tightly_coupled_fusion PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(tightly_coupled_fusion PUBLIC ${PROJECT_DEPENDENCIES};fusion_helper) # add dependency found in toplevel cmake

# add executable for tightly_coupled_fusion
add_executable(fusion_graph_ba_sanity_check samples/fusion_graph_ba_sanity_check.cpp)
target_link_libraries(fusion_graph_ba_sanity_check tightly_coupled_fusion)



# add modified exe from original colmap to call gui with custom code of this repo
add_executable(colfu exe/colmap.cc)
target_link_libraries(colfu tightly_coupled_fusion)

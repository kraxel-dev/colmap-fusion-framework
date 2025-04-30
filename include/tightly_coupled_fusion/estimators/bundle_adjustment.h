/**
 * @file bundle_adjustment.h
 * @author kraxel
 * @brief
 * @ref (original colmap repo) src/colmap/estimators/bundle_adjustment.h
 * @version 0.1
 * @date 2025-03-12
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/scene/reconstruction.h>
#include <fusion_helper/fusion_iteration_callback.h>
#include <fusion_helper/rr_fusion_recorder.h>

// -------------------- Custom implementation

namespace tcf {  // tightly coupled fusion

/// Options for tightly coupled colmap fusion with odometry data TODO: other modalities
struct FusionGraphBundleAdjustmentOptions {
  bool is_mapping_with_fusion = true;  // if not, switch to regular (vision-only) incremental mapping with rerun visualization.

  // FIXME: expose to user
  std::string tum_file = "/home/azuo/transfer/eval/backwards/vehicle_wo_as_campose_training_matched_stamps.tum";

  // FIXME: expose to user
  double cov = 0.015;  // odom covariance all entries

  // set pose of first camera (time sorted) in active Bundle Adjustemnt as constant param in ceres optimizaton
  bool fix_first_cam_pose = true;
  // set position of 2nd camera (time sorted) in active Bundle Adjustemnt (global + local) as constant param in ceres optimizaton. true in
  // original colmap default behavior to fix the scale during vision only BA. But should be false in fusion to adjust to metric scale from
  // odometry.
  bool fix_second_cam_position = false;

  bool fusion_in_local_ba = true;   // whether to include odometry edges in local BA
  bool fusion_in_global_ba = true;  // whether to include odometry edges in global BA

  double time_between_local_ba = 1.0;  // [secs] passed time between reg images to allow new round of local BA during mapping

  // FIXME: Kick section below if not needed
  //   // Whether to use a robust loss on prior locations.
  //   bool use_robust_loss_on_prior_position = false;

  //   // Threshold on the residual for the robust loss
  //   // (chi2 for 3DOF at 95% = 7.815).
  //   double prior_position_loss_scale = 7.815;

  // Maximum RANSAC error for Sim3 alignment.
  double ransac_max_error = 0.;
};

////////////////////////////////////////////////////////////////////////////////
// Factory creator methods
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Factory create custom fusion bundle adjuster object that can be used in colmaps incremental mapping pipeline.
 *
 * @param options ba options (global vs local)
 * @param fusion_options
 * @param rr_options rerun visaulization options
 * @param config pre-populated config that states which images and points will be considered in ceres for building the factor graph
 * @param reconstruction full colmap reconstruction
 * @param fusion_graph_data_edges full (non-filtered) fusion graph data edges (image edges with odometry) that adds relative pose
 * constraints to the ceres optimization. Will be filtered internally to only keep edges that are active in the current BA problem.
 * @return std::unique_ptr<colmap::BundleAdjuster>
 */
std::unique_ptr<colmap::BundleAdjuster> CreateFusionGraphBundleAdjuster(
    colmap::BundleAdjustmentOptions options,
    const tcf::FusionGraphBundleAdjustmentOptions& fusion_options,
    const fuhe::rrfuse::RerunFusionVisOptions& rr_options,
    const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder,
    colmap::BundleAdjustmentConfig config,
    colmap::Reconstruction& reconstruction,
    const fuhe::edges::MapOfImageEdges& fusion_graph_data_edges);

std::unique_ptr<colmap::BundleAdjuster> CreateDefaultBundleAdjusterRerun(
    colmap::BundleAdjustmentOptions options,
    colmap::BundleAdjustmentConfig config,
    colmap::Reconstruction& reconstruction,
    const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder);

}  // namespace tcf
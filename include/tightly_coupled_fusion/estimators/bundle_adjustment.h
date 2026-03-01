/**
 * @file bundle_adjustment.h
 * @author kraxel
 * @brief Drived versions of colmaps default BundleAdjuster classes to:
 - introduce rerun visualization into colmaps standard bundle adjustment optimization.
 - introduce colmap BundleAdjustment with fusion capabilities of other sensor modalities (e.g. odometry).
 The BA obejcts in this (and the original colmap) header are in charge of building and solving the ceres optimization
 problem. Actual implementations are in the corresponding .cc file.
 * @source: (original colmap repo) src/colmap/estimators/bundle_adjustment.h
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
#include <fusion_helper/rr_sfm_logger.h>

// -------------------- Custom implementation

namespace tcf {  // tightly coupled fusion

/// Options for tightly coupled colmap fusion with odometry data TODO: other modalities
struct FusionGraphBundleAdjustmentOptions {
  bool is_mapping_with_fusion = true;  // if not, switch to regular (vision-only) incremental mapping with rerun visualization.

  //! FIXME: remove eventually since its not used anymore
  std::string tum_file = "/home/azuo/transfer/eval/backwards/vehicle_wo_as_campose_training_matched_stamps.tum";

  // set pose of first camera (time sorted) in active Bundle Adjustment as constant param in ceres optimization
  bool fix_first_cam_pose = true;
  // set position of 2nd camera (time sorted) in active Bundle Adjustment (global + local) as constant param in ceres
  // optimization. true in original colmap default behavior to fix the scale during vision only BA. But should be false in fusion
  // to adjust to metric scale from odometry.
  bool fix_second_cam_position = false;

  // whether to include odometry edges in local BA
  bool fusion_in_local_ba = true;
  // whether to include odometry edges in global BA
  bool fusion_in_global_ba = true;

  // whether to estimate real world-scale between colmap-model and odometry as through scale-aware ceres factors or brute-force
  // scale through enforcing the odometry measurements. If false, scale-aware optim will always be deployed (e.g. during
  // fusion mapping process). If brute force is toggled make sure to reduce measurement covariance to enforce the relative
  // odometry scale onto the camera poses.
  bool brute_force_scale_recovery = true;
  // estimated scale diff between colmap model and rel pose measurements above this value will be ignored
  double scale_diff_thresh = 0.92;
  // Cauchy loss on ceres scale parameter estimatinon. only valid if not brute force scale recovery
  bool use_robust_loss_on_scale_estimation = true;
  // Blindly taken from colmaps PosePriorBA options: Threshold on the residual for the robust loss (chi2 for 3DOF at 95%
  // = 7.815).
  double scale_estimation_loss_factor = 7.815;

  // [secs] passed time between reg images to allow new round of local BA during mapping. This is opposed to colmaps default
  // behavior in which local ba is triggered after each successfully registered image.
  double time_between_local_ba = 0.0001;
};

////////////////////////////////////////////////////////////////////////////////
// Factory creator methods
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Factory create custom fusion bundle adjuster object that can be used in colmaps incremental mapping pipeline.
 *
 * @param options BA options (global vs local)
 * @param fusion_options options for fusion enhanced BA
 * @param rr_sfm_logger custom rerun sfm logger object. Please provide nullptr if streaming to rerun is not desired
 * @param config pre-populated ba_config that states which images and points will be considered in ceres for building the factor
 * graph
 * @param reconstruction full colmap reconstruction
 * @param fusion_graph_data_edges full (non-filtered) fusion graph data edges (image edges with odometry) that adds relative pose
 * constraints to the ceres optimization. Will be filtered internally to only keep edges that are active in the current BA
 * problem.
 * @return std::unique_ptr<colmap::BundleAdjuster>
 */
std::unique_ptr<colmap::BundleAdjuster> CreateFusionGraphBundleAdjuster(
    colmap::BundleAdjustmentOptions options,
    const tcf::FusionGraphBundleAdjustmentOptions& fusion_options,
    colmap::BundleAdjustmentConfig config,
    colmap::Reconstruction& reconstruction,
    const fuhe::edges::MapOfImageEdges& fusion_graph_data_edges,
    const std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger = nullptr);

/**
 * @brief Create a Default Bundle Adjuster with capabilities to stream optimization process to rerun.
 *
 * @param options BA options (global vs local)
 * @param config pre-populated config that states which images and points will be considered in ceres for building the factor
 * graph
 * @param reconstruction rerun visaulization options
 * @param rr_sfm_logger custom rerun sfm logger object. Please provide nullptr if streaming to rerun is not desired

 * @return std::unique_ptr<colmap::BundleAdjuster>
 */
std::unique_ptr<colmap::BundleAdjuster> CreateDefaultBundleAdjusterRerun(
    colmap::BundleAdjustmentOptions options,
    colmap::BundleAdjustmentConfig config,
    colmap::Reconstruction& reconstruction,
    const std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger = nullptr);

}  // namespace tcf
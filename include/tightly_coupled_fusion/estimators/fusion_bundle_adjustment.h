/**
 * @file bundle_adjustment_original.h
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

#include "tightly_coupled_fusion/estimators/bundle_adjustment_original.h"
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
  // FIXME: make parametrizable
  std::string tum_file = "/home/azuo/transfer/eval/backwards/vehicle_wo_as_campose_training_matched_stamps.tum";

  const double cov = 0.01;  // odom covariance all entries

  // FIXME: Kick section below if not needed
  //   // Whether to use a robust loss on prior locations.
  //   bool use_robust_loss_on_prior_position = false;

  //   // Threshold on the residual for the robust loss
  //   // (chi2 for 3DOF at 95% = 7.815).
  //   double prior_position_loss_scale = 7.815;

  // Maximum RANSAC error for Sim3 alignment.
  double ransac_max_error = 0.;
};

class FusionGraphBundleAdjuster : public colmap::BundleAdjuster {
 public:
  FusionGraphBundleAdjuster(colmap::BundleAdjustmentOptions options,
                            const tcf::FusionGraphBundleAdjustmentOptions& fusion_options,
                            const fuhe::rrfuse::RerunFusionVisOptions& rr_options,
                            colmap::BundleAdjustmentConfig config,
                            colmap::Reconstruction& reconstruction);

  void AddOdomToProblem(const colmap::image_t img_id_i,
                        const colmap::image_t img_id_j,
                        const Eigen::Isometry3d& i_from_j,
                        const Eigen::Matrix<double, 6, 6> cov_i_from_j);

  std::shared_ptr<ceres::Problem>& Problem() override;

  ceres::Solver::Summary Solve() override;

 private:
  const tcf::FusionGraphBundleAdjustmentOptions fusion_options_;  // tum file path, cov mat, etc
  const fuhe::rrfuse::RerunFusionVisOptions rr_options_;          // rerun visualization options
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_rec_ = nullptr;
  std::shared_ptr<fuhe::FusionIterationCallback> iter_callback_ = nullptr;  // custom iteration callback to log data to rerun if toggled

  const std::unordered_map<colmap::image_t, colmap::PosePrior> rel_poses_;  // FIXME: kick if not needed
  colmap::Sim3d normalized_from_metric_;                                    // TODO: kick is not needed

  std::shared_ptr<fuhe::types::MapOfImageIdsSec> imgs_by_stamp_ =
      nullptr;  // registered colmap image ids sorted by their timestamp (ascending)
  std::shared_ptr<fuhe::edges::MapOfOdomEdges> fusion_graph_data_edges_ = nullptr;  // odometry data edges constraining the images
  colmap::Reconstruction& reconstruction_;

  std::unique_ptr<colmap::DefaultBundleAdjuster> default_bundle_adjuster_;
  std::unique_ptr<ceres::LossFunction> prior_loss_function_;  // FIXME: decide to kick

  /// Attach iteration callback that logs visualization data to rerun during optimization
  void AddFusionIterationCallback(ceres::Solver::Options& solver_options);
};

}  // namespace tcf
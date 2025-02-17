/**
 * @file fusion_graph_interface.h
 * @author kraxel
 * @brief Helper functions for:
 * - generating ceres sensor cost factors from colmap models and other modalities
 * - adding them to the ceres bundle adjustment graph / ceres optimizatino problem.
 * Used for high-level fusion where colmap models are assumed to be
 * fully reconstructed.
 * @version 0.1
 * @date 2025-02-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstdlib>

#include <ceres/problem.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/exe/sfm.h>
#include <fusion_helper/types.h>
#include <rerun.hpp>

namespace hifuse {  // high-level fusion

class FusionGraphInterface {
 public:
  FusionGraphInterface(std::shared_ptr<colmap::Reconstruction>& reconstruction,
                       std::shared_ptr<ceres::Problem>& ceres_graph,
                       bool log_to_rerun = true);
  ~FusionGraphInterface() = default;

  void AddReprojectionFactor(const colmap::image_t img_id,
                             const bool const_t = false,
                             const bool const_q = false,
                             const bool const_3d_pts = false);

  void AddBetweenFactor(const colmap::image_t img_id_i,
                        const colmap::image_t img_id_j,
                        const Eigen::Isometry3d& i_from_j,
                        const Eigen::Matrix<double, 6, 6> cov_i_from_j);

  // TODO: kick out once ceres callback is implemented with rerun
  /// update registered reprojection and rel pose factors in rerun in one swoop
  void UpdateRegisterdFactorsRerun(const fuhe::types::MapOfPosesSec& metric_poses);
  /// update colmap image poses and 3d points in rerun in one swoop
  void UpdateWholeReconstroctionRerun();

  std::shared_ptr<ceres::Problem> GetCeresGraph() { return this->ceres_graph; }
  std::shared_ptr<colmap::Reconstruction> GetReconstruction() { return this->reconstruction; }

  std::shared_ptr<rerun::RecordingStream> GetRerunRec() const { return this->rr_rec; }
  std::shared_ptr<rerun::Pinhole> GetRerunPinhole() const { return this->rr_pinhole; }

 private:
  bool is_log_to_rerun = true;  // flag to enable logging and visualization of graph construction and optimization to rerun
  std::shared_ptr<rerun::RecordingStream> rr_rec = nullptr;   // rerun logger and viewer object
  std::shared_ptr<rerun::Pinhole> rr_pinhole = nullptr;       // rerun pinhole model representing the camera used in colmap model
  std::shared_ptr<rerun::Pinhole> rr_pinhole_pred = nullptr;  // rerun pinhole model representing the predicted position through odometry

  std::shared_ptr<ceres::Problem> ceres_graph;             // ceres problem that acts as factor graph
  std::shared_ptr<colmap::Reconstruction> reconstruction;  // colmap model to be used for factor graph construction

  std::vector<std::vector<ceres::ResidualBlockId>>
      reproj_residual_ids;  // ceres ids for registerd reprojection factors for all images (each image has multiple residuals)
  std::vector<ceres::ResidualBlockId>
      odom_residual_ids;  // ceres ids for registerd odom factors such that we can perform residual evaluation

  void InitRerunViewer();
};

}  // namespace hifuse
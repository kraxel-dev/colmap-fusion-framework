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

#include <ceres/problem.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/exe/sfm.h>
#include <fusion_helper/fusion_residuals_tracker.h>
#include <fusion_helper/types.h>
#include <rerun.hpp>

namespace hifuse {  // high-level fusion

class FusionGraphInterface {
 public:
  FusionGraphInterface(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                       ceres::Problem& ceres_graph,
                       const bool track_residuals,
                       const bool log_to_rerun = true,
                       const bool save_rerun_recording = false,
                       const std::string recording_path = "");
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

  inline const std::shared_ptr<colmap::Reconstruction> GetReconstruction() { return this->reconstruction; }

  inline std::shared_ptr<rerun::RecordingStream> GetRerunRec() const { return this->rr_rec; }
  inline std::shared_ptr<rerun::Pinhole> GetRerunPinhole() const { return this->rr_pinhole; }

  inline std::vector<std::vector<ceres::ResidualBlockId>> GetReprojResidualIds() const { return this->reproj_residual_ids; }
  inline std::vector<ceres::ResidualBlockId> GetOdomResidualIds() const { return this->odom_residual_ids; }

  /// nullptr if residual tracking is deactivated by user
  inline const std::shared_ptr<fuhe::FusionResidualsTracker> GetResidualsTracker() const { return residuals_tracker; }

 private:
  bool is_log_to_rerun = true;         // flag to enable logging and visualization of graph construction and optimization to rerun
  bool is_save_rerun_to_disk = false;  // enable saving rerun logged data to disk as rrd file
  std::string recording_path = "";
  std::shared_ptr<rerun::RecordingStream> rr_rec = nullptr;   // rerun logger and viewer object
  std::shared_ptr<rerun::Pinhole> rr_pinhole = nullptr;       // rerun pinhole model representing the camera used in colmap model
  std::shared_ptr<rerun::Pinhole> rr_pinhole_pred = nullptr;  // rerun pinhole model representing the predicted position through odometry

  // NOTE: acceppt ceres problem only as reference
  ceres::Problem& ceres_graph;                                   // ceres problem that acts as factor graph
  const std::shared_ptr<colmap::Reconstruction> reconstruction;  // colmap model to be used for factor graph construction

  bool is_track_residuals = false;  // whether to track cost function residual for each ceres iteration
  std::shared_ptr<fuhe::FusionResidualsTracker> residuals_tracker =
      nullptr;  // for storing and tracking residuals of each registered ceres cost function
  std::vector<std::vector<ceres::ResidualBlockId>>
      reproj_residual_ids;  // ceres ids for registerd reprojection factors for all images (each image has multiple residuals)
  std::vector<ceres::ResidualBlockId>
      odom_residual_ids;  // ceres ids for registerd odom factors such that we can perform residual evaluation

  void InitRerunViewer(); // FIXME: swap out with new fusion logger class
};

}  // namespace hifuse
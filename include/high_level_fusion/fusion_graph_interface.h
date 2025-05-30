/**
 * @file fusion_graph_interface.h
 * @author kraxel
 * @brief Interface class with methods to:
 * - create ceres sensor cost factors from a colmap model and other modalities.
 * - adding them to the ceres bundle adjustment graph / ceres optimizatino problem.
 * Used for high-level fusion where colmap models are assumed to be fully reconstructed and is seeked to be optimized through
 * Bundle Adjustment and sensor factors from external modalities like wheel odometry.
 *
 * This class was mainly written for personal familiarization and debugging of factor-graph concepts AND as platform/playground
 * easily integrate and test additional factors (like IMU) before integrating it properly in tightly-coupled fusion.
 *
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

/**
 * @brief Options needed to perform high level fusion on a colmap model with other sensor modalities
 *
 */
struct HighLevelFusionOptions {
  // scalar covariance value for relative odometry measurement that will be used for all 6x6 cov matrix entries. Small covariance
  // lead to relative odometry strongly being considered by the optimization
  double cov = 0.1;

  // file path to tum file containing absolute poses from external odometry source
  std::string tum_file = "";

  // whether to track and print residuals of each factor during ceres optimization. Only in conjunction with rerun logging
  bool track_residuals = false;
};

/**
 * @brief Interface class that helps build a ceres fusion optimization problem from a colmap model and external sensor soruces
 * like odometry. Is not in charge of creating the graph edges that it should iterate over, but assumes correctly associated
 * odometry and image data.
 * Actively exposes many steps to the user as this class was mainly written for personal familiarization and debugging of
 * factor-graph concepts AND as platform/playground easily integrate and test additional factors (like IMU) before integrating it
 * properly in tightly-coupled fusion.
 *
 */
class FusionGraphInterface {
 public:
  FusionGraphInterface(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                       ceres::Problem& ceres_graph,
                       const bool track_residuals,
                       const bool log_to_rerun = true,
                       const bool save_rerun_recording = false,
                       const std::string recording_path = "");
  ~FusionGraphInterface() = default;

  /**
   * @brief Given a colmap image id, registers reprojection error for all 3d-2d image point correspondences (known from colmap)
   * to internally stored ceres problem. FIXME: Force camera intrinsics to constant (currently actively optimized)
   *
   * @param img_id colmap image id of target image
   * @param const_t set translation of image constant during optimization
   * @param const_q set orientation of image constant during optimization
   * @param const_3d_pts set xyz position of 3d point constant during optimization
   */
  void AddReprojectionFactors(const colmap::image_t img_id,
                              const bool const_t = false,
                              const bool const_q = false,
                              const bool const_3d_pts = false);

  /**
   * @brief Given a relative pose measurement and the 2 image ids it should constrain, register 6DoF relative pose factor between
   * 2 image poses to internally stored ceres problem.
   *
   * @param img_id_i colmap image id of pose i
   * @param img_id_j colmap image id of pose j
   * @param i_from_j Measured realtive pose of j expressed in i
   * @param cov_i_from_j 6x6 covariance of relative pose. Due to colmap notation: first 3 entries are rotation, last 3 are
   * translational noise
   */
  void AddBetweenFactor(const colmap::image_t img_id_i,
                        const colmap::image_t img_id_j,
                        const Eigen::Isometry3d& i_from_j,
                        const Eigen::Matrix<double, 6, 6> cov_i_from_j);

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
  bool is_log_to_rerun = true;  // flag to enable logging and visualization of graph construction and optimization to rerun
  bool is_save_rerun_to_disk = false;  // enable saving rerun logged data to disk as rrd file
  std::string recording_path = "";
  std::shared_ptr<rerun::RecordingStream> rr_rec = nullptr;  // rerun logger and viewer object
  std::shared_ptr<rerun::Pinhole> rr_pinhole = nullptr;      // rerun pinhole model representing the camera used in colmap model
  std::shared_ptr<rerun::Pinhole> rr_pinhole_pred =
      nullptr;  // rerun pinhole model representing the predicted position through odometry

  // NOTE: acceppt ceres problem only as reference
  ceres::Problem& ceres_graph;                                   // ceres problem that acts as factor graph
  const std::shared_ptr<colmap::Reconstruction> reconstruction;  // colmap model to be used for factor graph construction

  bool is_track_residuals = false;  // whether to track cost function residual for each ceres iteration
  // for storing and tracking residuals of each registered ceres cost function by category
  std::shared_ptr<fuhe::FusionResidualsTracker> residuals_tracker = nullptr;
  // ceres ids for registerd reprojection factors for all images (each image has multiple residuals)
  std::vector<std::vector<ceres::ResidualBlockId>> reproj_residual_ids;
  // ceres ids for registerd odom factors such that we can perform residual evaluation
  std::vector<ceres::ResidualBlockId> odom_residual_ids;

  void InitRerunViewer();  // FIXME: swap out with new fusion logger class
};

}  // namespace hifuse
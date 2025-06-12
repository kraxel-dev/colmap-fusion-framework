/**
 * @file rr_sfm_logger.h
 * @author kraxel
 * @brief Helper class in charge of rerun viewer initialization and logging of colmap models, bundle adjustment and fusion
 * graph optimization process to rerun.
 * @version 0.1
 * @date 2025-03-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "fusion_helper/rr_utils.h"
#include "fusion_helper/types.h"
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/geometry/rigid3.h>
#include <colmap/scene/reconstruction.h>
#include <rerun.hpp>

namespace fuhe {
namespace rr {

/// rerun visualization options for colmap models, bundle adjustment and fusion graph optimization process
struct RerunVisualizationOptions {
  // enable logging and visualization to rerun
  bool is_log_to_rerun = true;
  // enable saving rerun logged data to disk as rrd file. Note that real-time logging deactivates with this.
  bool is_save_rerun_to_disk = false;
  std::string recording_path = "";
  float img_plane_dist = 0.2f;  // controls size of cam pinhole in rerun viewer //FIXME: CURRENTLY NOT USED please kick

  // whether to draw external odometry as predicted poses with respect to source camera or as absolute poses
  bool draw_rerun_odom_as_predicted_poses = true;

  // whether to highlight active images of BA in rerun with bounding boxes. Deactivate for use cases of BA in which the full
  // model is part of the optimization and the view can get cluttered as a result (high-lvl fusion for example).
  bool is_highlight_active_cams = true;

  // whether to show camera labels in rerun viewer. Might clutter scene.
  bool is_show_cam_labels = false;

  // whether to ignore 3d points in rerun viewer that are outside model bounding box computed from the majority of the 3d points.
  // Ideal to get rid of bogus pts that mess up the rerun viz
  bool is_ignore_pts_beyond_model_bbox = true;
  float model_bbox_lb = 0.05;  // lower bound of ommited percentiles in bbox
  float model_bbox_ub = 0.95;  // upper bound of ommited percentiles in bbox
};

/**
 * @brief In charge of rerun viewer initialization and logging of colmap models, bundle adjustment and fusion
 * graph optimization process to rerun. Provides a rerun recording stream and pinhole model based on the first camera in the
 * COLMAP model. Can be used to log stand-alone colmap models or in a ceres iteration callback to log each optimization step in
 * the bundle adjustment process. Inspired by glomap's rerun example:
 * https://github.com/colmap/glomap/commit/5115de482dc0a72b5c6d01d39da3524b7a296608#diff-2139378b2a5608827fa60ae83cfc220b18a3c68e7972248aeb715da8b60594fc
 *
 */
class RerunSfmLogger {
 public:
  RerunSfmLogger(const RerunVisualizationOptions& rr_opts, std::shared_ptr<colmap::Reconstruction> reconstruction);
  ~RerunSfmLogger() = default;

  std::shared_ptr<rerun::RecordingStream> GetRerunRec() const;
  std::shared_ptr<rerun::Pinhole> GetRerunPinhole() const;
  inline int& TimeStep() { return time_step_; }
  inline void SetTimeStep(int time_step) { time_step_ = time_step; }

  // FIXME: write Brief
  // @source:
  // https://github.com/colmap/glomap/commit/5115de482dc0a72b5c6d01d39da3524b7a296608#diff-2139378b2a5608827fa60ae83cfc220b18a3c68e7972248aeb715da8b60594fc
  void LogFullReconstruction();

  /**
   * @brief Given full range of imgs and 3d points, log only active points and images in an ongoing Bundle Adjustment (local or
   * global) to rerun. Active subset of imgs and points are determined by a populated ba_config (make sure its populated
   * correctly which is normally done by colmap itself). Almost as LogFullReconstruction but with different colors for this
   * subset. Can be used in ceres iteration callback.
   *
   * @param ba_config bundle adjustment config that contains the active images and points3D in the current BA problem
   */
  void LogActivBundle(const colmap::BundleAdjustmentConfig* ba_config);

  /**
   * @brief Clear all temporary visualizations of an active bundle adjustment process (cam bbs and subsetted 3D points) in rerun.
   * Use in destructors of ceres marathon iteration callbacks.
   *
   */
  void ClearActiveBundle();

  /**
   * @brief Update the bounding box of the model based on the 3D points of the colmap reconstruction. With an BBox object, rerun
   * can ignore 3D points that are beyond the model bounding box in the viewer. This is useful to get rid of bogus points. Better
   * to only use this before an Bundle Adjustment call, therefore let ceres iteration callbacks handle this one.
   *
   */
  void UpdateModelBBox();

  /**
   * @brief Log the camera pose of a colmap image (w.r.t world) to rerun viewer. The camera pose is logged as a Transform3D and a
   * Pinhole object.
   *
   * @param img the colmap image to log
   * @param highlight whether to highlight the camera pose with a bounding box in rerun viewer. Useful for signaling local (or
   * global) bundle adjustment but not for full model visualization as it can get cluttered.
   */
  void LogCamPose(const colmap::Image& img, const bool highlight = false);
  void LogCamPose(const colmap::image_t& id, const bool highlight = false);

  /**
   * @brief Log the 3D points of a colmap reconstruction to rerun viewer. The points are logged as a Points3D object.
   *
   * @param points3D the colmap 3D points to log to rerun viewer.
   * @param is_subset Whether the points3Ds represent a subset of all model points. Useful for signaling the active points in a
   * local (or global) bundle adjustment. Make sure to do the subsetting yourself though. If true, the points3D will be logged
   * with a different color and entity name in rerun.
   */
  void LogPoints3D(const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D, const bool is_subset = false);

  /**
   * @brief Log a text log message to rerun viewer
   *
   * @param msg the message to log
   */
  void LogInfoMsg(const std::string& msg);

  /**
   * @brief Getter for attached colmap reconstruction.
   *
   * @return std::shared_ptr<colmap::Reconstruction>
   */
  std::shared_ptr<colmap::Reconstruction> Reconstruction() const;

 protected:
  const RerunVisualizationOptions rr_options_;

  // rerun logger/streamer and viewer object
  std::shared_ptr<rerun::RecordingStream> rr_rec_ = nullptr;
  // rerun pinhole model representing the camera dims used in colmap model
  std::shared_ptr<rerun::Pinhole> rr_pinhole_ = nullptr;
  // time step of rerun recording stream. Used to log data in chronological order.
  int time_step_ = 0;

  // colmap reconstruction to log data from
  std::shared_ptr<colmap::Reconstruction> reconstruction_ = nullptr;
  // bounding box of the model computed from model 3D points
  std::shared_ptr<fuhe::types::ColmapBBox> model_bbox_ = nullptr;

  /**
   * @brief Increment time sequence for rerun recording stream. Should be called internally by methods that know when to
   * increment.
   *
   */
  void UpdateRerunTimeStep();
};

}  // namespace rr

}  // namespace fuhe
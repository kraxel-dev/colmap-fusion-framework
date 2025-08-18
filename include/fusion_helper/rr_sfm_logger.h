/**
 * @file rr_sfm_logger.h
 * @author kraxel
 * @brief Convenience classes in charge of rerun viewer initialization and logging colmap data (imgs, 3d points or whole
 * reconstruction), bundle adjustment and fusion graph optimization process to rerun. Most exciting when used during active ceres
 * optimization step in conjunction with the ceres iteration callback classese of this repo .
 * @version 0.1
 * @date 2025-03-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "fusion_helper/odom_edges.h"
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
  std::string recording_path = ".";
  float img_plane_dist = 0.2f;  // controls size of cam pinhole in rerun viewer

  // whether to draw external odometry as predicted poses with respect to source camera or as absolute poses
  bool draw_rerun_odom_as_predicted_poses = true;

  // whether to highlight active images of BA in rerun with bounding boxes. Deactivate for use cases of BA in which the full
  // model is part of the optimization and the view can get cluttered as a result (high-lvl fusion for example).
  bool is_highlight_active_cams = true;

  // whether to show camera labels in rerun viewer. Might clutter scene.
  bool is_show_cam_labels = false;
  // whether to show odometry edge between images. Might clutter scene.
  bool is_show_edge_labels = false;

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

  /**
   * @brief Log the full COLMAP reconstruction (all 3d pts and registered images) from the model that you have attached to this
   * logger object.
   *
   */
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
   * @brief Interanlly updates the bounding box of the attached COLMAP model based on its 3D points. With an BBox object, rerun
   * will ignore 3D points that are beyond the model bounding box in the viewer. Must be toggled from rerun options. Useful to
   * get rid of bogus points. Do not forget to call this method regularily during active mapping, otherwise new points will be
   * cut by an old BBox. Better to only use this before an Bundle Adjustment call, therefore let ceres iteration callbacks handle
   * this one.
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
   * @brief Streams the total ceres cost of a factor type for current iterations to rerun.
   *
   * @param factor_type
   * @param total_cost
   */
  void LogTotalFactorCost(const std::string& factor_type, const double total_cost);

  /**
   * @brief Getter for attached colmap reconstruction.
   *
   * @return std::shared_ptr<colmap::Reconstruction>
   */
  std::shared_ptr<colmap::Reconstruction> Reconstruction() const;
  RerunVisualizationOptions RerunOptions() const;
  const float GetFrameAxisLen() const;

 protected:
  const RerunVisualizationOptions rr_options_;

  // rerun logger/streamer and viewer object
  std::shared_ptr<rerun::RecordingStream> rr_rec_ = nullptr;
  // rerun pinhole model representing the camera dims used in colmap model
  std::shared_ptr<rerun::Pinhole> rr_pinhole_ = nullptr;
  // time step of rerun recording stream. Used to log data in chronological order.
  int time_step_ = 0;

  // length of frame axis for poses in rerun viewer
  float frame_axis_len_ = 0.3f;

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

/**
 * @brief Composition-based Rerun Logger class that (additionally to what a RerunSfmLogger already streams to rerun), given the
 * odometry edges for the BA fusion, streams odometry edges (from an external odom source) between 2 image poses of a COLMAP
 * model. Handy to understand the BA + odom fusion process better. Ideally, instantiation of this object should be handled by
 * ceres iterartion callbacks and not be used otherwise.
 *
 */
class RerunFusionGraphLogger {
 public:
  /**
   * @brief Construct a new Rerun Fusion Graph Logger object
   *
   * @param rr_sfm_logger Already active instance of an rerun sfm logger (should hold the COLMAP reconstruction)
   * @param active_fusion_graph_edges Graph nodes holding the odometry edges. Edges need to already be prefiltered by an upstream
   * instance (e.g. filter through ba_config)
   */
  RerunFusionGraphLogger(std::shared_ptr<RerunSfmLogger> rr_sfm_logger,
                         const fuhe::edges::MapOfImageEdges& active_fusion_graph_edges)
      : rr_sfm_logger_{rr_sfm_logger}, active_fusion_graph_edges_{active_fusion_graph_edges} {
    if (!rr_sfm_logger_) {
      LOG(ERROR) << "Rerun Fusion Graph Logger was provided an invalid Sfm Logger. The Colmap model and the fusion optimization "
                    "process cannot be streamed to rerun like this!";
    }
  };

  /**
   * @brief Call to SfmLogger member's LogFullReconstruction()
   *
   */
  void LogFullReconstruction();

  /**
   * @brief Call to SfmLogger member's LogActivBundle(const colmap::BundleAdjustmentConfig* ba_config)
   *
   * @param ba_config bundle adjustment config that contains the active images and points3D in the current BA problem
   */
  void LogActivBundle(const colmap::BundleAdjustmentConfig* ba_config);

  /**
   * @brief Log all relative poses of external odometry as predicted poses as seen from node i for all nodes i j. Note that valid
   * and filtered Fusion Graph Edges (passed during object constrction) are required to call this method.
   *
   */
  void LogOdometryEdges();

  /**
   * @brief  Draw all external odometry measurements as absolute poses and visually constrain the absolute colmap image poses
   * with them as edge.
   *
   */
  void LogOdometryEdgesAsTrajectory();

  /**
   * @brief Clear all odometry edges streamed to rerun previously
   *
   */
  void ClearAllOdometryEdges();

  /**
   * @brief Streams the total ceres cost of a factor type for current iterations to rerun.
   *
   * @param factor_type
   * @param total_cost
   */
  void LogTotalFactorCost(const std::string& factor_type, const double total_cost);

  inline std::shared_ptr<RerunSfmLogger> GetSfmLogger() const { return rr_sfm_logger_; }
  inline std::shared_ptr<rerun::RecordingStream> GetRerunRec() const { return rr_sfm_logger_->GetRerunRec(); }
  inline RerunVisualizationOptions RerunOptions() const { return rr_sfm_logger_->RerunOptions(); }
  inline std::shared_ptr<colmap::Reconstruction> Reconstruction() const { return rr_sfm_logger_->Reconstruction(); }

 protected:
  std::shared_ptr<RerunSfmLogger> rr_sfm_logger_ = nullptr;
  fuhe::edges::MapOfImageEdges active_fusion_graph_edges_;

  /**
   * @brief Log odometry edge constraining to colmap nodes i j as linestrip and the pose of the odometry measruement to highlight
   * them as factor graph edge. Pose can be drawn as relative increment (seen from source image i) or as absolute pose with
   * respect to some coord frame. When choosing to draw odometry reading as absolute pose, user must provide the correct absolute
   * pose himself. Linestrip drawing of edges works automatically for both cases.
   *
   * @param T_ij_odom
   * @param img_i id of colmap cam pose i (source) that the odom image edges should constrain
   * @param img_j id of colmap cam pose j (dest) that the odom image edges should constrain
   * @param is_odom_as_pred_pose if forwarded odom pose T_ij_odom can be understood as relative pose of j w.r.t to i. If not, an
   * absolute pose is assumed.
   */
  void LogOdometryEdge(const colmap::Rigid3d& T_ij_odom,
                       const colmap::Image& img_i,
                       const colmap::Image& img_j,
                       const bool is_odom_as_pred_pose = true);
};

}  // namespace rr

}  // namespace fuhe
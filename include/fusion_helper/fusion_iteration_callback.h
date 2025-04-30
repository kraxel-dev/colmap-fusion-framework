#pragma once

#include "fusion_helper/col_utils.h"
#include "fusion_helper/fusion_residuals_tracker.h"
#include "fusion_helper/rr_fusion_logging.h"
#include "fusion_helper/rr_fusion_recorder.h"
#include "fusion_helper/rr_utils.h"
#include <ceres/ceres.h>
#include <ceres/iteration_callback.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/scene/image.h>
#include <colmap/scene/point3d.h>
#include <rerun.hpp>

namespace fuhe {

/**
 * @brief Ceres iteration callback, called during every iteration of ceres optimization. Derived to log colmap reconstruction pose and point
 * updates during each optim steps to rerun. Works for vanilla colmap models from original colmap repo and can be used when you build your
 * ceres problem manually (e.g. in high level fusion)
 * @ref 1. https://github.com/rerun-io/glomap/blob/main/glomap/estimators/global_positioning.cc#L26
 *      2. http://ceres-solver.org/nnls_solving.html#_CPPv4N5ceres17IterationCallbackE
 *
 */
class BundleAdjustmentIterationCallback : public ceres::IterationCallback {
 public:
  BundleAdjustmentIterationCallback(const std::shared_ptr<rerun::RecordingStream> rr_rec,
                                    const std::shared_ptr<rerun::Pinhole> rrpinhole,
                                    const std::unordered_map<colmap::image_t, colmap::Image>& images,
                                    const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D)
      : rr_rec_{rr_rec}, rrpinhole_{rrpinhole}, images_{images}, points3D_{points3D} {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
    // -------------------- iteration summary logging
    // NOTE: Instead of logging iteration manually here, make use of default ceres per iteration printer by setting:
    //  solver_options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    // in the main executable

    // -------------------- Log full reconstruction
    rr_rec_->set_time_sequence("step", summary.iteration);
    rrfuse::LogReconstruction(rr_rec_, rrpinhole_, images_, points3D_);
    return ceres::SOLVER_CONTINUE;
  }

 protected:
  const std::shared_ptr<rerun::RecordingStream> rr_rec_;
  const std::shared_ptr<rerun::Pinhole> rrpinhole_;
  const std::unordered_map<colmap::camera_t, colmap::Image>& images_;
  const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D_;
};

/**
 * @brief Same as parent but used by colmap internal BundelAdjuster object. Assumes a correctly populated ba_config to only log active imgs
 * and pts. Can log consistently when there are multiple consecutive local and global bundle adjustemnts. This class obtains time step info
 * from outside to log rerun data consistently in chronological order.
 *
 */
class MarathonBundleAdjustIterCallback : public BundleAdjustmentIterationCallback {
 public:
  /**
   * @brief Construct a new Marathon Bundle Adust Iter Callback object
   *
   * @param rr_recorder custom object holding rerun recording stream and pinhole camera. Contains counter to set correct rerun time steps.
   * @param images all images of reconstruction (might contain images that are not registered yet)
   * @param points3D all 3d pts of reconstruction
   * @param ba_config visualize only a subset in rerun considered by BA problem. Requires correctly populated BA config (normaly
   * done by colmap reconstruction process).
   */
  MarathonBundleAdjustIterCallback(const std::shared_ptr<rrfuse::RerunFusionRecorder> rr_recorder,
                                   const std::unordered_map<colmap::image_t, colmap::Image>& images,
                                   const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                                   const colmap::BundleAdjustmentConfig* ba_config)
      : BundleAdjustmentIterationCallback(rr_recorder->GetRerunRec(), rr_recorder->GetRerunPinhole(), images, points3D),
        step_count_{rr_recorder->TimeStep()},
        ba_config_{ba_config} {}

  ~MarathonBundleAdjustIterCallback() override {
    if (ba_config_) {
      // log updated reconstruction to rerun (filter out images that are not yet registered in model and still lack pose info)
      rrfuse::LogReconstruction(rr_rec_, rrpinhole_, col_utils::SubsetOfImages(ba_config_->Images(), images_), points3D_);
      rrfuse::ClearActiveBundle(rr_rec_, ba_config_->Images());
    }
  }

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
    rr_rec_->set_time_sequence("step", step_count_++);
    // log only images and points3D that are active in current BA problem
    rrfuse::LogActivBundle(rr_rec_, rrpinhole_, images_, points3D_, ba_config_);

    return ceres::SOLVER_CONTINUE;
  }

 protected:
  int& step_count_;                                            // track rerun time sequence from outside
  const colmap::BundleAdjustmentConfig* ba_config_ = nullptr;  // visualize only a subset in rerun considered by BA problem
};

/**
 * @brief FIXME: write brief
 *
 */
class MarathonFusionIterCallback : public MarathonBundleAdjustIterCallback {
 public:
  MarathonFusionIterCallback(const std::shared_ptr<rrfuse::RerunFusionRecorder> rr_recorder,
                             const std::unordered_map<colmap::image_t, colmap::Image>& images,
                             const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                             const colmap::BundleAdjustmentConfig* ba_config,
                             const edges::MapOfImageEdges& active_fusion_graph_edges,
                             const bool is_draw_odom_edges_as_pred_pose = true,
                             const bool is_highlight_active_cams = true,
                             const std::shared_ptr<fuhe::FusionResidualsTracker> res_tracker = nullptr)
      : MarathonBundleAdjustIterCallback(rr_recorder, images, points3D, ba_config),
        active_fusion_edges_{active_fusion_graph_edges},
        is_draw_odom_edges_as_pred_pose_{is_draw_odom_edges_as_pred_pose},
        is_highlight_active_cams_{is_highlight_active_cams},
        tracked_residuals_{res_tracker} {}
  ~MarathonFusionIterCallback() override { rrfuse::ClearAllOdometryEdges(this->rr_rec_); }
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
    // -------------------- Log active imgs and pts in current BA
    step_count_++;
    rr_rec_->set_time_sequence("step", step_count_);

    // when a populated BA config knows which images and points are considered for this BA problem, log only those
    rrfuse::LogActivBundle(rr_rec_, rrpinhole_, images_, points3D_, ba_config_, is_highlight_active_cams_);

    // -------------------- visualize odometry edges for this iteraten
    if (is_draw_odom_edges_as_pred_pose_) {
      // draw external odometry as predicted poses with respect to source camera
      rrfuse::LogOdometryEdges(this->rr_rec_, images_, this->active_fusion_edges_);
    } else {
      // draw external odometry as absolute poses
      bool log_odom_trajectory_as_linestrip =
          (summary.iteration > 1) ? false : true;  // draw odometry linestrip only once at first iteration
                                                   // FIXME: refactor to new graph edge data
      rrfuse::LogOdometryEdgesAsTrajectory(rr_rec_, this->images_, this->active_fusion_edges_, log_odom_trajectory_as_linestrip);
    }

    // -------------------- track total factor costs in rerun plots if toggled on
    // if (this->tracked_residuals && summary.iteration > 1) {
    if (this->tracked_residuals_) {
      rrfuse::LogTotalFactorCost(this->rr_rec_, "odom", this->tracked_residuals_->GetTotalOdomCost());
      rrfuse::LogTotalFactorCost(this->rr_rec_, "reproj", this->tracked_residuals_->GetTotalReprojCost());
      rrfuse::LogTotalFactorCost(this->rr_rec_, "graph", summary.cost);
    }
    return ceres::SOLVER_CONTINUE;
  }

 protected:
  // subset of seqeuential image edges wiht active odometry in current BA problem. Make sure to filter properly before passing to this class
  const edges::MapOfImageEdges& active_fusion_edges_;

  const bool is_draw_odom_edges_as_pred_pose_ = true;

  // whether to highlight active images in rerun with bounding boxes. Toggle of for use cases of BA for full model where view can get
  // cluttered.
  const bool is_highlight_active_cams_;

  // residuals_tracker exposing sensor factor residuals during optimization. Will be ingroned if received as nullptr
  std::shared_ptr<fuhe::FusionResidualsTracker> tracked_residuals_ = nullptr;
};

// FIXME: kill class below once compared against the marathon class
/**
 * @brief Ceres iteration callback, called during every iteration of ceres optimization. Derived to log colmap fusion factor graph to
 * rerun. Logs colmap images in sorted order.
 * @ref 1. https://github.com/rerun-io/glomap/blob/main/glomap/estimators/global_positioning.cc#L26
 *      2. http://ceres-solver.org/nnls_solving.html#_CPPv4N5ceres17IterationCallbackE
 *
 */
class FusionIterationCallback : public BundleAdjustmentIterationCallback {
 public:
  FusionIterationCallback(const std::shared_ptr<rerun::RecordingStream> rr_rec,
                          const std::shared_ptr<rerun::Pinhole> rrpinhole,
                          const std::unordered_map<colmap::image_t, colmap::Image>& images,
                          const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                          const edges::MapOfImageEdges& sequential_image_edges,
                          bool is_draw_odom_edges_as_pred_pose = true,
                          const std::shared_ptr<fuhe::FusionResidualsTracker> res_tracker = nullptr)
      : BundleAdjustmentIterationCallback(rr_rec, rrpinhole, images, points3D),
        graph_data_edges{sequential_image_edges},
        is_draw_odom_edges_as_pred_pose{is_draw_odom_edges_as_pred_pose},
        tracked_residuals{res_tracker} {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    rr_rec_->set_time_sequence("step", summary.iteration);

    // --------------------  visualize full reconstruction for this iteration
    std::unordered_map<colmap::camera_t, colmap::Image> imgs_sorted_by_stamp;
    rrfuse::LogReconstruction(this->rr_rec_, this->rrpinhole_, this->images_, this->points3D_);

    // -------------------- visualize odometry edges for this iteraten
    rrfuse::ClearAllOdometryEdges(this->rr_rec_);
    if (is_draw_odom_edges_as_pred_pose) {
      // draw external odometry as predicted poses with respect to source camera
      rrfuse::LogOdometryEdges(this->rr_rec_, this->images_, this->graph_data_edges);
    } else {
      // draw external odometry as absolute poses
      bool log_odom_trajectory_as_linestrip =
          (summary.iteration > 1) ? false : true;  // draw odometry linestrip only once at first iteration
                                                   // FIXME: refactor to new graph edge data
      rrfuse::LogOdometryEdgesAsTrajectory(this->rr_rec_, this->images_, this->graph_data_edges, log_odom_trajectory_as_linestrip);
    }

    // -------------------- track total factor costs in rerun plots if toggled on
    // if (this->tracked_residuals && summary.iteration > 1) {
    if (this->tracked_residuals) {
      rrfuse::LogTotalFactorCost(this->rr_rec_, "odom", this->tracked_residuals->GetTotalOdomCost());
      rrfuse::LogTotalFactorCost(this->rr_rec_, "reproj", this->tracked_residuals->GetTotalReprojCost());
      rrfuse::LogTotalFactorCost(this->rr_rec_, "graph", summary.cost);
    }
    return ceres::SOLVER_CONTINUE;
  }

 protected:
  const edges::MapOfImageEdges& graph_data_edges;

  // whether to draw all external odometry measurements as absolute poses or predictes pose increments, seen from each colmap pose
  bool is_draw_odom_edges_as_pred_pose = true;

  // residuals_tracker exposing sensor factor residuals during optimization. Will be ingroned if received as nullptr
  std::shared_ptr<fuhe::FusionResidualsTracker> tracked_residuals = nullptr;
};

}  // namespace fuhe

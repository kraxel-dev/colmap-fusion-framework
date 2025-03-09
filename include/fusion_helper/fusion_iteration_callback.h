#pragma once

#include "fusion_helper/rr_utils.h"
#include "high_level_fusion/rerun_interface.h"  // TODO: move rerun_interface to fusion helper
#include <ceres/ceres.h>
#include <ceres/iteration_callback.h>
#include <colmap/scene/image.h>
#include <colmap/scene/point3d.h>
#include <rerun.hpp>

namespace fuhe {

/**
 * @brief Ceres iteration callback, called during every iteration of ceres optimization. Derived to log colmap reconstruction to rerun. This
 * is works with a vanilla colmap model.
 * @ref 1. https://github.com/rerun-io/glomap/blob/main/glomap/estimators/global_positioning.cc#L26
 *      2. http://ceres-solver.org/nnls_solving.html#_CPPv4N5ceres17IterationCallbackE
 *
 */
class BundleAdjustmentIterationCallback : public ceres::IterationCallback {
 public:
  BundleAdjustmentIterationCallback(const std::shared_ptr<rerun::RecordingStream> rr_rec,
                                    const std::shared_ptr<rerun::Pinhole> rrpinhole,
                                    const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                                    const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D)
      : rr_rec{rr_rec}, rrpinhole{rrpinhole}, images{images}, points3D{points3D} {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    // -------------------- iteration summary logging
    // NOTE: Instead of logging iteration manually here, make use of default ceres per iteration printer by setting:
    //  solver_options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    // in the main executable

    // -------------------- Log full reconstruction
    rr_rec->set_time_sequence("step", summary.iteration);
    rrfuse::LogReconstruction(rr_rec, rrpinhole, images, points3D);
    return ceres::SOLVER_CONTINUE;
  }

 protected:
  const std::shared_ptr<rerun::RecordingStream> rr_rec;
  const std::shared_ptr<rerun::Pinhole> rrpinhole;
  const std::unordered_map<colmap::camera_t, colmap::Image>& images;
  const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D;
};

/**
 * @brief Ceres iteration callback, called during every iteration of ceres optimization. Derived to log colmap fusion factor graph to rerun.
 * Logs colmap images in sorted order.
 * @ref 1. https://github.com/rerun-io/glomap/blob/main/glomap/estimators/global_positioning.cc#L26
 *      2. http://ceres-solver.org/nnls_solving.html#_CPPv4N5ceres17IterationCallbackE
 *
 */
class FusionIterationCallback : public BundleAdjustmentIterationCallback {
 public:
  FusionIterationCallback(const std::shared_ptr<rerun::RecordingStream> rr_rec,
                          const std::shared_ptr<rerun::Pinhole> rrpinhole,
                          const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                          const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                          const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp,
                          const std::map<const double, fuhe::edges::OdomEdge>& odom_edges,
                          bool is_draw_odom_edges_as_pred_pose = true)
      : BundleAdjustmentIterationCallback(rr_rec, rrpinhole, images, points3D),
        img_ids_by_stamp{img_ids_by_stamp},
        odom_edges{odom_edges},
        is_draw_odom_edges_as_pred_pose{is_draw_odom_edges_as_pred_pose} {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    std::unordered_map<colmap::camera_t, colmap::Image> imgs_sorted_by_stamp;

    rr_rec->set_time_sequence("step", summary.iteration);
    rrfuse::LogReconstructionSorted(this->rr_rec, this->rrpinhole, this->images, this->points3D, this->img_ids_by_stamp);

    if (is_draw_odom_edges_as_pred_pose) {
      // draw external odometry as predicted poses with respect to source camera
      rrfuse::LogOdometryEdges(this->rr_rec, this->images, this->odom_edges);
    } else {
      // draw external odometry as absolute poses
      bool log_odom_trajectory_as_linestrip =
          (summary.iteration > 1) ? false : true;  // draw odometry linestrip only once at first iteration
      rrfuse::LogOdometryEdgesAsTrajectory(this->rr_rec, this->images, this->odom_edges, log_odom_trajectory_as_linestrip);
    }

    return ceres::SOLVER_CONTINUE;
  }

 protected:
  const fuhe::types::MapOfImageIdsSec img_ids_by_stamp;
  const std::map<const double, fuhe::edges::OdomEdge> odom_edges;

  bool is_draw_odom_edges_as_pred_pose = true;  // whether to draw all external odometry measurements as absolute poses or predictes pose
                                                // increments, seen from each colmap pose
};

}  // namespace fuhe

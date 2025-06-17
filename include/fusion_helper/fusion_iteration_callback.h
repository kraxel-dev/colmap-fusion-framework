/**
 * @file fusion_iteration_callback.h
 * @author kraxel
 * @brief Collection of iteration callback classes that can be attached to a ceres optimization process to log the
 * updating states (cam poses + 3D points) in a colmap Bundle Adjstutment to rerun for each iteration. Contains callback classes
 * for vanilla Colmap BA, but also for high level fusion as well as tightly coupled fusion use-cases. Mainly inspired by:
 *      1. https://github.com/rerun-io/glomap/blob/main/glomap/estimators/global_positioning.cc#L26
 *      2. http://ceres-solver.org/nnls_solving.html#_CPPv4N5ceres17IterationCallbackE
 * Note that the permutation of use-cases is convoluted, which is why so many different classes with seemingly similar names
 * exist.
 * I: Iter callback deployable by Colmaps internal BA class: requires correctly populated ba_config and is used in: Full model
 * BA, Full model BA + Fusion, Default Mapping, Fusion Mapping. II: Iter callback deployable by self-built ceres problems (e.g.
 * FusionGraphInterFace class from in high-level-fusion): only requires rr_sfm_logger and is used in: Full model BA, Full model
 * BA + Fusion.
 * @version 0.1
 * @date 2025-06-12
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "fusion_helper/fusion_residuals_tracker.h"
#include "fusion_helper/rr_sfm_logger.h"
#include <ceres/ceres.h>
#include <ceres/iteration_callback.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <rerun.hpp>

namespace fuhe {
namespace iter_callbacks {

////////////////////////////////////////////////////////////////////////////////
// Iteration callbacks deployable by self-built ceres problems
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Iteration callback, called during every iteration of ceres optimization. Derived to log colmap reconstruction
 * pose and point updates during each optim steps to rerun. Works for vanilla colmap models from original colmap repo and can be
 * used when you build your ceres problem manually (e.g. in high-level-fusion). Does not require a self populated ba_config.
 * @ref 1. https://github.com/rerun-io/glomap/blob/main/glomap/estimators/global_positioning.cc#L26
 *      2. http://ceres-solver.org/nnls_solving.html#_CPPv4N5ceres17IterationCallbackE
 *
 */
class BundleAdjustmentIterationCallback : public ceres::IterationCallback {
 public:
  /**
   * @brief Construct a new Bundle Adjustment Iteration Callback object
   *
   * @param rr_sfm_logger Rerun Sfm Logger object that already holds the COLMAP reconstruction that will be logged
   */
  BundleAdjustmentIterationCallback(const std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger)
      : rr_sfm_logger_{rr_sfm_logger} {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
    // -------------------- iteration summary logging
    // NOTE: Instead of logging iteration manually here, make use of default ceres per iteration printer by setting:
    //  solver_options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    // in the main executable

    // -------------------- Log full reconstruction
    rr_sfm_logger_->LogFullReconstruction();
    return ceres::SOLVER_CONTINUE;
  }

 private:
  const std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger_ = nullptr;
};

/**
 * @brief Same as parent but additionally streams odometry edges between COLMAP image poses from a fusion process. Use when you
 * build your ceres problem manually (e.g. in high-level-fusion). Does not require a self populated ba_config.
 */
class FusionGraphIterCallback : public BundleAdjustmentIterationCallback {
 public:
  FusionGraphIterCallback(const std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger,
                          const edges::MapOfImageEdges& fusion_graph_edges,
                          const std::shared_ptr<fuhe::FusionResidualsTracker> res_tracker = nullptr)
      : BundleAdjustmentIterationCallback(rr_sfm_logger),
        fusion_graph_edges_{fusion_graph_edges},
        tracked_residuals{res_tracker} {
    rr_fusion_logger_ = std::make_shared<rr::RerunFusionGraphLogger>(rr_sfm_logger, fusion_graph_edges_);
  }

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    // update model BBox every n-th iter so that 3d points are not omitted during growing scale through fusion
    const int n_update = 7;
    if (summary.iteration % n_update == 0) {
      if (rr_fusion_logger_->RerunOptions().is_ignore_pts_beyond_model_bbox) {
        rr_fusion_logger_->GetSfmLogger()->UpdateModelBBox();
      }
    }

    // --------------------  visualize full reconstruction for this iteration
    rr_fusion_logger_->LogFullReconstruction();

    // -------------------- visualize odometry edges for this iteraten
    rr_fusion_logger_->ClearAllOdometryEdges();

    // -------------------- visualize odometry edges for this iteraten
    if (rr_fusion_logger_->RerunOptions().draw_rerun_odom_as_predicted_poses) {
      // draw external odometry as predicted poses with respect to source camera
      rr_fusion_logger_->LogOdometryEdges();
    } else {
      // draw external odometry as absolute poses
      rr_fusion_logger_->LogOdometryEdgesAsTrajectory();
    }

    // -------------------- track total factor costs in rerun plots if toggled on
    if (this->tracked_residuals) {
      const double reproj_cost = this->tracked_residuals->GetTotalReprojCost();
      // skip total reprojection cost if its way larger than total graph error to avoid rerun plotting problems
      if (reproj_cost < 6 * summary.cost) {
        rr_fusion_logger_->LogTotalFactorCost("reproj", reproj_cost);
      }
      rr_fusion_logger_->LogTotalFactorCost("odom", this->tracked_residuals->GetTotalOdomCost());
      rr_fusion_logger_->LogTotalFactorCost("graph", summary.cost);
    }
    return ceres::SOLVER_CONTINUE;
  }

 protected:
  const edges::MapOfImageEdges& fusion_graph_edges_;
  std::shared_ptr<rr::RerunFusionGraphLogger> rr_fusion_logger_ = nullptr;

  // residuals_tracker exposing sensor factor residuals during optimization. Will be ingroned if received as nullptr
  std::shared_ptr<fuhe::FusionResidualsTracker> tracked_residuals = nullptr;
};

////////////////////////////////////////////////////////////////////////////////
// Iteration callbacks deployable by COLMAPS internal BundleAdjuster classes
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Called during every iteration of ceres optimization, when attached. Written specifically for COLMAP's internal
 * BundleAdjuster classes. Logs a reconstruction's pose and point updates during each optim step of a bundle adjustmen process
 * (local and global BA) to the rerun viewer. Assumes a correctly populated ba_config to only log active imgs and pts. Can log
 * consistently when there are multiple local and global bundle adjustemnts run consecutively (e.g. during incremenal mapping).
 *
 */
class ColmapBundleAdjusterIterCallback : public ceres::IterationCallback {
 public:
  /**
   * @brief Construct a new Colmap Bundle Adjuster Iter Callback object
   *
   * @param rr_sfm_logger Rerun Sfm Logger object that already holds the COLMAP reconstruction that will be logged
   * @param ba_config knows which img and pose ids are part of the current ceres BA proble. Needed to visualize only the subset
   * in rerun considered by BA problem. Requires correctly populated BA config (normaly done by colmap reconstruction process).
   */
  ColmapBundleAdjusterIterCallback(const std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger,
                                   const colmap::BundleAdjustmentConfig* ba_config)
      : rr_sfm_logger_{rr_sfm_logger}, ba_config_{ba_config} {
    // check the provided ba_config
    if (!ba_config_) {
      LOG(ERROR) << "The BundleAdjustment Config you provided seems to be faulty! This iteration callback cannot operate "
                    "correctly in this case!";
    }

    // update COLMAP model bbox to not stream bogus points beyond to rerun
    if (rr_sfm_logger_->RerunOptions().is_ignore_pts_beyond_model_bbox) {
      rr_sfm_logger_->UpdateModelBBox();
    }
  }

  ~ColmapBundleAdjusterIterCallback() override {
    // log updated post-ba reconstruction to rerun
    rr_sfm_logger_->LogFullReconstruction();
    // cleare the cam poses and pts that were active during this BA
    rr_sfm_logger_->ClearActiveBundle();
  }

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
    // update model BBox every n-th iter so that 3d points are not omitted during growing scale through fusion
    const int n_update = 7;
    if (summary.iteration % n_update == 0) {
      if (rr_sfm_logger_->RerunOptions().is_ignore_pts_beyond_model_bbox) {
        rr_sfm_logger_->UpdateModelBBox();
      }
    }

    // log only images and points3D that are active in current BA problem
    rr_sfm_logger_->LogActivBundle(ba_config_);

    return ceres::SOLVER_CONTINUE;
  }

 protected:
  // visualize only a subset in rerun considered by BA problem
  const colmap::BundleAdjustmentConfig* ba_config_ = nullptr;

 private:
  const std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger_;
};

/**
 * @brief Same as parent but additionally streams odometry edges between COLMAP image poses from a fusion process (e.g.
 * incremenal fusion mapping in tightly coupled fusion). Written specifically for tcf's fusion BundleAdjuster classes derived
 * from COLMAP default BundleAadjusters.
 *
 */
class ColmapFusionBAIterCallback : public ColmapBundleAdjusterIterCallback {
 public:
  /**
   * @brief Construct a new Colmap Fusion B A Iter Callback object
   *
   * @param rr_sfm_logger Rerun Sfm Logger object that already holds the COLMAP reconstruction that will be logged
   * @param ba_config knows which img and pose ids are part of the current ceres BA proble. Needed to visualize only the subset
   * in rerun considered by BA problem. Requires correctly populated BA config (normaly done by colmap reconstruction process).
   * @param active_fusion_graph_edges subset of seqeuential image edges wiht active odometry in current BA problem. Make sure to
   * filter properly before passing to this class
   */
  ColmapFusionBAIterCallback(const std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger,
                             const colmap::BundleAdjustmentConfig* ba_config,
                             const edges::MapOfImageEdges& active_fusion_graph_edges)
      : ColmapBundleAdjusterIterCallback(rr_sfm_logger, ba_config), active_fusion_edges_{active_fusion_graph_edges} {
    // instantiate a rerun fusion logger object that can also stream the odom edges to rerun
    rr_fusion_logger_ = std::make_shared<rr::RerunFusionGraphLogger>(rr_sfm_logger, active_fusion_edges_);
  }
  ~ColmapFusionBAIterCallback() override { rr_fusion_logger_->ClearAllOdometryEdges(); }
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
    // -------------------- Log active imgs and pts in current BA

    // when a populated BA config knows which images and points are considered for this BA problem, log only those
    rr_fusion_logger_->LogActivBundle(ba_config_);

    // -------------------- visualize odometry edges for this iteraten
    if (rr_fusion_logger_->RerunOptions().draw_rerun_odom_as_predicted_poses) {
      // draw external odometry as predicted poses with respect to source camera
      rr_fusion_logger_->LogOdometryEdges();
    } else {
      // draw external odometry as absolute poses
      rr_fusion_logger_->LogOdometryEdgesAsTrajectory();
    }

    return ceres::SOLVER_CONTINUE;
  }

 protected:
  // subset of seqeuential image edges wiht active odometry in current BA problem. Make sure to filter properly before passing to
  // this class
  const edges::MapOfImageEdges& active_fusion_edges_;

  std::shared_ptr<rr::RerunFusionGraphLogger> rr_fusion_logger_ = nullptr;
};

}  // namespace iter_callbacks
}  // namespace fuhe

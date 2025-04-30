/**
 * @file bundle_adjustment.cc
 * @author kraxel
 * @brief TODO: fill brief
 * @ref (original colmap repo) src/colmap/estimators/bundle_adjustment.cc
 * @version 0.1
 * @date 2025-03-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "tightly_coupled_fusion/estimators/bundle_adjustment.h"

#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/manifold.h>
#include <fusion_helper/col_utils.h>
#include <fusion_helper/io.h>
#include <fusion_helper/odom_edges_manager.h>
#include <fusion_helper/stream_utils.h>
#include <fusion_helper/types.h>

namespace tfc {  // namespace tfc

////////////////////////////////////////////////////////////////////////////////
// Default Bundle Adjuster with Rerun Logging
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Exact same behavior as DefaultBundelAdjuster but with rerun logging during optimization. Can be used in both local and global BA
 * of colmaps native (standard behavior) incremental mapping process
 *
 */
class DefaultBundleAdjusterRerunLogging : public colmap::BundleAdjuster {
 public:
  /**
   * @brief Construct a new Default Bundle Adjuster Rerun Logging object
   *
   * @param options
   * @param config
   * @param reconstruction
   * @param rr_recorder
   */
  DefaultBundleAdjusterRerunLogging(colmap::BundleAdjustmentOptions options,
                                    colmap::BundleAdjustmentConfig config,
                                    colmap::Reconstruction& reconstruction,
                                    const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder)
      : BundleAdjuster(std::move(options), std::move(config)), reconstruction_{reconstruction}, rr_recorder_{rr_recorder} {
    default_bundle_adjuster_ = colmap::CreateDefaultBundleAdjuster(options_, config_, reconstruction);
  }

  /**
   * @brief same behavior as default bundle adjuster, but with iteration callback for rerun logging during optim
   *
   * @return ceres::Solver::Summary
   */
  ceres::Solver::Summary Solve() override {
    ceres::Solver::Summary summary;
    if (default_bundle_adjuster_->Problem()->NumResiduals() == 0) {
      return summary;
    }

    ceres::Solver::Options solver_options = options_.CreateSolverOptions(config_, *default_bundle_adjuster_->Problem());

    // attach iteration callback for rerun visualization of iterations if toggled
    if (rr_recorder_->GetRerunRec()) {
      VLOG(2) << "Attaching iteration callback to log data to rerun during optimization!";
      this->AddIterationCallbackRerunLogging(solver_options);
    }

    ceres::Solve(solver_options, default_bundle_adjuster_->Problem().get(), &summary);

    if (options_.print_summary || VLOG_IS_ON(1)) {
      colmap::PrintSolverSummary(summary, "Bundle adjustment report");
    }

    return summary;
  }

  /**
   * @brief
   *
   * @return std::shared_ptr<ceres::Problem>&
   */
  std::shared_ptr<ceres::Problem>& Problem() override { return default_bundle_adjuster_->Problem(); }

 protected:
  /// Attach iteration callback that logs visualization data to rerun during optimization
  void AddIterationCallbackRerunLogging(ceres::Solver::Options& solver_options) {
    VLOG(2) << "Deploying rerun iteration callback!";

    // deploy own iteration callback that logs to rerun during optimization
    iter_callback_ =
        std::make_shared<fuhe::MarathonBundleAdjustIterCallback>(rr_recorder_,
                                                                 reconstruction_.Images(),
                                                                 reconstruction_.Points3D(),
                                                                 &config_);  // pass ba_config to only log subset of images and points

    // only with real value updates can rerun log during the optimization process
    solver_options.update_state_every_iteration = true;
    solver_options.callbacks.push_back(iter_callback_.get());
  }

  std::unique_ptr<colmap::BundleAdjuster> default_bundle_adjuster_ = nullptr;
  std::shared_ptr<fuhe::BundleAdjustmentIterationCallback> iter_callback_ = nullptr;
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder_ = nullptr;
  colmap::Reconstruction& reconstruction_;
};

////////////////////////////////////////////////////////////////////////////////
// Fusion Bundle Adjuster
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Fusion Bundle Adjuster class that builds and solves bundle adjustment problem with odometry data. Can be used in both local and
 * global BA steps of colmaps incremental mapping steps. Full declaration and definiton is scopred to this .cc to adept native colmaps
 * implementation style.
 *
 */
class FusionGraphBundleAdjuster : public colmap::BundleAdjuster {
 public:
  FusionGraphBundleAdjuster(colmap::BundleAdjustmentOptions options,
                            const tcf::FusionGraphBundleAdjustmentOptions& fusion_options,
                            const fuhe::rrfuse::RerunFusionVisOptions& rr_options,
                            const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder,
                            colmap::BundleAdjustmentConfig config,
                            colmap::Reconstruction& reconstruction,
                            const fuhe::edges::MapOfImageEdges& fusion_graph_data_edges)
      : BundleAdjuster(std::move(options), std::move(config)),
        fusion_options_{fusion_options},
        rr_options_{rr_options},
        rr_rec_{rr_recorder},
        reconstruction_{reconstruction} {
    // -------------------- Default Bundle Adjuster creation
    default_bundle_adjuster_ = colmap::CreateDefaultBundleAdjuster(options_, config_, reconstruction_);

    // -------------------- Filter graph data edges (img to img)
    // keep only images edges with odometry that are active part of current BA. Note that ba_cfg must be correctly populated before this
    // call
    active_fusion_graph_edges_ = fuhe::edges::SubsetActiveEdges(config_, fusion_graph_data_edges);

    // -------------------- Iterate over fusion graph edges to build factor graph problem
    VLOG(2) << "Adding between factors to fusion graph!";
    int between_factor_counter = 0;

    // iterate over active image nodes that hold valid odometry edges for current BA setup
    for (const std::pair<const double, fuhe::edges::SequentialImageEdge>& pair : active_fusion_graph_edges_) {
      const fuhe::edges::SequentialImageEdge& img_edge = pair.second;
      const double curr_img_stamp = pair.first;

      const colmap::image_t curr_img_id = img_edge.CurrId();  // node j
      VLOG(3) << "Iteration for image: " << curr_img_id << " of stamp " << curr_img_stamp;

      // Define covariance of relative motion.
      Eigen::Matrix<double, 6, 6> covarince_i_from_j = img_edge.OdomEdge()->CovMat_ij() * fusion_options_.cov;
      //   // weight non local z-axis motion and rotation in relative odometry // TODO: think about motion axis weighting
      //   fuhe::cov_utils::WeightPoseCovNonMotionDirection(covarince_i_from_j, non_motion_weighting);

      // register metric relative pose factor in BA
      this->AddOdomToProblem(
          img_edge.OdomEdge()->PrevId(), img_edge.OdomEdge()->CurrId(), img_edge.OdomEdge()->T_i_from_j(), covarince_i_from_j);
      between_factor_counter++;

      // TODO: think deeply about const images and scale normalization
      // if (use_prior_position) {
      //   // Normalize the reconstruction to avoid any numerical instability but do
      //   // not transform priors as they will be transformed when added to
      //   // ceres::Problem.
      //   normalized_from_metric_ = reconstruction_.Normalize(/*fixed_scale=*/true);

      //   if (prior_options_.use_robust_loss_on_prior_position) {
      //     prior_loss_function_ = std::make_unique<ceres::CauchyLoss>(prior_options_.prior_position_loss_scale);
      //   }

      //   for (const image_t image_id : config_.Images()) {
      //     const auto pose_prior_it = pose_priors_.find(image_id);
      //     if (pose_prior_it != pose_priors_.end()) {
      //       AddPosePriorToProblem(image_id, pose_prior_it->second, reconstruction);
      //     }
      //   }
      // }
    }
    VLOG(2) << "Added nr of between factors: " << between_factor_counter;

    // fix cam poses deemed as constant for this bundle to static. Necessary since default BA does not actually set the ceres param block as
    // constant during init.
    for (const colmap::image_t image_id : config_.Images()) {
      if (!(config_.HasConstantCamPose(image_id))) {
        continue;
      }

      VLOG(2) << "Forcing pose parameter block of image " << image_id << " to constant!";
      auto& img_const_pose = reconstruction_.Image(image_id);
      // recover image pose from colmap model
      double *q = nullptr, *t = nullptr;  // ceres param blocks for pose
      fuhe::col_utils::GetPointersToPose(img_const_pose, q, t);
      // Check existence of param block (in some cases like image only BA, the parameter block of a const pose might be not registered in
      // the problem).
      if (!(default_bundle_adjuster_->Problem()->HasParameterBlock(q) && default_bundle_adjuster_->Problem()->HasParameterBlock(t))) {
        continue;
      }
      default_bundle_adjuster_->Problem()->SetParameterBlockConstant(q);
      default_bundle_adjuster_->Problem()->SetParameterBlockConstant(t);
    }
  }

  /// Add relative odometry factor to ceres problem
  void AddOdomToProblem(const colmap::image_t img_id_i,
                        const colmap::image_t img_id_j,
                        const Eigen::Isometry3d& i_from_j,
                        const Eigen::Matrix<double, 6, 6> cov_i_from_j) {
    // -------------------- Recover pointers to image poses from colmap model
    colmap::Image& img_i = reconstruction_.Image(img_id_i);  // prev image
    colmap::Image& img_j = reconstruction_.Image(img_id_j);  // curr image

    // recover image pose from colmap model
    double *q_i = nullptr, *t_i = nullptr, *q_j = nullptr, *t_j = nullptr;  // pointers to pose params of both images
    fuhe::col_utils::GetPointersToPose(img_i, q_i, t_i);
    fuhe::col_utils::GetPointersToPose(img_j, q_j, t_j);

    // -------------------- Create between factor and add to problem
    // convert raltive eigen pose to colmap format
    const colmap::Rigid3d T_ij_rigid = colmap::Rigid3d(Eigen::Quaterniond(i_from_j.rotation()), i_from_j.translation());

    VLOG(3) << "Creating metric relative odom cost function from img id: " << img_id_i << " to id: " << img_id_j;
    // create ceres relative pose factor weighted by its covariance
    ceres::CostFunction* cost_func =
        colmap::CovarianceWeightedCostFunctor<colmap::RelativePosePriorCostFunctor>::Create(cov_i_from_j, T_ij_rigid);
    VLOG(3) << "Adding residual block to ceres graph!";
    // register odom between factor in ceres graph and directly retrieve id of residual block. order of params: q_i, t_i, q_j, t_j

    this->Problem()->AddResidualBlock(cost_func, nullptr, q_i, t_i, q_j, t_j);

    // // -------------------- Double check if pose parameters are registered as manifold in optimizaion
    // Set Lie algebra for pose on manifold optimization in case it was not set already
    if (!default_bundle_adjuster_->Problem()->GetParameterization(q_i)) {
      colmap::SetQuaternionManifold(default_bundle_adjuster_->Problem().get(), q_i);
    }
    if (!default_bundle_adjuster_->Problem()->GetParameterization(q_j)) {
      colmap::SetQuaternionManifold(default_bundle_adjuster_->Problem().get(), q_j);
    }
  }

  /// Obtain ceres problem
  std::shared_ptr<ceres::Problem>& Problem() override { return default_bundle_adjuster_->Problem(); };

  /// Solve fusion graph bundle adjustment problem
  ceres::Solver::Summary Solve() override {
    ceres::Solver::Summary summary;
    std::shared_ptr<ceres::Problem> problem = default_bundle_adjuster_->Problem();
    if (problem->NumResiduals() == 0) {
      return summary;
    }

    ceres::Solver::Options solver_options = options_.CreateSolverOptions(config_, *problem);

    // attach iteration callback for rerun visualization of iterations if toggled
    if (rr_rec_->GetRerunRec()) {
      VLOG(2) << "Attaching fusion iteration callback to log data to rerun during optimization!";
      this->AddFusionIterationCallback(solver_options);
    }

    ceres::Solve(solver_options, problem.get(), &summary);

    if (options_.print_summary || VLOG_IS_ON(1)) {
      colmap::PrintSolverSummary(summary, "Fusion Graph Bundle adjustment report");
    }

    return summary;
  };

 protected:
  const tcf::FusionGraphBundleAdjustmentOptions fusion_options_;  // tum file path, cov mat, etc
  const fuhe::rrfuse::RerunFusionVisOptions rr_options_;          // rerun visualization options
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_rec_ = nullptr;
  std::shared_ptr<fuhe::MarathonFusionIterCallback> iter_callback_ = nullptr;  // custom iteration callback to log data to rerun if toggled

  const std::unordered_map<colmap::image_t, colmap::PosePrior> rel_poses_;  // FIXME: kick if not needed
  colmap::Sim3d normalized_from_metric_;                                    // TODO: kick if not needed

  fuhe::edges::MapOfImageEdges
      active_fusion_graph_edges_;  // time sorted image sequence with odometry edges constraining images (if availabe)
  colmap::Reconstruction& reconstruction_;

  std::unique_ptr<colmap::BundleAdjuster> default_bundle_adjuster_;
  std::unique_ptr<ceres::LossFunction> prior_loss_function_;  // FIXME: decide to kick

  /// Attach iteration callback that logs visualization data to rerun during optimization
  void AddFusionIterationCallback(ceres::Solver::Options& solver_options) {
    VLOG(2) << "Deploying rerun iteration callback!";
    // deploy own iteration callback that logs to rerun during optimization
    iter_callback_ = std::make_shared<fuhe::MarathonFusionIterCallback>(rr_rec_,
                                                                        reconstruction_.Images(),
                                                                        reconstruction_.Points3D(),
                                                                        &config_,
                                                                        active_fusion_graph_edges_,
                                                                        rr_options_.draw_rerun_odom_as_predicted_poses,
                                                                        rr_options_.is_highlight_active_cams);
    // only with real value updates can rerun log during the optimization process
    solver_options.update_state_every_iteration = true;

    solver_options.callbacks.push_back(iter_callback_.get());
  }
};
}  // namespace tfc

std::unique_ptr<colmap::BundleAdjuster> tcf::CreateFusionGraphBundleAdjuster(
    colmap::BundleAdjustmentOptions options,
    const tcf::FusionGraphBundleAdjustmentOptions& fusion_options,
    const fuhe::rrfuse::RerunFusionVisOptions& rr_options,
    const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder,
    colmap::BundleAdjustmentConfig config,
    colmap::Reconstruction& reconstruction,
    const fuhe::edges::MapOfImageEdges& fusion_graph_data_edges) {
  return std::make_unique<tfc::FusionGraphBundleAdjuster>(
      std::move(options), fusion_options, rr_options, rr_recorder, std::move(config), reconstruction, fusion_graph_data_edges);
}

std::unique_ptr<colmap::BundleAdjuster> tcf::CreateDefaultBundleAdjusterRerun(
    colmap::BundleAdjustmentOptions options,
    colmap::BundleAdjustmentConfig config,
    colmap::Reconstruction& reconstruction,
    const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder) {
  return std::make_unique<tfc::DefaultBundleAdjusterRerunLogging>(std::move(options), std::move(config), reconstruction, rr_recorder);
}

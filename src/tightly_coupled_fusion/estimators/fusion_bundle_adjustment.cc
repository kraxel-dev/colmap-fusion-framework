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

#include "tightly_coupled_fusion/estimators/fusion_bundle_adjustment.h"

#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/manifold.h>
#include <fusion_helper/col_utils.h>
#include <fusion_helper/io.h>
#include <fusion_helper/odom_edges_manager.h>
#include <fusion_helper/stream_utils.h>
#include <fusion_helper/types.h>

namespace tfc {}  // namespace tfc

tcf::FusionGraphBundleAdjuster::FusionGraphBundleAdjuster(colmap::BundleAdjustmentOptions options,
                                                          const tcf::FusionGraphBundleAdjustmentOptions& fusion_options,
                                                          const fuhe::rrfuse::RerunFusionVisOptions& rr_options,
                                                          colmap::BundleAdjustmentConfig config,
                                                          colmap::Reconstruction& reconstruction)
    : BundleAdjuster(std::move(options), std::move(config)),
      fusion_options_(fusion_options),
      rr_options_(rr_options),
      reconstruction_(reconstruction) {
  // -------------------- Rerun logging stuff
  if (rr_options_.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    rr_rec_ = std::make_shared<fuhe::rrfuse::RerunFusionRecorder>(rr_options_, reconstruction_);
  }

  // -------------------- Default Bundle Adjuster creation
  default_bundle_adjuster_ = std::make_unique<colmap::DefaultBundleAdjuster>(options_, config_, reconstruction);

  // -------------------- Odometry graph data edges (img to img)
  fuhe::types::MapOfPosesSec metric_poses;  // absolute poses from external odom sensor sorted by stamps
  fuhe::io::TumToPosesEigen(fusion_options_.tum_file, metric_poses, true);

  // obtain all images from model
  const std::set<colmap::image_t>& reg_image_ids = reconstruction_.RegImageIds();
  // get image ids in sorted order
  imgs_by_stamp_ = std::make_shared<fuhe::types::MapOfImageIdsSec>(fuhe::col_utils::ImageIdsByStamp(reg_image_ids, reconstruction_));

  // data structure holding odometry rel poses between associated images
  fusion_graph_data_edges_ = fuhe::edges::OdomEdgesManager::CreateOdomEdgesBetweenImagesPtr(*imgs_by_stamp_, metric_poses);

  // -------------------- Iterate over fusion graph edges to build factor graph problem
  VLOG(2) << "Iterating over all odometry edges to add between factor to fusion graph!";
  double curr_img_stamp, prev_stamp = -1;  // stamps for successfully utilized external odoms

  // iterate over all sequential image-odometry edges in model
  for (const std::pair<const double, fuhe::edges::OdomEdge>& pair : *fusion_graph_data_edges_) {
    // data stuff
    const fuhe::edges::OdomEdge edge = pair.second;
    curr_img_stamp = pair.first;

    const colmap::image_t curr_img_id = edge.j;
    const colmap::image_t prev_img_id = edge.i;
    VLOG(3) << "Iteration for image: " << curr_img_id << " of stamp " << curr_img_stamp;

    // -------------------- First iteration init condition
    if (edge.i == edge.j) {
      VLOG(2) << "Origin image node detected! Kickoff fusion graph construction";

      // preparing next iteration
      prev_stamp = curr_img_stamp;
      continue;
    }
    // check if external odom is available for current image
    if (!edge.T_odom_ij_ptr) {
      continue;
    }
    //   // -------------------- Add relative odometry factor
    VLOG(3) << "Found matching tumposes to use as realtive pose factor!";

    // Get metric relative  pose of j (curr) expressed in i (prev)
    const Eigen::Isometry3d T_i_from_j(edge.T_odom_ij_ptr->ToMatrix());
    VLOG(4) << "Relataive motion is: " << T_i_from_j;

    // Define covariance of relative motion.
    Eigen::Matrix<double, 6, 6> covarince_i_from_j = Eigen::Matrix<double, 6, 6>::Identity() * fusion_options_.cov;
    //   // weight non local z-axis motion and rotation in relative odometry // TODO: think about motion axis weighting
    //   fuhe::cov_utils::WeightPoseCovNonMotionDirection(covarince_i_from_j, non_motion_weighting);

    // -------------------- Inlcude metric relative pose factor in BA
    this->AddOdomToProblem(prev_img_id, curr_img_id, T_i_from_j, covarince_i_from_j);

    // preparing next iteration
    prev_stamp = curr_img_stamp;

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
}

void tcf::FusionGraphBundleAdjuster::AddOdomToProblem(const colmap::image_t img_id_i,
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

  // -------------------- Double check if pose parameters are registered as manifold in optimizaion
  // Set Lie algebra for pose on manifold optimization in case it was not set already
  if (!default_bundle_adjuster_->Problem()->GetParameterization(q_i)) {
    colmap::SetQuaternionManifold(default_bundle_adjuster_->Problem().get(), q_i);
  }
  if (!default_bundle_adjuster_->Problem()->GetParameterization(q_j)) {
    colmap::SetQuaternionManifold(default_bundle_adjuster_->Problem().get(), q_j);
  }
}

std::shared_ptr<ceres::Problem>& tcf::FusionGraphBundleAdjuster::Problem() { return default_bundle_adjuster_->Problem(); }

ceres::Solver::Summary tcf::FusionGraphBundleAdjuster::Solve() {
  ceres::Solver::Summary summary;
  std::shared_ptr<ceres::Problem> problem = default_bundle_adjuster_->Problem();
  if (problem->NumResiduals() == 0) {
    return summary;
  }

  ceres::Solver::Options solver_options = options_.CreateSolverOptions(config_, *problem);

  if (rr_rec_->GetRerunRec()) {
    VLOG(2) << "Attaching fusion iteration callback to log data to rerun during optimization!";
    // attach iteration callback for rerun visualization of iterations if toggled
    this->AddFusionIterationCallback(solver_options);
  }

  ceres::Solve(solver_options, problem.get(), &summary);

  if (options_.print_summary || VLOG_IS_ON(1)) {
    colmap::PrintSolverSummary(summary, "Fusion Graph Bundle adjustment report");
  }

  return summary;
}

void tcf::FusionGraphBundleAdjuster::AddFusionIterationCallback(ceres::Solver::Options& solver_options) {
  VLOG(2) << "Deploying rerun iteration callback!";
  // deploy own iteration callback that logs to rerun during optimization
  iter_callback_ = std::make_shared<fuhe::FusionIterationCallback>(rr_rec_->GetRerunRec(),
                                                                   rr_rec_->GetRerunPinhole(),
                                                                   reconstruction_.Images(),
                                                                   reconstruction_.Points3D(),
                                                                   *fusion_graph_data_edges_,
                                                                   rr_options_.draw_rerun_odom_as_predicted_poses);
  // only with real value updates can rerun log during the optimization process
  solver_options.update_state_every_iteration = true;

  solver_options.callbacks.push_back(iter_callback_.get());
}

/**
 * @file metric_odom_bundle_adjust.cpp
 * @author kraxel
 * @brief Showcase of high-level-fusion between a fully constructed colmap model and relative pose constraints from an external
 odometry source. Iterates through registered colmap images in time ascending order and adds reprojection factors as well as
 relative pose factors between images to a ceres optimization problem.

 Inputs:
 1. Colmap model, 2. external tum file of odometry with asbolute poses. Optimization prolem entails ALL images of reconstruction,
 checking validity of src and destination img id of an odom edge is not necessary.

 Options:
 BundleAdjustment options can be passed as command line arguments exactly as in vanilla colmap exes. Use
 ./metric_odom_bundle_adjust.cpp -h to see all available options. Options to refine extra params (cam intrinsics) are forcefully
 ignored in this sample.

 Visualization:
 Will log the optimization process to rerun for visualization purposes. Has wonky feature to log factor costs (by
 category) during optimization to rerun.

 * @version 0.1
 * @date 2025-01-27
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <ostream>

#include "fusion_helper/ceres_eval_utils.h"
#include "fusion_helper/col_utils.h"
#include "fusion_helper/fusion_evaluation_callback.h"
#include "fusion_helper/fusion_iteration_callback.h"
#include "fusion_helper/io.h"
#include "fusion_helper/odom_edges.h"
#include "fusion_helper/stream_utils.h"
#include "high_level_fusion/fusion_graph_interface.h"
#include <Eigen/Core>
#include <ceres/problem.h>
#include <colmap/controllers/option_manager.h>
#include <colmap/estimators/coordinate_frame.h>
#include <colmap/util/file.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  // -------------------- Parse COLMAP and Ceres inputs
  std::string input_path;
  std::string output_path;
  // whether to align model with PCA before optimization (e.g. for better visualization)
  bool pca_align = true;
  fuhe::cov_utils::OdomCovOptions cov_options;     // covariance options for relative odometry measurements
  fuhe::rr::RerunVisualizationOptions rr_options;  // rerun visualization options
  hifuse::HighLevelFusionOptions fusion_options;   // high level fusion option

  colmap::OptionManager col_options;

  // arguments for performing high level fusion
  col_options.AddRequiredOption("input_path", &input_path);
  col_options.AddRequiredOption("output_path", &output_path);
  // Fusion options
  col_options.AddRequiredOption("Fusion.tum_file", &fusion_options.tum_file);
  col_options.AddDefaultOption("Fusion.track_residuals", &fusion_options.track_residuals);

  col_options.AddDefaultOption("Model.pca_align", &pca_align);

  // Rerun visualization options
  // whether to log data to rerun viewer
  col_options.AddDefaultOption("Rerun.log", &rr_options.is_log_to_rerun);
  // whether to save logged rerun data to rr file
  col_options.AddDefaultOption("Rerun.save_rrd", &rr_options.is_save_rerun_to_disk);
  // whether to draw external odometry as predicted poses with respect to source camera or as absolute poses
  col_options.AddDefaultOption("Rerun.odom_as_pred", &rr_options.draw_rerun_odom_as_predicted_poses);

  // Odom covariance options
  // std values per Secs
  col_options.AddDefaultOption("OdomCov.tx_std", &cov_options.std_tx_per_s);
  col_options.AddDefaultOption("OdomCov.ty_std", &cov_options.std_ty_per_s);
  col_options.AddDefaultOption("OdomCov.tz_std", &cov_options.std_tz_per_s);
  col_options.AddDefaultOption("OdomCov.rx_std", &cov_options.std_rx_per_s);
  col_options.AddDefaultOption("OdomCov.ry_std", &cov_options.std_ry_per_s);
  col_options.AddDefaultOption("OdomCov.rz_std", &cov_options.std_rz_per_s);

  // add vanilla colmap bundle adjustment options that will be used as solver settings
  col_options.AddBundleAdjustmentOptions();
  col_options.Parse(argc, argv);

  // -------------------- Logging stuff
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;

  // Set log directory
  const std::string log_dir = fuhe::io::GetRepoRootDir() + "/logs";
  FLAGS_log_dir = log_dir;
  VLOG(3) << "Logging path is: " << log_dir;

  google::InitGoogleLogging(argv[0]);

  // -------------------- check directoreis
  if (!colmap::ExistsDir(input_path)) {
    LOG(ERROR) << "`input_path` is not a directory";
    return EXIT_FAILURE;
  }

  if (!colmap::ExistsDir(output_path)) {
    LOG(ERROR) << "`output_path` is not a directory";
    return EXIT_FAILURE;
  }

  if (fusion_options.tum_file.empty() || !colmap::ExistsFile(fusion_options.tum_file)) {
    LOG(ERROR) << "`fusion_options.tum_file` is empty or does not exist";
    return EXIT_FAILURE;
  }

  // -------------------- Read TUM file
  fuhe::types::MapOfPosesSec metric_poses;  // absolute poses from external odom sensor sorted by stamps
  fuhe::io::TumToPosesEigen(fusion_options.tum_file, metric_poses, /*cut_precision=*/true);

  // -------------------- Read COLMAP model
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(input_path);

  // align model to groundplane via PCA if toggled (better rerun visualization)
  if (pca_align) {
    VLOG(1) << "Aligning model through PCA before optimization!";
    colmap::Sim3d align;
    colmap::AlignToPrincipalPlane(reconstruction.get(), &align);
  } else {
    VLOG(1) << "Skipping model PCA alignment before optimization!";
  }

  // obtain all images from model in time-ascending order
  const std::set<colmap::image_t>& reg_image_ids = reconstruction->RegImageIds();
  auto imgs_by_stamp = fuhe::col_utils::ImageIdsByStamp(reconstruction->Images());

  // -------------------- Create directed odom edges between images in sorted order
  // covariance manager for relative odometry measurements
  std::shared_ptr<fuhe::cov_utils::TimeScaledOdomCovManager> cov_manager = std::make_shared<fuhe::cov_utils::TimeScaledOdomCovManager>(cov_options);
  // main data structure that we will iterate over to construct the fusion problem. Contains the image ids of
  // the colmap model in time ascending order. Most importantly this associates the absolute odom poses from the tum file to the
  // constraining image pairs as relative edge (which can be used by the interface as relative pose factor).
  auto data_graph_edges = fuhe::edges::CreateSequentialImageEdges(imgs_by_stamp, metric_poses, *cov_manager);

  // -------------------- Create Ceres problem
  // NOTE: keep ceres problem as non-pointer and only pass as reference to avoid double free issues
  ceres::Problem ceres_problem;
  // keep track of imgs added to problem such that appropiate colmap solver options can be generated. Besides that it does not
  // influence the system in any way
  colmap::BundleAdjustmentConfig ba_config;

  // -------------------- Create fusion interface object
  // this acts as managing object to construct the fusion graph ceres problem
  hifuse::FusionGraphInterface fusion_interface(reconstruction, ceres_problem, fusion_options.track_residuals);

  // streaming sfm and fusion bundle adjustment process to rerun
  std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_sfm_logger = nullptr;
  if (rr_options.is_log_to_rerun) {
    VLOG(1) << "Rerun recording toggled. Instantiating sfm logger!";
    rr_sfm_logger = std::make_shared<fuhe::rr::RerunSfmLogger>(rr_options, reconstruction);

    // establish model bbox from 3d points of initial pair to filter out pts in rerun that would cause mayhem in the viewer
    if (rr_options.is_ignore_pts_beyond_model_bbox) {
      rr_sfm_logger->UpdateModelBBox();
    }

    rr_sfm_logger->LogFullReconstruction();
  }

  // -------------------- Iterate over COLMAP model to build factor graph problem
  double curr_img_stamp, prev_stamp = -1;  // stamps for successfully utilized external odoms

  // iterate over all sequential image edges in model
  for (const std::pair<const double, fuhe::edges::SequentialImageEdge>& pair : data_graph_edges) {
    const fuhe::edges::SequentialImageEdge image_edge = pair.second;
    curr_img_stamp = pair.first;

    const colmap::image_t curr_img_id = image_edge.CurrId();
    const colmap::image_t prev_img_id = image_edge.PrevId();
    VLOG(2) << "Iteration for image: " << curr_img_id << " of stamp " << curr_img_stamp;

    // -------------------- First iteration init condition
    if (image_edge.IsSourceNode()) {
      VLOG(1) << "Origin image node detected! Kickoff factor graph construction";
      // add very first image to bundle adjustment and force constant position but variable 3d pts to lock (some parts of) gauge
      // freedom
      fusion_interface.AddReprojectionFactors(curr_img_id, /*const_t=*/true, /*const_q=*/true, /*const_3d_pts=*/false);
      // notify that id was added for solver options later on
      ba_config.AddImage(curr_img_id);

      // preparing next iteration
      prev_stamp = curr_img_stamp;
      continue;
    }

    // -------------------- Add BA and rel pose factors to graph
    // add image to bundle adjustment with variable pose and 3d points
    fusion_interface.AddReprojectionFactors(curr_img_id, /*const_t=*/false, /*const_q=*/false, /*const_3d_pts=*/false);
    // notify that id was added for solver options later on
    ba_config.AddImage(curr_img_id);

    // check if external odom is available for current image
    if (!image_edge.OdomEdge()) {
      continue;
    }

    // -------------------- Prepare relative odometry factor
    VLOG(2) << "Found matching tumposes to use as realtive pose factor!";

    // Get metric relative  pose of j (curr) expressed in i (prev)
    const Eigen::Isometry3d T_i_from_j(image_edge.OdomEdge()->T_i_from_j());
    VLOG(4) << "Relataive motion is: " << T_i_from_j;

    // -------------------- Inlcude metric relative pose factor in BA
    // colmap-id-i, colmap-id-j, measured pose of j expressed in i, covariance of relative pose
    fusion_interface.AddBetweenFactor(
        image_edge.OdomEdge()->PrevId(), image_edge.OdomEdge()->CurrId(), T_i_from_j, image_edge.OdomEdge()->CovMat_ij());

    // preparing next iteration
    prev_stamp = curr_img_stamp;
  }

  // -------------------- Configure Bundle Adjustment for CERES and COLMAP
  ceres::Problem::Options ceres_options;  // ceres options

  // -------------------- Set ceres solver options
  // create ceres solver options the same way that colmap would do (e.g. sparse vs dense depending on nr of resiudals)
  ceres::Solver::Options solver_options = col_options.bundle_adjustment->CreateSolverOptions(ba_config, ceres_problem);
  solver_options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;

  // -------------------- rerun iteration callback during ceres optim
  // deploy own iteration callback that logs pts and poses to rerun during optimization
  std::shared_ptr<fuhe::iter_callbacks::FusionGraphIterCallback> callback = nullptr;
  if (rr_options.is_log_to_rerun) {
    VLOG(2) << "Deploying rerun iteration callback!";
    callback = std::make_shared<fuhe::iter_callbacks::FusionGraphIterCallback>(
        rr_sfm_logger, data_graph_edges, fusion_interface.GetResidualsTracker());
    solver_options.callbacks.push_back(callback.get());
  }

  // -------------------- residuals tracking during ceres optim
  // deply residual cost tracking by category during optimization
  std::shared_ptr<fuhe::FusionEvaluationCallback> fusion_eval_callback = nullptr;
  if (fusion_interface.GetResidualsTracker()) {
    VLOG(2) << "Deploying residual tracking evaluation callback!";
    fusion_eval_callback = std::make_shared<fuhe::FusionEvaluationCallback>(fusion_interface.GetResidualsTracker());
    solver_options.evaluation_callback = fusion_eval_callback.get();
  }

  solver_options.minimizer_progress_to_stdout = false;
  // update colmap poses for every iteration such that rerun can log them during iteration callbacks
  solver_options.update_state_every_iteration = true;

  // -------------------- Evaluate errors before optimization
  fuhe::ceres_eval_utils::CeresCostEvaluator cost_evaluator(
      ceres_problem, fusion_interface.GetReprojResidualIds(), fusion_interface.GetOdomResidualIds());
  if (VLOG_IS_ON(3)) {
    VLOG(3) << "Starting to calc absolute error in graph before optimization!";
    cost_evaluator.CalcTotalOdomCost();
    cost_evaluator.CalcTotalReprojectionCost();
  }

  // -------------------- Perform Bundle Adjustment
  VLOG(1) << "Starting to solve graph!";
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &ceres_problem, &summary);

  // --------------------Metrics after optimization
  if (VLOG_IS_ON(3)) {
    VLOG(3) << "Calc absolute error in graph after optimization!";
    cost_evaluator.CalcTotalOdomCost();
    cost_evaluator.CalcTotalReprojectionCost();
  }

  reconstruction->WriteText(output_path);
  reconstruction->TearDown();

  VLOG(1) << summary.FullReport();
}
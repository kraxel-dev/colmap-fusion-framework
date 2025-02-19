// TODO: write brief

#include <ostream>
#include <thread>

#include "fusion_helper/col_utils.h"
#include "fusion_helper/cov_utils.h"
#include "fusion_helper/fusion_iteration_callback.h"
#include "fusion_helper/io.h"
#include "fusion_helper/odom_edges_manager.h"
#include "fusion_helper/stream_utils.h"
#include "high_level_fusion/fusion_graph_interface.h"
#include <Eigen/Core>
#include <ceres/problem.h>
#include <colmap/controllers/option_manager.h>
#include <colmap/util/file.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  // -------------------- Parse COLMAP and Ceres inputs
  std::string input_path;
  std::string output_path;
  int max_consecutive_nonmonotonic_steps = 0;
  double cov = 1;                   // certainty for relative odometry. The smaller the stronger relative odometry is considered
  double non_motion_weighting = 1;  // weight for non-motion directions in relative odometry covariance
  bool log_to_rerun = true;         // whether to log data to rerun viewer
  bool save_rerun_rec = false;      // whether to save logged rerun data to rr file

  colmap::OptionManager col_options;

  col_options.AddRequiredOption("input_path", &input_path);
  col_options.AddRequiredOption("output_path", &output_path);
  col_options.AddDefaultOption("cov", &cov);
  col_options.AddDefaultOption("non_motion_weighting", &non_motion_weighting);
  col_options.AddDefaultOption("rerun", &log_to_rerun);
  col_options.AddDefaultOption("save_rrd", &save_rerun_rec);
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
  // -------------------- Read TUM file
  // HACK: make parametrizable
  std::string tumFile =
      "/home/azuo/transfer/eval/backwards/"
      "vehicle_wo_as_campose_training_matched_stamps.tum";

  fuhe::types::MapOfPosesSec metric_poses;  // absolute poses from external odom sensor sorted by stamps
  fuhe::io::TumToPosesEigen(tumFile, metric_poses, true);

  // -------------------- Read COLMAP model
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(input_path);
  // TODO: kick out again
  fuhe::col_utils::CropFarAwayPoints(reconstruction);

  // obtain all images from model
  const std::set<colmap::image_t>& reg_image_ids = reconstruction->RegImageIds();
  // get image ids in sorted order
  auto imgs_by_stamp = fuhe::col_utils::ImageIdsByStamp(reg_image_ids, reconstruction);

  // -------------------- Create directed odom edges between images in sorted order
  auto edges = fuhe::OdomEdgesManager::CreateOdomEdgesBetweenImages(imgs_by_stamp, metric_poses);

  // -------------------- Create Ceres problem
  std::shared_ptr<ceres::Problem> ceres_problem = std::make_shared<ceres::Problem>();

  // -------------------- Create fusion interface object
  hifuse::FusionGraphInterface fusion_interface(reconstruction, ceres_problem, log_to_rerun, save_rerun_rec, output_path);

  // -------------------- Iterate over COLMAP model to build factor graph problem
  int i = 0;  // image iteration counter

  double curr_img_stamp, prev_stamp = -1;  // stamps for successfully utilized external odoms

  // iterate over all images in model
  for (const auto pair : imgs_by_stamp) {
    curr_img_stamp = pair.first;
    colmap::image_t curr_img_id = pair.second;

    VLOG(2) << "Iteration for image: " << curr_img_id << " of stamp " << curr_img_stamp;

    // -------------------- First iteration init condition
    if (i == 0) {
      // start graph only if synchronized meas from both sources are availalbe
      if (metric_poses.find(curr_img_stamp) != metric_poses.end()) {
        VLOG(1) << "Found matching pose in tumfile! Kickoff factor graph construction";

        // add image reference image to bundle adjustment and force constant position but variable 3d pts
        fusion_interface.AddReprojectionFactor(curr_img_id, true, true, false);

        // preparing next iteration
        prev_stamp = curr_img_stamp;
        i++;  // break init loop
      }
      continue;
    }

    // -------------------- Add BA and rel pose factors to graph
    // add image to bundle adjustment with variable position and 3d points
    fusion_interface.AddReprojectionFactor(curr_img_id, false, false, false);

    // check if external odom is available for current image
    if (metric_poses.find(curr_img_stamp) == metric_poses.end()) {
      i++;
      continue;
    }

    // -------------------- Add relative odometry factor
    VLOG(2) << "Found matching tumposes to use as realtive pose factor!";

    // Get metric relative  pose of j (curr) expressed in i (prev) := i_from_j = world_from_i.inverse() * world_from_j
    const Eigen::Isometry3d T_i_from_j = metric_poses.at(prev_stamp).inverse() * metric_poses.at(curr_img_stamp);
    VLOG(5) << "Relataive motion is: " << T_i_from_j;

    // Define covariance of relative motion.
    Eigen::Matrix<double, 6, 6> covarince_i_from_j = Eigen::Matrix<double, 6, 6>::Identity() * cov;
    // weight non local z-axis motion and rotation in relative odometry
    fuhe::cov_utils::WeightPoseCovNonMotionDirection(covarince_i_from_j, non_motion_weighting);

    // -------------------- Inlcude metric relative pose factor in BA
    colmap::image_t prev_img_id = imgs_by_stamp.at(prev_stamp);
    fusion_interface.AddBetweenFactor(prev_img_id, curr_img_id, T_i_from_j, covarince_i_from_j);

    // preparing next iteration
    prev_stamp = curr_img_stamp;
    i++;
  }

  // TODO: double check where to place this
  // clear manually and incrementally registered 3D points that were logged per image
  if (fusion_interface.GetRerunRec()) {
    rrfuse::ClearAllCamPoints3D(fusion_interface.GetRerunRec(), reconstruction->Images());
  }

  // -------------------- Configure Bundle Adjustment for CERES and COLMAP
  ceres::Problem::Options ceres_options;  // ceres options

  // which images and points should be considered or set to constant
  colmap::BundleAdjustmentConfig ba_config;

  // -------------------- Set ceres solver options
  // Turn nonmono steps to true if its user specified
  if (max_consecutive_nonmonotonic_steps > 0) {
    VLOG(1) << "Allowing non-monotic steps with n: " << max_consecutive_nonmonotonic_steps;
    col_options.bundle_adjustment->solver_options.use_nonmonotonic_steps = true;
    col_options.bundle_adjustment->solver_options.max_consecutive_nonmonotonic_steps = max_consecutive_nonmonotonic_steps;
  }

  ceres::Solver::Options solver_options = col_options.bundle_adjustment->solver_options;
  solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  solver_options.num_threads = std::thread::hardware_concurrency();
  solver_options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;

  // use rerun iteration callback during ceres optim
  std::shared_ptr<fuhe::FusionIterationCallback> callback = nullptr;
  if (fusion_interface.GetRerunRec()) {
    // deploy own iteration callback that logs to rerun during optimization
    callback = std::make_shared<fuhe::FusionIterationCallback>(fusion_interface.GetRerunRec(),
                                                               fusion_interface.GetRerunPinhole(),
                                                               fusion_interface.GetReconstruction()->Images(),
                                                               fusion_interface.GetReconstruction()->Points3D(),
                                                               imgs_by_stamp,
                                                               edges);
    solver_options.callbacks.push_back(callback.get());
  }
  solver_options.minimizer_progress_to_stdout = false;
  solver_options.update_state_every_iteration = true;

  // TODO: implement residual eval correctly
  // -------------------- Evaluate errors
  //   VLOG(3) << "Starting to calc absolute odom error in graph before optimization!";
  //   if (VLOG_IS_ON(3)) {
  //     cetrahe::calcTotalOdomError(ceres_problem, odom_error_ids, "");
  //   }

  // -------------------- Perform Bundle Adjustment
  VLOG(1) << "Starting to solve graph!";
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, ceres_problem.get(), &summary);

  VLOG(1) << summary.FullReport();

  // // -------------------- Update image poses in rerun
  // fuhe::col_utils::CropFarAwayPoints(reconstruction);

  // TODO: decide on how to rerun visualize for good
  // if (fusion_interface.GetRerunRec()) {
  //   fusion_interface.UpdateWholeReconstroctionRerun();
  // fusion_interface.UpdateRegisterdFactorsRerun(metric_poses);
  // }

  // TODO: implement residual eval correctly
  // --------------------Metrics after optim
  //   VLOG(3) << "Starting to calc absolute odom error in graph after optimization!";
  //   if (VLOG_IS_ON(3)) {
  //     cetrahe::calcTotalOdomError(ceres_problem, odom_error_ids, "");
  //   }

  reconstruction->WriteText(output_path);

  reconstruction->TearDown();
}
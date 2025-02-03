// TODO: write brief

#include <ostream>
#include <thread>

#include "fusion_helper/io.h"
#include "fusion_helper/stream_utils.h"
#include "fusion_helper/col_utils.h"
#include "fusion_helper/cov_utils.h"
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

  colmap::OptionManager col_options;

  col_options.AddRequiredOption("input_path", &input_path);
  col_options.AddRequiredOption("output_path", &output_path);
  col_options.AddDefaultOption("cov", &cov);
  col_options.AddDefaultOption("non_motion_weighting", &non_motion_weighting);
  col_options.AddBundleAdjustmentOptions();
  col_options.Parse(argc, argv);

  // -------------------- Logging stuff
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;

  // Set log directory
  FLAGS_log_dir = "./../logs"; // TODO: fix log dir

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
  auto reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(input_path);

  // obtain all images from model
  const std::set<colmap::image_t>& reg_image_ids = reconstruction->RegImageIds();
  // get image ids in sorted order
  auto imgs_by_stamp = fuhe::col_utils::ImageIdsByStamp(reg_image_ids, reconstruction);

  // -------------------- Create Ceres problem
  std::shared_ptr<ceres::Problem> ceres_problem = std::make_shared<ceres::Problem>();

  // define ids to obtain resiudals per type
  std::vector<std::vector<ceres::ResidualBlockId>>
      reprojection_error_ids;  // NOTE: each entry contains id for ALL reprojection residuals per image
  std::vector<ceres::ResidualBlockId> odom_error_ids;

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
        reprojection_error_ids.push_back(hifuse::AddReprojectionFactor(curr_img_id, ceres_problem, reconstruction, true, true, false));

        // preparing next iteration
        prev_stamp = curr_img_stamp;
        i++;  // break init loop
      }
      continue;
    }

    // -------------------- Add BA and rel pose factors to graph
    // add image to bundle adjustment with variable position and 3d points
    reprojection_error_ids.push_back(hifuse::AddReprojectionFactor(curr_img_id, ceres_problem, reconstruction, false, false, false));

    // -------------------- Add relative odometry factor
    if (metric_poses.find(curr_img_stamp) != metric_poses.end()) {
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
      odom_error_ids.push_back(
          hifuse::AddBetweenFactor(prev_img_id, curr_img_id, T_i_from_j, covarince_i_from_j, ceres_problem, reconstruction));

      // preparing next iteration
      prev_stamp = curr_img_stamp;
    }
    i++;
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
  solver_options.num_threads = std::thread::hardware_concurrency();

  solver_options.minimizer_progress_to_stdout = true;

    //TODO: implement residual eval correctly
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

    //TODO: implement residual eval correctly
  // --------------------Metrics after optim
  //   VLOG(3) << "Starting to calc absolute odom error in graph after optimization!";
  //   if (VLOG_IS_ON(3)) {
  //     cetrahe::calcTotalOdomError(ceres_problem, odom_error_ids, "");
  //   }

  reconstruction->WriteText(output_path);

  reconstruction->TearDown();
}
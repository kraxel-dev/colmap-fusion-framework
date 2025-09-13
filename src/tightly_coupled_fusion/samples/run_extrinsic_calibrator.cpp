/**
 * @file run_extrinsic_calibrator.cpp
 * @author kraxel
 * @brief Given a ref and a target tum trajectory of 2 different sensor links w.r.t. the same diriving sequence, find the optimal
 * extrinsics using ceres optimization.
 * @version 0.1
 * @date 2025-09-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <ostream>

#include "fusion_helper/col_utils.h"
#include "fusion_helper/cost_functions.h"
#include "fusion_helper/io.h"
#include "fusion_helper/stream_utils.h"
#include <colmap/controllers/option_manager.h>
#include <colmap/estimators/manifold.h>
#include <colmap/util/file.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////////////////////
  // Parse COLMAP and ceres options and inputs
  ////////////////////////////////////////////////////////////////////////////////
  std::string ref_tum_file;     // path to tum file of absolute pose traj with poses from ref sensor (e.g. lidar)
  std::string target_tum_file;  // path to tum file of absolute pose traj with poses from target sensor (e.g. camera)

  colmap::OptionManager col_options;  // classic colmap options and cmd arg parser

  // classic colmap options
  col_options.AddRequiredOption("ref_tum", &ref_tum_file);
  col_options.AddRequiredOption("target_tum", &target_tum_file);

  col_options.AddBundleAdjustmentOptions();
  // actually parse command line args into option members
  col_options.Parse(argc, argv);

  // -------------------- Logging stuff
  colmap::InitializeGlog(argv);
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;

  // Set log directory
  const std::string log_dir = fuhe::io::GetRepoRootDir() + "/logs";
  FLAGS_log_dir = log_dir;
  VLOG(2) << "Logging path is: " << log_dir;

  // -------------------- Read TUM files
  fuhe::types::MapOfPosesSec ref_poses, target_poses;  // absolute poses
  // fuhe::io::TumToPosesEigen(ref_tum_file, ref_poses, /*cut_precision=*/false);
  // fuhe::io::TumToPosesEigen(target_tum_file, target_poses, /*cut_precision=*/false);
  fuhe::io::TumToPosesEigen(ref_tum_file, ref_poses, /*cut_precision=*/true);
  fuhe::io::TumToPosesEigen(target_tum_file, target_poses, /*cut_precision=*/true);

  // -------------------- Allocate extrinsics to be estimated
  Eigen::Vector3d x(0.255, -0.013, -0.53);  // init guess for translation
  // Eigen::Vector3d x(0.255, -0.013, 0);                                                   // init guess for translation
  Eigen::Quaterniond q = Eigen::Quaterniond(0.46289, -0.53836, 0.54045, -0.451024);  // wxyz
  // extrinsics (pose of target sensor w.r.t. ref sensor)
  std::shared_ptr<colmap::Rigid3d> T_ref_from_target = std::make_shared<colmap::Rigid3d>(q, x);

  T_ref_from_target->rotation.normalize();
  // ceres param blocks for extrinsics
  double* q_rt = T_ref_from_target->rotation.coeffs().data();  // quaternion pointer
  double* t_rt = T_ref_from_target->translation.data();        // translation pointer

  // ceres graph
  std::shared_ptr<ceres::Problem> ceres_problem = std::make_shared<ceres::Problem>();

  // -------------------- Anchor initial geuss with translation prior cost
  Eigen::Vector3d weight(0.5, 0.5, 0.5);  // importance of each translation axis
  ceres::CostFunction* translation_prior = fuhe::cost::TranslationPriorCostFunctor::Create(x, weight);
  ceres_problem->AddResidualBlock(translation_prior, nullptr, t_rt);

  // -------------------- Iterate over all target poses to create edges
  VLOG(1) << "Constructing ceres problem with " << target_poses.size() - 1 << " relative pose measurements!";

  //* obtain rel poses from both sensors w.r.t. to target (camera) stamps. Assumes target tum is matched to ref poses.
  int i = 0;
  int n = 0;  // cost adder count
  double prev_stamp = -1;
  for (const auto& [stamp, _] : target_poses) {
    // init pose
    if (prev_stamp < 0 && ref_poses.find(stamp) != ref_poses.end()) {
      prev_stamp = stamp;
      continue;
    }

    if (ref_poses.find(stamp) == ref_poses.end()) {
      LOG(WARNING) << "No matching ref pose found for target stamp " << stamp << "! Skipping this measurement!";
      i++;

      continue;
    }

    // skip every second meas
    // if (i % 2 == 0) {
    //   i++;
    //   continue;
    // }

    // skip time jumps
    if (stamp - prev_stamp > 0.45) {
      LOG(WARNING) << "Time jump of " << stamp - prev_stamp << "s detected between target stamps " << prev_stamp << " and "
                   << stamp << "! Skipping this measurement!";
      prev_stamp = stamp;
      i++;
      continue;
    }

    // get rel poses from both sensors
    // -------------------- ref sensor rel pose (e.g. lidar)
    Eigen::Isometry3d T_ij_ref = ref_poses.at(prev_stamp).inverse() * ref_poses.at(stamp);
    const colmap::Rigid3d T_ij_ref_rigid = colmap::Rigid3d(Eigen::Quaterniond(T_ij_ref.rotation()), T_ij_ref.translation());

    // -------------------- target sensor rel pose (e.g. cam)
    Eigen::Isometry3d T_ij_target = target_poses.at(prev_stamp).inverse() * target_poses.at(stamp);
    const colmap::Rigid3d T_ij_target_rigid =
        colmap::Rigid3d(Eigen::Quaterniond(T_ij_target.rotation()), T_ij_target.translation());

    // add extrinsic calibration cost factor to ceres problem
    VLOG(2) << "Creating extrinsic cost factor for stamp:  " << stamp;

    ceres::CostFunction* cost_func = fuhe::cost::ExtrinsicCalibCostFunctor::Create(T_ij_ref_rigid, T_ij_target_rigid);
    ceres_problem->AddResidualBlock(cost_func, new ceres::HuberLoss(0.1), q_rt, t_rt);

    prev_stamp = stamp;
    i++;
    n++;
  }

  // -------------------- Double check if pose parameters are registered as manifold in optimizaion
  // Set Lie algebra for pose on manifold optimization in case it was not set already
  if (!ceres_problem->GetParameterization(q_rt)) {
    colmap::SetQuaternionManifold(ceres_problem.get(), q_rt);
  }

  VLOG(1) << "Graph constructed with  " << n << " cost factos!";

  // -------------------- Configure Bundle Adjustment for CERES and COLMAP
  ceres::Problem::Options ceres_options;  // ceres options

  // -------------------- Set ceres solver options
  colmap::BundleAdjustmentConfig ba_config;
  // create ceres solver options the same way that colmap would do (e.g. sparse vs dense depending on nr of resiudals)
  ceres::Solver::Options solver_options = col_options.bundle_adjustment->CreateSolverOptions(ba_config, *ceres_problem);
  // ceres::Solver::Options solver_options;
  // solver_options.max_num_iterations = 200;
  // solver_options.max_linear_solver_iterations = 200;
  // solver_options.use_nonmonotonic_steps = true;
  // solver_options.linear_solver_type = ceres::SPARSE_SCHUR;

  solver_options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
  // update colmap poses for every iteration such that rerun can log them during iteration callbacks
  solver_options.update_state_every_iteration = false;

  // -------------------- Perform Optimization
  VLOG(1) << "Starting to solve graph!";
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, ceres_problem.get(), &summary);

  // -------------------- Optimize ceres problem
  VLOG(1) << "Extrinsic Pose: " << T_ref_from_target->ToMatrix();
  VLOG(1) << "Translation:" << T_ref_from_target->translation;
  VLOG(1) << "Rotation xyzw order:\n"
          << T_ref_from_target->rotation.x() << " " << T_ref_from_target->rotation.y() << " " << T_ref_from_target->rotation.z()
          << " " << T_ref_from_target->rotation.w();

  return EXIT_SUCCESS;
}
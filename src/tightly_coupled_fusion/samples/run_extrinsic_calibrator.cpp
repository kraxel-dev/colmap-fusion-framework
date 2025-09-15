/**
 * @file run_extrinsic_calibrator.cpp
 * @author kraxel
 * @brief Given a ref, a target, and an aux tum trajectory of 3 different sensor links w.r.t. the same diriving sequence, find
 * the optimal extrinsics using ceres optimization.
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
  std::string aux_tum_file;     // path to tum file of absolute pose traj with poses from auxiliary sensor (e.g. baselink radar)

  colmap::OptionManager col_options;  // classic colmap options and cmd arg parser

  // classic colmap options
  col_options.AddRequiredOption("ref_tum", &ref_tum_file);
  col_options.AddRequiredOption("target_tum", &target_tum_file);
  col_options.AddRequiredOption("aux_tum", &aux_tum_file);

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
  fuhe::types::MapOfPosesSec ref_poses, target_poses, aux_poses;  // absolute poses
  fuhe::io::TumToPosesEigen(ref_tum_file, ref_poses, /*cut_precision=*/true);
  fuhe::io::TumToPosesEigen(target_tum_file, target_poses, /*cut_precision=*/true);
  fuhe::io::TumToPosesEigen(aux_tum_file, aux_poses, /*cut_precision=*/true);

  // -------------------- Allocate extrinsics to be estimated

  Eigen::Vector3d x(0, 0, 0);                             // init guess for translation
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // wxyz
  // extrinsics (pose of target sensor w.r.t. ref sensor)
  std::shared_ptr<colmap::Rigid3d> T_ref_from_target = std::make_shared<colmap::Rigid3d>(q, x);
  // extrinsics from ref to aux sensor (e.g. lidar to radar)
  std::shared_ptr<colmap::Rigid3d> T_ref_from_aux = std::make_shared<colmap::Rigid3d>(q, x);
  // extrinsics from aux to target sensor (e.g. radar to cam)
  std::shared_ptr<colmap::Rigid3d> T_aux_from_target = std::make_shared<colmap::Rigid3d>(q, x);

  T_ref_from_target->rotation.normalize();
  T_ref_from_aux->rotation.normalize();
  T_aux_from_target->rotation.normalize();
  // ceres param blocks for extrinsics
  double* q_rt = T_ref_from_target->rotation.coeffs().data();  // quaternion pointer
  double* t_rt = T_ref_from_target->translation.data();        // translation pointer
  double* q_ra = T_ref_from_aux->rotation.coeffs().data();     // quaternion pointer
  double* t_ra = T_ref_from_aux->translation.data();           // translation pointer
  double* q_at = T_aux_from_target->rotation.coeffs().data();  // quaternion pointer
  double* t_at = T_aux_from_target->translation.data();        // translation pointer
  // scale param block
  std::shared_ptr<double> scale_diff_rt = std::make_shared<double>(1.0);
  std::shared_ptr<double> scale_diff_ra = std::make_shared<double>(1.0);
  std::shared_ptr<double> scale_diff_at = std::make_shared<double>(1.0);
  double* scale_diff_rt_ptr = scale_diff_rt.get();
  double* scale_diff_ra_ptr = scale_diff_ra.get();
  double* scale_diff_at_ptr = scale_diff_at.get();

  // ceres graph
  std::shared_ptr<ceres::Problem> ceres_problem = std::make_shared<ceres::Problem>();

  // -------------------- Anchor initial geuss with translation prior cost
  Eigen::Vector3d t_prior_rt(0.255, -0.013, -0.534);  // init guess for translation
  Eigen::Vector3d t_prior_ra(0, 0, -2.089);           // init guess for translation
  Eigen::Vector3d t_prior_at(1.580, 0.060, 1.566);    // init guess for translation

  Eigen::Vector3d weight(0.1, 0.5, 0.1);     // importance of each translation axis
  Eigen::Vector3d weight_ra(0.0, 0.1, 0.1);  // importance of each translation axis
  Eigen::Vector3d weight_at(0.1, 0.5, 0.1);  // importance of each translation axis
  ceres::CostFunction* translation_prior_rt = fuhe::cost::TranslationPriorCostFunctor::Create(t_prior_rt, weight);
  ceres::CostFunction* translation_prior_ra = fuhe::cost::TranslationPriorCostFunctor::Create(t_prior_ra, weight_ra);
  ceres::CostFunction* translation_prior_at = fuhe::cost::TranslationPriorCostFunctor::Create(t_prior_at, weight_at);
  ceres_problem->AddResidualBlock(translation_prior_rt, nullptr, t_rt);
  ceres_problem->AddResidualBlock(translation_prior_ra, nullptr, t_ra);
  ceres_problem->AddResidualBlock(translation_prior_at, nullptr, t_at);

  // -------------------- Anchor extrsinsics for consistency between each other
  Eigen::Matrix<double, 6, 6> weight6 = Eigen::Matrix<double, 6, 6>::Identity() * 15;  // bigger the more importance
  // ceres::CostFunction* tri_extr_consist_factor = fuhe::cost::TriExtrinsicCalibCostFunctor::Create(weight6);
  ceres::CostFunction* tri_extr_consist_factor =
      colmap::CovarianceWeightedCostFunctor<fuhe::cost::TriExtrinsicCalibCostFunctor>::Create(weight6);
  ceres_problem->AddResidualBlock(tri_extr_consist_factor, nullptr, q_rt, t_rt, q_ra, t_ra, q_at, t_at);

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

    if (ref_poses.find(stamp) == ref_poses.end() || aux_poses.find(stamp) == aux_poses.end()) {
      LOG(WARNING) << "No matching ref or aux pose found for target stamp " << stamp << "! Skipping this measurement!";
      i++;

      continue;
    }

    // skip time jumps
    double thresh = 1.3;  // s
    if (stamp - prev_stamp > thresh) {
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

    // -------------------- target sensor rel pose (e.g. cam)
    Eigen::Isometry3d T_ij_aux = aux_poses.at(prev_stamp).inverse() * aux_poses.at(stamp);
    const colmap::Rigid3d T_ij_aux_rigid = colmap::Rigid3d(Eigen::Quaterniond(T_ij_aux.rotation()), T_ij_aux.translation());

    // -------------------- add extrinsic calibration cost factor to ceres problem
    VLOG(2) << "Creating extrinsic cost factor for stamp:  " << stamp;

    // dummy cov as we do not have info about measurement uncertainty
    Eigen::Matrix<double, 6, 6> cov_i_from_j = Eigen::Matrix<double, 6, 6>::Identity() * 5;

    // -------------------- ref to target
    ceres::CostFunction* cost_func_rt = colmap::CovarianceWeightedCostFunctor<fuhe::cost::ExtrinsicCalibCostFunctor>::Create(
        cov_i_from_j, T_ij_ref_rigid, T_ij_target_rigid);
    ceres_problem->AddResidualBlock(cost_func_rt, new ceres::HuberLoss(1), scale_diff_rt_ptr, q_rt, t_rt);

    // -------------------- ref to aux
    ceres::CostFunction* cost_func_ra = colmap::CovarianceWeightedCostFunctor<fuhe::cost::ExtrinsicCalibCostFunctor>::Create(
        cov_i_from_j, T_ij_ref_rigid, T_ij_aux_rigid);
    ceres_problem->AddResidualBlock(cost_func_ra, new ceres::HuberLoss(1), scale_diff_ra_ptr, q_ra, t_ra);

    // -------------------- aux to target
    ceres::CostFunction* cost_func_at = colmap::CovarianceWeightedCostFunctor<fuhe::cost::ExtrinsicCalibCostFunctor>::Create(
        cov_i_from_j, T_ij_aux_rigid, T_ij_target_rigid);
    ceres_problem->AddResidualBlock(cost_func_at, new ceres::HuberLoss(1), scale_diff_ra_ptr, q_at, t_at);

    prev_stamp = stamp;
    i++;
    n++;
  }

  // -------------------- Double check if pose parameters are registered as manifold in optimizaion
  // Set Lie algebra for pose on manifold optimization in case it was not set already
  if (!ceres_problem->GetParameterization(q_rt)) {
    colmap::SetQuaternionManifold(ceres_problem.get(), q_rt);
  }
  if (!ceres_problem->GetParameterization(q_ra)) {
    colmap::SetQuaternionManifold(ceres_problem.get(), q_ra);
  }
  if (!ceres_problem->GetParameterization(q_at)) {
    colmap::SetQuaternionManifold(ceres_problem.get(), q_at);
  }

  VLOG(1) << "Graph constructed with  " << n << " cost factors!";

  // -------------------- Configure Bundle Adjustment for CERES and COLMAP
  ceres::Problem::Options ceres_options;  // ceres options

  // -------------------- Set ceres solver options
  colmap::BundleAdjustmentConfig ba_config;
  // create ceres solver options the same way that colmap would do (e.g. sparse vs dense depending on nr of resiudals)
  ceres::Solver::Options solver_options = col_options.bundle_adjustment->CreateSolverOptions(ba_config, *ceres_problem);

  solver_options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
  // update colmap poses for every iteration such that rerun can log them during iteration callbacks
  solver_options.update_state_every_iteration = false;

  // -------------------- Perform Optimization
  VLOG(1) << "Starting to solve graph!";
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, ceres_problem.get(), &summary);

  // -------------------- Optimize ceres problem
  VLOG(1) << "\nExtrinsic Pose Ref to target:" << T_ref_from_target->ToMatrix();
  VLOG(1) << "Scale Diff: " << *scale_diff_rt;
  VLOG(1) << "Translation:" << T_ref_from_target->translation;
  VLOG(1) << "Rotation xyzw order:\n"
          << T_ref_from_target->rotation.x() << " " << T_ref_from_target->rotation.y() << " " << T_ref_from_target->rotation.z()
          << " " << T_ref_from_target->rotation.w();

  VLOG(1) << "\nExtrinsic Pose Ref to Aux:" << T_ref_from_aux->ToMatrix();
  VLOG(1) << "Scale Diff: " << *scale_diff_ra;
  VLOG(1) << "Translation:" << T_ref_from_aux->translation;
  VLOG(1) << "Rotation xyzw order:\n"
          << T_ref_from_aux->rotation.x() << " " << T_ref_from_aux->rotation.y() << " " << T_ref_from_aux->rotation.z() << " "
          << T_ref_from_aux->rotation.w();

  VLOG(1) << "\nExtrinsic Pose Aux to target:" << T_aux_from_target->ToMatrix();
  VLOG(1) << "Scale Diff: " << *scale_diff_at;
  VLOG(1) << "Translation:" << T_aux_from_target->translation;
  VLOG(1) << "Rotation xyzw order:\n"
          << T_aux_from_target->rotation.x() << " " << T_aux_from_target->rotation.y() << " " << T_aux_from_target->rotation.z()
          << " " << T_aux_from_target->rotation.w();

  return EXIT_SUCCESS;
}
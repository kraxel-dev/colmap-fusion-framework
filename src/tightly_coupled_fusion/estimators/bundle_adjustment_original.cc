/**
 * @file bundle_adjustment_original.cc
 * @author kraxel
 * @brief Exact dupliaction of colmaps bundle_adjustment implementation. Duplication is needed, as colmap does not provide header
 * declaration of their BA classes which fusion Bundle Adjuster (implemented in this project) needs to derive from.
 * @ref (original colmap repo) src/colmap/estimators/bundle_adjustment.cc
 * @version 0.1
 * @date 2025-03-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "tightly_coupled_fusion/estimators/bundle_adjustment_original.h"

#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/manifold.h>

// -------------------- Exact duplicate of colmap code
namespace colmap {  // namespace colmap
void ParameterizeCameras(const BundleAdjustmentOptions& options,
                         const BundleAdjustmentConfig& config,
                         const std::unordered_set<camera_t>& camera_ids,
                         Reconstruction& reconstruction,
                         ceres::Problem& problem) {
  const bool constant_camera = !options.refine_focal_length && !options.refine_principal_point && !options.refine_extra_params;
  for (const camera_t camera_id : camera_ids) {
    Camera& camera = reconstruction.Camera(camera_id);

    if (constant_camera || config.HasConstantCamIntrinsics(camera_id)) {
      problem.SetParameterBlockConstant(camera.params.data());
    } else {
      std::vector<int> const_camera_params;

      if (!options.refine_focal_length) {
        const span<const size_t> params_idxs = camera.FocalLengthIdxs();
        const_camera_params.insert(const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      }
      if (!options.refine_principal_point) {
        const span<const size_t> params_idxs = camera.PrincipalPointIdxs();
        const_camera_params.insert(const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      }
      if (!options.refine_extra_params) {
        const span<const size_t> params_idxs = camera.ExtraParamsIdxs();
        const_camera_params.insert(const_camera_params.end(), params_idxs.begin(), params_idxs.end());
      }

      if (const_camera_params.size() > 0) {
        SetSubsetManifold(static_cast<int>(camera.params.size()), const_camera_params, &problem, camera.params.data());
      }
    }
  }
}

void ParameterizePoints(const BundleAdjustmentConfig& config,
                        const std::unordered_map<point3D_t, size_t>& point3D_num_observations,
                        Reconstruction& reconstruction,
                        ceres::Problem& problem) {
  for (const auto& [point3D_id, num_observations] : point3D_num_observations) {
    Point3D& point3D = reconstruction.Point3D(point3D_id);
    if (point3D.track.Length() > num_observations) {
      problem.SetParameterBlockConstant(point3D.xyz.data());
    }
  }

  for (const point3D_t point3D_id : config.ConstantPoints()) {
    Point3D& point3D = reconstruction.Point3D(point3D_id);
    problem.SetParameterBlockConstant(point3D.xyz.data());
  }
}

// -------------------- Exact duplicate of colmaps DefaultBundleAdjuster class
DefaultBundleAdjuster::DefaultBundleAdjuster(BundleAdjustmentOptions options, BundleAdjustmentConfig config, Reconstruction& reconstruction)
    : BundleAdjuster(std::move(options), std::move(config)),
      loss_function_(std::unique_ptr<ceres::LossFunction>(options_.CreateLossFunction())) {
  ceres::Problem::Options problem_options;
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_ = std::make_shared<ceres::Problem>(problem_options);

  // Set up problem
  // Warning: AddPointsToProblem assumes that AddImageToProblem is called
  // first. Do not change order of instructions!
  for (const image_t image_id : config_.Images()) {
    AddImageToProblem(image_id, reconstruction);
  }
  for (const auto point3D_id : config_.VariablePoints()) {
    AddPointToProblem(point3D_id, reconstruction);
  }
  for (const auto point3D_id : config_.ConstantPoints()) {
    AddPointToProblem(point3D_id, reconstruction);
  }

  ParameterizeCameras(options_, config_, camera_ids_, reconstruction, *problem_);
  ParameterizePoints(config_, point3D_num_observations_, reconstruction, *problem_);
}

ceres::Solver::Summary DefaultBundleAdjuster::Solve() {
  ceres::Solver::Summary summary;
  if (problem_->NumResiduals() == 0) {
    return summary;
  }

  const ceres::Solver::Options solver_options = options_.CreateSolverOptions(config_, *problem_);

  ceres::Solve(solver_options, problem_.get(), &summary);

  if (options_.print_summary || VLOG_IS_ON(1)) {
    PrintSolverSummary(summary, "Bundle adjustment report");
  }

  return summary;
}

void DefaultBundleAdjuster::AddImageToProblem(const image_t image_id, Reconstruction& reconstruction) {
  Image& image = reconstruction.Image(image_id);
  Camera& camera = *image.CameraPtr();

  // CostFunction assumes unit quaternions.
  image.CamFromWorld().rotation.normalize();

  double* cam_from_world_rotation = image.CamFromWorld().rotation.coeffs().data();
  double* cam_from_world_translation = image.CamFromWorld().translation.data();
  double* camera_params = camera.params.data();

  const bool constant_cam_pose = !options_.refine_extrinsics || config_.HasConstantCamPose(image_id);

  // Add residuals to bundle adjustment problem.
  size_t num_observations = 0;
  for (const Point2D& point2D : image.Points2D()) {
    if (!point2D.HasPoint3D()) {
      continue;
    }

    num_observations += 1;
    point3D_num_observations_[point2D.point3D_id] += 1;

    Point3D& point3D = reconstruction.Point3D(point2D.point3D_id);
    assert(point3D.track.Length() > 1);

    if (constant_cam_pose) {
      problem_->AddResidualBlock(
          CreateCameraCostFunction<ReprojErrorConstantPoseCostFunctor>(camera.model_id, point2D.xy, image.CamFromWorld()),
          loss_function_.get(),
          point3D.xyz.data(),
          camera_params);
    } else {
      problem_->AddResidualBlock(CreateCameraCostFunction<ReprojErrorCostFunctor>(camera.model_id, point2D.xy),
                                 loss_function_.get(),
                                 cam_from_world_rotation,
                                 cam_from_world_translation,
                                 point3D.xyz.data(),
                                 camera_params);
    }
  }

  if (num_observations > 0) {
    camera_ids_.insert(image.CameraId());

    // Set pose parameterization.
    if (!constant_cam_pose) {
      SetQuaternionManifold(problem_.get(), cam_from_world_rotation);
      if (config_.HasConstantCamPositions(image_id)) {
        const std::vector<int>& constant_position_idxs = config_.ConstantCamPositions(image_id);
        SetSubsetManifold(3, constant_position_idxs, problem_.get(), cam_from_world_translation);
      }
    }
  }
}

void DefaultBundleAdjuster::AddPointToProblem(const point3D_t point3D_id, Reconstruction& reconstruction) {
  Point3D& point3D = reconstruction.Point3D(point3D_id);

  // Is 3D point already fully contained in the problem? I.e. its entire track
  // is contained in `variable_image_ids`, `constant_image_ids`,
  // `constant_x_image_ids`.
  if (point3D_num_observations_[point3D_id] == point3D.track.Length()) {
    return;
  }

  for (const auto& track_el : point3D.track.Elements()) {
    // Skip observations that were already added in `FillImages`.
    if (config_.HasImage(track_el.image_id)) {
      continue;
    }

    point3D_num_observations_[point3D_id] += 1;

    Image& image = reconstruction.Image(track_el.image_id);
    Camera& camera = *image.CameraPtr();
    const Point2D& point2D = image.Point2D(track_el.point2D_idx);

    // CostFunction assumes unit quaternions.
    image.CamFromWorld().rotation.normalize();

    // We do not want to refine the camera of images that are not
    // part of `constant_image_ids_`, `constant_image_ids_`,
    // `constant_x_image_ids_`.
    if (camera_ids_.count(image.CameraId()) == 0) {
      camera_ids_.insert(image.CameraId());
      config_.SetConstantCamIntrinsics(image.CameraId());
    }
    problem_->AddResidualBlock(
        CreateCameraCostFunction<ReprojErrorConstantPoseCostFunctor>(camera.model_id, point2D.xy, image.CamFromWorld()),
        loss_function_.get(),
        point3D.xyz.data(),
        camera.params.data());
  }
}

}  // namespace colmap

/**
 * @file bundle_adjustment.h
 * @author kraxel
 * @brief Exact dupliaction of colmaps bundle_adjustment implementation. Duplication is needed, as colmap does not provide header
 * declaration of their BA classes which fusion Bundle Adjuster (implemented in this project) needs to derive from.
 * @ref (original colmap repo) src/colmap/estimators/bundle_adjustment.h
 * @version 0.1
 * @date 2025-03-12
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/scene/reconstruction.h>

namespace colmap {

/// Original colmap implementation (exact copy of the colmap code with explcit declarations in header)
void ParameterizeCameras(const BundleAdjustmentOptions& options,
                         const BundleAdjustmentConfig& config,
                         const std::unordered_set<camera_t>& camera_ids,
                         Reconstruction& reconstruction,
                         ceres::Problem& problem);
/// Original colmap implementation (exact copy of the colmap code with explcit declarations in header)
void ParameterizePoints(const BundleAdjustmentConfig& config,
                        const std::unordered_map<point3D_t, size_t>& point3D_num_observations,
                        Reconstruction& reconstruction,
                        ceres::Problem& problem);

/// Original colmap implementation (exact copy of the colmap code with explcit declarations in header)
class DefaultBundleAdjuster : public BundleAdjuster {
 public:
  DefaultBundleAdjuster(BundleAdjustmentOptions options, BundleAdjustmentConfig config, Reconstruction& reconstruction);

  ceres::Solver::Summary Solve() override;

  std::shared_ptr<ceres::Problem>& Problem() override { return problem_; }

  void AddImageToProblem(const image_t image_id, Reconstruction& reconstruction);

  void AddPointToProblem(const point3D_t point3D_id, Reconstruction& reconstruction);

 private:
  std::shared_ptr<ceres::Problem> problem_;
  std::unique_ptr<ceres::LossFunction> loss_function_;

  std::unordered_set<camera_t> camera_ids_;
  std::unordered_map<point3D_t, size_t> point3D_num_observations_;
};

}  // namespace colmap
/**
 * @file fusion_graph_interface.h
 * @author kraxel
 * @brief Helper functions for:
 * - generating ceres sensor cost factors from colmap models and other modalities
 * - adding them to the ceres bundle adjustment graph / ceres optimizatino problem.
 * Used for high-level fusion where colmap models are assumed to be
 * fully reconstructed.
 * @version 0.1
 * @date 2025-02-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstdlib>

#include <ceres/problem.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/exe/sfm.h>

namespace hifuse {  // high-level fusion

/**
* @brief From forwarded colmap image, get pointers to image pose (cam_from_world: world pose expressed in cam) objects required by ceres for
considering image pose as parameters for the optimization problem in the factor graph. Takes care of quaternion normalization for
convenience.

* @param img Reference to colmap image whose pose we want retrieve as paramter for optimization.
* @param q_c_from_w pointer to first quaternion value (double) in memory
* @param t_c_from_w pointer to first translation value (double) in memory
*/
void GetPointersToPose(colmap::Image& img, double*& q_c_from_w, double*& t_c_from_w);

std::vector<ceres::ResidualBlockId> AddReprojectionFactor(const colmap::image_t img_id,
                                                          std::shared_ptr<ceres::Problem> ceres_graph,
                                                          std::shared_ptr<colmap::Reconstruction> reconstruction,
                                                          const bool const_t = false,
                                                          const bool const_q = false,
                                                          const bool const_3d_pts = false);

ceres::ResidualBlockId AddBetweenFactor(const colmap::image_t img_id_i,
                                        const colmap::image_t img_id_j,
                                        const Eigen::Isometry3d& i_from_j,
                                        const Eigen::Matrix<double, 6, 6> cov_i_from_j,
                                        std::shared_ptr<ceres::Problem> ceres_graph,
                                        std::shared_ptr<colmap::Reconstruction> model);
}  // namespace hifuse
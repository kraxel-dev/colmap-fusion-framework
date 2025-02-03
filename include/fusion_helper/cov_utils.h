// TODO: add docstring
#pragma once

#include <Eigen/Core>

namespace fuhe {
namespace cov_utils {

/**
 * @brief Downweight the covariance matrix for non-motion directions and rotation. E.g: For moving vehicles with cameras pointing forwards
 the direction of relative motion is mostly along z-axis (inlcuding some elevation) with rotation happening around local y-axis of camera.
 NOTE: That this helper function is tailored to specific use-case and is not suited for general use.
 *
 * @param cov Uniform covariance matrix (e.g. identiy)
 * @param llambda Weighting factor to downweight non-motion directions and rotation
 */
void WeightPoseCovNonMotionDirection(Eigen::Matrix<double, 6, 6>& cov, const double llambda);
}  // namespace cov_utils
}  // namespace fuhe
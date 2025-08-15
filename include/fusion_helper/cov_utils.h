// TODO: add docstring
#pragma once

#include <Eigen/Core>

namespace fuhe {
namespace cov_utils {
/**
 * @brief Options for odometry covariance matrix generation. Please store the std deviations [rad, m] per second for translation
 * and rotation axes.
 *
 */
struct OdomCovOptions {
  // translation standard deviations per second
  double std_tx_per_s = 0.1;  // (per second) standard deviation for x axis translation
  double std_ty_per_s = 0.1;  // (per second) standard deviation for y axis translation
  double std_tz_per_s = 0.1;  // (per second) standard deviation for z axis translation

  // rotation axis standard deviations per second
  double std_rx_per_s = 0.1;  // (per second) standard deviation for x axis rotation (rad)
  double std_ry_per_s = 0.1;  // (per second) standard deviation for y axis rotation (rad)
  double std_rz_per_s = 0.1;  // (per second) standard deviation for z axis rotation (rad)
};

/**
 * @brief Utiliy class for returning 6x6 covariance matrices for 6DoF relative odometry measurements. Odom Cov is scaled by the
 * time diff of each relative measurement.
 *
 */
class OdomCovManager {
 public:
  OdomCovManager(const OdomCovOptions& options);

  /**
   * @brief Get the 6x6 Cov Matrix scaled by the time diff of the rel pose. Cov entries follow COLMAP convention (first 3 entries
   * rotation, last 3 translation)
   *
   * @param time_diff time diff [secs] between 2 consec. imgs that odom meas refers to
   * @return const Eigen::Matrix<double, 6, 6>: 6x6 Cov Matrix for a rel pose (first 3 entries rotation, last 3 translation)
   */
  const Eigen::Matrix<double, 6, 6> GetTimeDependantCovMat(const double time_diff) const;

 protected:
  OdomCovOptions options_;

  // rotation axis variances per second
  double var_rx_per_s = 0.01;  // (per second) var for x axis rotation (rad^2)
  double var_ry_per_s = 0.01;  // (per second) var for y axis rotation (rad^2)
  double var_rz_per_s = 0.01;  // (per second) var for z axis rotation (rad^2)
  
  // translation variances per second
  double var_tx_per_s = 0.01;  // (per second) var for x axis translation [m^2]
  double var_ty_per_s = 0.01;  // (per second) var for y axis translation [m^2]
  double var_tz_per_s = 0.01;  // (per second) var for z axis translation [m^2]
};

}  // namespace cov_utils
}  // namespace fuhe
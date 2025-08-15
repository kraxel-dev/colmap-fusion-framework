#include "fusion_helper/cov_utils.h"

#include "fusion_helper/stream_utils.h"
#include <glog/logging.h>

namespace fuhe {
namespace cov_utils {

fuhe::cov_utils::OdomCovManager::OdomCovManager(const OdomCovOptions& options) : options_(options) {
  var_rx_per_s = pow(options_.std_rx_per_s, 2);
  var_ry_per_s = pow(options_.std_ry_per_s, 2);
  var_rz_per_s = pow(options_.std_rz_per_s, 2);

  var_tx_per_s = pow(options_.std_tx_per_s, 2);
  var_ty_per_s = pow(options_.std_ty_per_s, 2);
  var_tz_per_s = pow(options_.std_tz_per_s, 2);

  VLOG(2) << "6DoF Odom Covariance values per second:"
          << "\nrx [rad^2/s]:" << var_rx_per_s << " ry [rad^2/s]:" << var_ry_per_s << " rz [rad^2/s]:" << var_rz_per_s
          << "\ntx [m^2/s]:" << var_tx_per_s << " ty [m^2/s]:" << var_ty_per_s << " tz [m^2/s]:" << var_tz_per_s;
}

const Eigen::Matrix<double, 6, 6> fuhe::cov_utils::OdomCovManager::GetTimeDependantCovMat(const double time_diff) const {
  // create 6x6 covariance matrix
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  // fill in rotation covariances
  cov(0, 0) = var_rx_per_s * time_diff;
  cov(1, 1) = var_ry_per_s * time_diff;
  cov(2, 2) = var_rz_per_s * time_diff;
  // fill in translation covariances
  cov(3, 3) = var_rx_per_s * time_diff;
  cov(4, 4) = var_ry_per_s * time_diff;
  cov(5, 5) = var_rz_per_s * time_diff;

  VLOG(5) << "Created 6DoF odom cov for time diff of " << time_diff << " secs:\n" << cov;
  return cov;
}

const Eigen::Matrix<double, 6, 6> OdomCovManager::GetIdentityCovMat() const { return Eigen::Matrix<double, 6, 6>::Identity(); }

}  // namespace cov_utils
}  // namespace fuhe
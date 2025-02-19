#include "fusion_helper/cov_utils.h"
#include "fusion_helper/stream_utils.h"

#include <glog/logging.h>

void fuhe::cov_utils::WeightPoseCovNonMotionDirection(Eigen::Matrix<double, 6, 6>& cov, const double llambda) {
  // -------------------- Tune covariance for wheel odom case (no informatin on vehicle roll and pitch)
  VLOG(3) << "Downweighting covariance matrix for non-motion directions and rotation with factor: " << llambda;
  // increase uncertainty for vehicle roll, pitch and non-forwards facing motion
  cov(0, 0) *= llambda;  // vehicle pitch is local x axis in camera frame
  cov(2, 2) *= llambda;  // vehicle roll is local z axis in camera frame
  cov(3, 3) *= llambda;  // motion along vehicle lateral is x axis of local camera frame
  cov(4, 4) *= llambda;  // motion along vehicle height is -y axis of local camera frame
  VLOG(4) << "Covariance matrix after downweighting: " << cov;
}

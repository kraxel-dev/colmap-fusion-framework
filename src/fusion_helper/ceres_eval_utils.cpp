#include "fusion_helper/ceres_eval_utils.h"

#include <glog/logging.h>

namespace fuhe {
namespace ceres_eval_utils {

void LogBetweenFactorCost(ceres::CostFunction*& cost_func, double*& q_i, double*& t_i, double*& q_j, double*& t_j) {
  const double* params[4] = {q_i, t_i, q_j, t_j};  // pose parameters of both images
  double residuals[6];                             // residuals of relative pose factor (roll, pitch, yaw, x, y, z)

  // log reisudal error of current between factor
  cost_func->Evaluate(params, residuals, nullptr);
  LOG(INFO) << "X Y Z residuals in [m]: ";
  LOG(INFO) << "x: " << residuals[3] << " y: " << residuals[4] << " z: " << residuals[5];
  LOG(INFO) << "roll: " << residuals[0] << " pitch: " << residuals[1] << " yaw: " << residuals[2];
}

void LogReprojFactorCost(ceres::CostFunction*& cost_func, double*& q_cw, double*& t_cw, double*& pt3Dxyz, double*& camera_params) {
  const double* params[4] = {q_cw, t_cw, pt3Dxyz, camera_params};  // pose parameters of both images
  double residuals[2];                                             // residuals of relative pose factor (roll, pitch, yaw, x, y, z)

  // log reisudal error of current between factor
  cost_func->Evaluate(params, residuals, nullptr);

  LOG(INFO) << "Reprojection error -> x y residuals in image coords [px]: ";
  LOG(INFO) << "x [px]: " << residuals[0] << " y [px]: " << residuals[1];
}

}  // namespace ceres_eval_utils
}  // namespace fuhe
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

double CalcTotalFactorTypeCost(const std::shared_ptr<ceres::Problem> graph,
                               const std::vector<ceres::ResidualBlockId>& residual_ids) {
  // TODO: implement residual evaluation
  std::shared_ptr<std::vector<double>> residuals = std::make_shared<std::vector<double>>();  // error per state variable

  ceres::Problem::EvaluateOptions eval_options;
  eval_options.residual_blocks = residual_ids;

  double total_error = 0;  // total cost of factor type
  graph->Evaluate(eval_options, &total_error, residuals.get(), nullptr, nullptr);

  return total_error;
}

CeresCostEvaluator::CeresCostEvaluator(const std::shared_ptr<ceres::Problem> fusion_graph,
                                       const std::vector<std::vector<ceres::ResidualBlockId>>& reproj_residual_ids,
                                       const std::vector<ceres::ResidualBlockId>& odom_residual_ids)
    : fusion_graph(fusion_graph), reproj_residual_ids(reproj_residual_ids), odom_residual_ids(odom_residual_ids) {
  // -------------------- Flatten reprojection residual ids
  int num_reproj_residuals = 0;
  for (const auto& img_residuals : reproj_residual_ids) {
    num_reproj_residuals += img_residuals.size();
  }
  this->reproj_residual_ids_flattened.reserve(num_reproj_residuals);

  for (const auto& img_residuals : reproj_residual_ids) {
    reproj_residual_ids_flattened.insert(reproj_residual_ids_flattened.end(), img_residuals.begin(), img_residuals.end());
  }
}

double CeresCostEvaluator::CalcTotalOdomCost() const {
  const double total_error = CalcTotalFactorTypeCost(this->fusion_graph, this->odom_residual_ids);

  LOG(INFO) << "Total cost of odometry factors registered in ceres problem: " << total_error;
  LOG(INFO) << "Total amount of odometry factors: " << this->odom_residual_ids.size();

  return total_error;
}

double CeresCostEvaluator::CalcTotalReprojectionCost() const {
  const double total_error = CalcTotalFactorTypeCost(this->fusion_graph, this->reproj_residual_ids_flattened);

  LOG(INFO) << "Total cost of reprojection factors registered in ceres problem: " << total_error;
  LOG(INFO) << "Total amount of reprojection factors: " << this->reproj_residual_ids_flattened.size();

  return total_error;
}

}  // namespace ceres_eval_utils
}  // namespace fuhe
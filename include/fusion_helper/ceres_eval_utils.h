/**
 * @file ceres_eval_utils.h
 * @author kraxel
 * @brief Helper functions to evaluate actual error values for employed ceres cost functions.
 * @version 0.1
 * @date 2025-02-11
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <ceres/cost_function.h>
#include <ceres/problem.h>

namespace fuhe {
namespace ceres_eval_utils {

void LogBetweenFactorCost(ceres::CostFunction*& cost_func, double*& q_i, double*& t_i, double*& q_j, double*& t_j);

void LogReprojFactorCost(ceres::CostFunction*& cost_func, double*& q_cw, double*& t_cw, double*& pt3Dxyz, double*& camera_params);

/// Calculate total cost of all factors of specific type registered in ceres problem. User is responsible for grouping ids beforehand.
double CalcTotalFactorTypeCost(ceres::Problem& graph,
                               const std::shared_ptr<std::vector<ceres::ResidualBlockId>> residual_ids);

class CeresCostEvaluator {
 public:
  CeresCostEvaluator(ceres::Problem& fusion_graph,
                     const std::vector<std::vector<ceres::ResidualBlockId>>& reproj_residual_ids,
                     const std::vector<ceres::ResidualBlockId>& odom_residual_ids);
  double CalcTotalOdomCost() const;
  double CalcTotalReprojectionCost() const;

 protected:
  ceres::Problem& fusion_graph;  // ceres problem that acts as factor graph

  std::vector<std::vector<ceres::ResidualBlockId>>
      reproj_residual_ids;  // ceres ids for registerd reprojection factors for all images (each image has multiple residuals)
  std::vector<ceres::ResidualBlockId>
      reproj_residual_ids_flattened;  // flattened version of reprojection ids (poitns of all images in one single vector)
  std::vector<ceres::ResidualBlockId>
      odom_residual_ids;  // ceres ids for registerd odom factors such that we can perform residual evaluation
};

}  // namespace ceres_eval_utils
}  // namespace fuhe
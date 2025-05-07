/**
 * @file ceres_eval_utils.h
 * @author kraxel
 * @brief Helper functions and class to evaluate error values for employed ceres cost functions by category (e.g. reprojection
 * err vs relative pose factor). Functionalities of this .h and .cpp file are limited to usage before and after problem
 * optimization, eval during optimization is not possible.
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

/// Given a registered ceres relative-pose-factor and pointers to the pose params it constraints, glog the residual costs (r, p,
/// y, x, y, z) [rad, m] between the 2 absolute poses and their rel-pose factor. User is responsible for providing the correct
/// pointers. Based of ceres docs, you cannot call this during active optimization.
void LogBetweenFactorCost(ceres::CostFunction*& cost_func, double*& q_i, double*& t_i, double*& q_j, double*& t_j);

/// Given a registered reprojection factor and pointers to the pose it constraints, glog the x and y [pxl] residuals. User is
/// responsible for providing the correct pointers. Based of ceres docs, you cannot call this during active optimization.
void LogReprojFactorCost(
    ceres::CostFunction*& cost_func, double*& q_cw, double*& t_cw, double*& pt3Dxyz, double*& camera_params);

/// Calculate total (scalar) cost of all factors of specific type registered in ceres problem. User is responsible for grouping
/// ids beforehand.
double CalcTotalFactorTypeCost(ceres::Problem& graph, const std::shared_ptr<std::vector<ceres::ResidualBlockId>> residual_ids);

/**
 * @brief Class that stores ids of registered ceres cost factors by category (e.g. reprojection vs relpose factor) to evaluate
 * total error that each categegory contribute to the ceres problem. Evaluation can be called before and after optimization to
 * see which factor type the ceres solver reacted the most to. Eval during optimization is not possible, due to ceres
 * implementtaion details. Please refer to residual stalking for cost eval during optimization.
 *
 */
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
      reproj_residual_ids_flattened;  // flattened version of reprojection ids (points of all images in one single vector)
  std::vector<ceres::ResidualBlockId>
      odom_residual_ids;  // ceres ids for registerd odom factors such that we can perform residual evaluation
};

}  // namespace ceres_eval_utils
}  // namespace fuhe
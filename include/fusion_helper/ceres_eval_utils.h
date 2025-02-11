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

}  // namespace ceres_eval_utils
}  // namespace fuhe
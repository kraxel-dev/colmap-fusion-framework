#pragma once

#include "fusion_helper/fusion_residuals_tracker.h"
#include <ceres/ceres.h>
#include <ceres/evaluation_callback.h>

namespace fuhe {
namespace fusion_evaluation_callback {

/**
 * @brief Friend function that notifies a residual stalker object whether upcoming ceres evaluation step is a jacobian evaluation step. If
 not, stalker is allowed to obtain residuals from ceres cost functor for that iteration. Should only be called from ceres evaluation
 callback is its the only object that could know.
 *
 * @tparam kNumResiduals
 * @param is_jacobian_iter true or false. Knowledge comes from ceres evaluation callback.
 * @param stalker Residual stalker object to be notified
 */
template <int kNumResiduals>
void SetIsJacobianIter(const bool is_jacobian_iter, const std::shared_ptr<ResidualStalker<kNumResiduals>> stalker) {
  stalker->is_jacobian_iter = is_jacobian_iter;
}

/**
 * @brief Friend function to toggle status whether stalker object is supervised by evaluation callback. Only under supervision is a stalker
 * eglible for obtaining residual vectors from cost functors. This status should only be set once from the fusion evaluation callback
 * constructor.
 *
 * @tparam kNumResiduals
 * @param is_supervised
 * @param stalker Residual stalker object toggled as supervised
 */
template <int kNumResiduals>
void SetIsSupervisedByEvaluationCallback(const bool is_supervised, const std::shared_ptr<ResidualStalker<kNumResiduals>> stalker) {
  stalker->is_supervised_by_evaluation_callback = is_supervised;
};

}  // namespace fusion_evaluation_callback

/**
 * @brief Ceres EvaluatonCallback, called before every optimization/evaluation iteration of ceres. Derived to supervise external data
 * stalkers that track calculated residual vectors from each cost-functor duing optimization. If you want to add tracking for additional
 * factor types (e.g odom vs reproj), you need to add 2 things: 1. set supervision of the stalkers in this constructor. 2. set
 * jacboian iter status for each stalkers in "Prepare" callback. Only through supervision of this callback, are cost-factor residuals
 * allowed to be tracked during optimization. See docs of the derived PrepareForEvaluation() override for more details.
 *
 */
class FusionEvaluationCallback : public ceres::EvaluationCallback {
 public:
  FusionEvaluationCallback(const std::shared_ptr<FusionResidualsTracker> residuals_tracker);
  ~FusionEvaluationCallback() override = default;

  /**
   * @brief (Called by ceres before each cost function evaluation). Callback checks if current cost function evaluation
   * is a jacobian evlauation step. If not, all residual stalkers are notified that they are allowed to obtain residuals from ceres cost
   * functors. Reasoning: every ceres cost evaluation iteration actually consists of 2 steps which results in the cost-functor being called
   * 2 times in the same iteration. First, the residual vector calculation for each factor, second the jacobian evaluation. For the second
   * step (jacobian evaluation call), the resisudals are non-stalkable values and should not be passed by each cost functor to an associated
   * residual stalker.
   *
   * @param evaluate_jacobians
   * @param new_evaluation_point
   */
  void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) final;

 private:
  int eval_iter_count = 0;
  const std::shared_ptr<FusionResidualsTracker> residuals_tracker = nullptr;
};
}  // namespace fuhe

#include "fusion_helper/fusion_evaluation_callback.h"

namespace fuhe {

FusionEvaluationCallback::FusionEvaluationCallback(const std::shared_ptr<FusionResidualsTracker> residuals_tracker)
    : residuals_tracker{residuals_tracker} {
  // iterate over all residual stalkers
  auto stalked_odom_residuals = this->residuals_tracker->StalkedOdomResiduals();
  for (auto& [_, stalker] : stalked_odom_residuals) {
    // notify stalker that it is supervised by this evaluation callback
    fusion_evaluation_callback::SetIsSupervisedByEvaluationCallback(true, *stalker);
  }
};

void FusionEvaluationCallback::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
  // iterate over all residual stalkers
  auto stalked_odom_residuals = residuals_tracker->StalkedOdomResiduals();
  for (auto& [_, stalker] : stalked_odom_residuals) {
    // notify stalker that upcoming eval iteration is or is not a jacobian eval step, allowing the stalker to kidnap (copy) its residuals
    // if it IS NOT a jacobian eval step
    fusion_evaluation_callback::SetIsJacobianIter(evaluate_jacobians, *stalker);
  }
}
}  // namespace fuhe

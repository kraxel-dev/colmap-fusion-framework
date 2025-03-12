#include "fusion_helper/fusion_evaluation_callback.h"

namespace fuhe {

FusionEvaluationCallback::FusionEvaluationCallback(const std::shared_ptr<FusionResidualsTracker> residuals_tracker)
    : residuals_tracker{residuals_tracker} {
  // iterate over all odom residual stalkers
  // auto stalked_odom_residuals = this->residuals_tracker->StalkedOdomResiduals();
  for (auto& [_, stalker] : this->residuals_tracker->StalkedOdomResiduals()) {
    // notify stalker that it is supervised by this evaluation callback
    fusion_evaluation_callback::SetIsSupervisedByEvaluationCallback(true, stalker);
  }

  // iterate over all reproj residual stalkers
  // auto stalked_reproj_residuals = this->residuals_tracker->StalkedReprojectionResiduals();
  for (auto& [_, stalker] : this->residuals_tracker->StalkedReprojectionResiduals()) {
    // notify stalker that it is supervised by this evaluation callback
    fusion_evaluation_callback::SetIsSupervisedByEvaluationCallback(true, stalker);
  }
};

void FusionEvaluationCallback::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) {
  // iterate over all residual stalkers
  // auto stalked_odom_residuals = this->residuals_tracker->StalkedOdomResiduals();
  for (auto& [_, stalker] : this->residuals_tracker->StalkedOdomResiduals()) {
    // notify stalker whether upcoming eval iteration is a jacobian eval step or not, allowing the stalker to kidnap (copy) its residuals
    // if it IS NOT a jacobian eval step
    fusion_evaluation_callback::SetIsJacobianIter(evaluate_jacobians, stalker);
  }

  // auto stalked_reproj_residuals = this->residuals_tracker->StalkedReprojectionResiduals();
  for (auto& [_, stalker] : this->residuals_tracker->StalkedReprojectionResiduals()) {
    // notify stalker whether upcoming eval iteration is a jacobian eval step or not, allowing the stalker to kidnap (copy) its residuals
    // if it IS NOT a jacobian eval step
    fusion_evaluation_callback::SetIsJacobianIter(evaluate_jacobians, stalker);
  }
}
}  // namespace fuhe

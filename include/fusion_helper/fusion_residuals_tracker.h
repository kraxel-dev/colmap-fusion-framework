/**
 * @file fusion_residual_tracker.h
 * @author kraxel
 * @brief Data container for tracking residuals of ceres factors during optimization. Can be used for debugging or evaluation
 * purposes. To allow residual cost tracking during optim is a messy and convoluted endeavor (as you can tell from code below) is
 * achieved in combination with other classes in cost_functions.h and fusion_evaluation_callback.h respectively.
 * @version 0.1
 * @date 2025-03-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <map>

#include <Eigen/Core>
#include <ceres/cost_function.h>
#include <ceres/problem.h>

namespace fuhe {

// -------------------- Fun with Forward declarations
/// Forward declaration for stalker class to allow forward declaration of friend function
template <int ResidualVecSize>
class ResidualStalker;

// namespace needed for forward declaration
namespace fusion_evaluation_callback {

/// Forward declaration to allow friend access
template <int kNumResiduals>
void SetIsJacobianIter(const bool is_jacobian_iter, ResidualStalker<kNumResiduals>& stalker);

/// Forward declaration to allow friend access
template <int kNumResiduals>
void SetIsSupervisedByEvaluationCallback(const bool is_supervised, ResidualStalker<kNumResiduals>& stalker);
}  // namespace fusion_evaluation_callback
// -------------------- End of fun with Forward declarations

/**
 * @brief Structure that can track (stalk) the residual vector, calculated from a ceres cost-function during active optimization.
 * Ptr of stalker object needs to be attached to a residual-exposed cost-functor during functor creation AND to an extra data
 * container (residuals tracker) to view the obtained residuals. Residuals can only be obtained by stalker when being actively
 * registered and supervied by an FusionEvaluationObject.
 *
 * @tparam ResidualVecSize
 */
template <int ResidualVecSize>
class ResidualStalker {
 public:
  static const std::shared_ptr<ResidualStalker<ResidualVecSize>> Create() {
    return std::make_shared<ResidualStalker<ResidualVecSize>>();
  }

  /// (Friends only access) Toggle status whether stalker object is supervised by evaluation callback. Under supervision, stalker
  /// is eglible for obtaining residual vectors from cost functor in general. This function should ONLY be called by the
  /// supervising FusionEvaluationCallback.
  template <int kNumResiduals>
  friend void fusion_evaluation_callback::SetIsSupervisedByEvaluationCallback(
      const bool is_supervised, const std::shared_ptr<ResidualStalker<kNumResiduals>> stalker);
  /// (Friends only access) Notify stalker whether upcoming ceres evaluation step is a jacobian evaluation step. If not, stalker
  /// is allowed to obtain residuals from ceres cost functor for that iteration. This function should ONLY be called by the
  /// supervising FusionEvaluationCallback.
  template <int kNumResiduals>
  friend void fusion_evaluation_callback::SetIsJacobianIter(const bool is_jacobian_iter,
                                                            const std::shared_ptr<ResidualStalker<kNumResiduals>> stalker);

  /// Safety check whether stalker object is actively supervised by a fusion evaluation callback. Only under supervision is
  /// stalker eglible to obtain residual vectors from cost functor in general.
  inline bool IsSupervisedByEvaluationCallback() const { return this->is_supervised_by_evaluation_callback; }

  /// If true, stalker is allowed to obtain residuals for the current optim iteration from ceres cost functor it is attached to.
  /// The internal stalker flag that decides whether stalking is valid for current iter MUST BE CONTROLLED by its supervising
  /// FusionEvaluationCallback.
  inline const bool IsStalkingAllowedCurrentIter() const {
    // Do not attempt to obtain residuals from ceres cost functor during jacobian evaluation step.
    return !this->is_jacobian_iter;
  }

  /// Copy calculated ceres residual vector of current optim iteraton to stalker object. This should only ever be called by the
  /// ceres-cost functor this stalker is assigned to.
  void StalkResidualVector(const Eigen::Matrix<double, ResidualVecSize, 1>& residual) {
    // hardcore safety check
    if (!this->IsSupervisedByEvaluationCallback()) {
      LOG(ERROR) << "Trying to pass down ceres residuals to stalker without him being supervised by evaluation callback! Refer "
                    "to stalker "
                    "object docs to understand how to setup ceres residuals stalking during optimization.";
      return;
    }

    // skip assignment if current ceres iteration is a jacobian evaluation step
    if (this->is_jacobian_iter) {
      return;
    }

    // this->stalked_residual = residual.template cast<double>();
    this->stalked_residual = residual;

    // safety measure to refuse next attempt to pass residuals. Permission to stalk needs to be set externally be supervisor
    // again.
    this->is_jacobian_iter = false;
    this->iter_count++;
  }

  /// Overload that accepts ceres::Jets as template type that occur during an jacobian iter step. Do nothing in this overload,
  /// otherwise the jet type residuals would blow up the whole stalking process. Do not delete this overload, otherwise the jet
  /// types would break compilaition for the stalker that expects double tpyed matrices instead of jets. This should only ever be
  /// called by the ceres-cost functor this stalker is assigned to.
  template <typename T>
  void StalkResidualVector(const Eigen::Matrix<T, ResidualVecSize, 1>& residual) {
    // hardcore safety check
    if (!this->IsSupervisedByEvaluationCallback()) {
      LOG(ERROR) << "Trying to pass down ceres residuals to stalker without him being supervised by evaluation callback! Refer "
                    "to stalker "
                    "object docs to understand how to setup ceres residuals stalking during optimization.";
      return;
    }

    // skip assignment if current ceres iteration is a jacobian evaluation step
    if (this->is_jacobian_iter) {
      return;
    }
  }

  inline const Eigen::Matrix<double, ResidualVecSize, 1> GetTrackedResidual() const { return stalked_residual; }
  inline const std::string GetId() const { return this->id; }
  inline void SetId(const std::string& id) { this->id = id; }

 private:
  /// Only under supervision is a stalker eglible for obtaining residual vectors from cost functors. Supervision status is
  /// granted once the stalker is registerd in the FusionEvaluationCallback. On its own, neither the stalker nor the cost functor
  /// it is attached to know if active cost calculation step is a jacobian evaluation step or not.
  bool is_supervised_by_evaluation_callback = false;

  /// Ceres cost evaluation iteration is actually 2 steps. The cost functor is called 2 times in the same iteration. First, the
  /// residual vector calculation for each factor, second the jacobian evaluation. For the second step (jacobian evaluation
  /// call), the resisudals are non - stalkable values and should not be passed by each cost functor to an associated residual
  /// stalker. Stalker is notified by its
  bool is_jacobian_iter = true;
  int iter_count = 0;

  /// Tracked residual vector for current ceres iter
  Eigen::Matrix<double, ResidualVecSize, 1> stalked_residual = Eigen::Matrix<double, ResidualVecSize, 1>::Zero();
  std::string id = "";
};

/**
 * @brief Grouped stalker objects that track the calculated residual values, caused by each ceres cost function during
 * optimization. Needs to be attached to an FusionEvaluationCallback and each registered stalker object needed to be attached to
 * an to-be-tracked ceres cost-functor as well.
 *
 */
class FusionResidualsTracker {
 public:
  /// Create a new instance of the fusion residual tracker
  static const std::shared_ptr<FusionResidualsTracker> Create();

  /// Register a stalker for a residuals of ceres odometry factor
  void RegisterStalkedOdomResidual(const std::shared_ptr<ResidualStalker<6>> odom_residual_stalker, const std::string& edge_id);

  /// Register a stalker for a residual of a ceres reprojection factor
  void RegisterStalkedReprojectionResidual(const std::shared_ptr<ResidualStalker<2>> reproj_residual_stalker,
                                           const std::string& img_id,
                                           const std::string& pt3D_id);

  /// Get all registered stalker for odometry residuals as reference to underlying map
  const std::map<std::string, std::shared_ptr<ResidualStalker<6>>>& StalkedOdomResiduals() const;
  /// Get all registered stalker for reprojection residuals as reference to underlying map
  const std::map<std::string, std::shared_ptr<ResidualStalker<2>>>& StalkedReprojectionResiduals() const;

  /// For current iteration, get squared cost of all registered odometry residuals from stalkers
  const double GetTotalOdomCost() const;
  /// For current iteration, get squared cost of all registered reprojection residuals from stalkers
  const double GetTotalReprojCost() const;

 protected:
  std::map<std::string, std::shared_ptr<ResidualStalker<6>>> stalked_odom_residuals;
  std::map<std::string, std::shared_ptr<ResidualStalker<2>>> stalked_reprojection_residuals;
};

}  // namespace fuhe
/**
 * @file cost_functions.h
 * @author kraxel
 * @brief ceres cost functions
 * @version 0.1
 * @date 2025-02-27
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <Eigen/Core>
#include <colmap/estimators/cost_functions.h>
#include <fusion_helper/fusion_residuals_tracker.h>

namespace fuhe {
namespace cost {

/**
 * @brief Derived colmap::CovarianceWeightedCostFunctor, behaving exacly as colmap parent but with addition of exposing its
 * calculated residuals to outside world. Can be used for accessing residuals during ceres iteration or evaluation callbacks, if an stalker
 * obejct is attached to it during creation.
 *
 * @tparam CostFunctor
 */
template <class CostFunctor>
class WeightedCostExposedResiduals : public colmap::CovarianceWeightedCostFunctor<CostFunctor> {
 public:
  /// Covariance or sqrt information matrix type.
  using CovMat = Eigen::Matrix<double, CostFunctor::kNumResiduals, CostFunctor::kNumResiduals>;
  /// Shorten stalker type
  using ResidualStalkerPtr = std::shared_ptr<fuhe::ResidualStalker<CostFunctor::kNumResiduals>>;
  /// inherit ceres operator() overload from colmap CostFunctor parent
  using colmap::CovarianceWeightedCostFunctor<CostFunctor>::operator();

  ~WeightedCostExposedResiduals() = default;

  template <typename... Args>
  explicit WeightedCostExposedResiduals(const ResidualStalkerPtr residual_stalker, const CovMat& cov, Args&&... args)
      : residual_stalker{residual_stalker}, colmap::CovarianceWeightedCostFunctor<CostFunctor>{cov, std::forward<Args>(args)...} {}

  /**
   * @brief Create ceres cost-function of any native colmap weighted cost-functor type with additional capabilites to pass down its own
   * residuals during optimization. Usage: Create exactly like standard colmap CovarianceWeightedCostFunctor but with additional stalker
   * object as additional input in front of the covariance matrix.
   * @ref thirdparty/install/include/colmap/estimators/cost_functions.h
   *
   * @tparam Args
   * @param residual_stalker
   * @param cov
   * @param args
   * @return ceres::CostFunction*
   */
  template <typename... Args>
  static ceres::CostFunction* Create(const ResidualStalkerPtr residual_stalker, const CovMat& cov, Args&&... args) {
    return colmap::CreateAutoDiffCostFunction(
        new WeightedCostExposedResiduals<CostFunctor>(residual_stalker, cov, std::forward<Args>(args)...));
  }

  /// extend parents original operator() with additional section to store residuals (calculate by current ceres iteration)
  template <typename... Args>
  bool operator()(Args... args) const {
    // -------------------- Original operation() call
    // Call original parent function to calculate ceres residuals at current iteration
    if (!colmap::CovarianceWeightedCostFunctor<CostFunctor>::operator()(args...)) {
      return false;
    }

    // -------------------- Safety checks
    // no stalker = no stalking
    if (!this->residual_stalker) {
      return true;
    }

    // no supervision = no stalking
    if (!this->residual_stalker->IsSupervisedByEvaluationCallback()) {
      return true;
    }

    // if current operator() call is jacobian evaluation step, skip over passing down residuals to stalker
    if (!this->residual_stalker->IsStalkingAllowedCurrentIter()) {
      return true;  // skip due to resisudals being bogus from stalker perspective for jacobian iter.
    }

    // obtain residual ptr (internally filled by parent operator() call)
    /* NOTE: Every 2nd call of operator overlad results in residual ptr being of type ceres::Jet instead of typical numeric. Stalker should
     * have safety overalds to mitigate the Jet iterations since these would blow up residual stalking process
     */
    auto residuals_ptr = colmap::LastValueParameterPack(args...);
    typedef typename std::remove_reference<decltype(*residuals_ptr)>::type T;

    // load values at memory adress into variable
    Eigen::Map<Eigen::Matrix<T, CostFunctor::kNumResiduals, 1>> residuals(residuals_ptr);
    const Eigen::Matrix<T, CostFunctor::kNumResiduals, 1> tmp_res(residuals);

    // pass copy of calculated residuals to stalker object
    this->residual_stalker->StalkResidualVector(tmp_res);

    // iter_count++;
    return true;
  }

  /// Attach a stalker object for this cost-functor, which allows tracking of calculated residuals during ceres optimization.
  void AttachResidualStalker(const std::shared_ptr<fuhe::ResidualStalker<CostFunctor::kNumResiduals>> residual_stalker) {
    this->residual_stalker = residual_stalker;
  };

 protected:
  // stalker object for this cost-functor, which allows tracking of calculated residuals during ceres optimization
  const ResidualStalkerPtr residual_stalker = nullptr;  // NOTE: residuals here are locked to double data type
};

}  // namespace cost
}  // namespace fuhe
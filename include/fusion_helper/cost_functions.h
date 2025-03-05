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

namespace fuhe {
namespace cost_functions {

// 6-DoF error between two absolute camera poses based on a prior on their
// relative pose, with identical scale for the translation. The residual is
// computed in the frame of camera i. Its first and last three components
// correspond to the rotation and translation errors, respectively.
//
// Derivation:
//    i_T_w = ΔT_i·i_T_j·j_T_w
//    where ΔT_i = exp(η_i) is the resjdual in SE(3) and η_i in tangent space.
//    Thus η_i = log(i_T_w·j_T_w⁻¹·j_T_i)
//    Rotation term: ΔR = log(i_R_w·j_R_w⁻¹·j_R_i)
//    Translation term: Δt = i_t_w + i_R_w·j_R_w⁻¹·(j_t_i -j_t_w)
struct RelativePosePriorCostFunctor : public colmap::AutoDiffCostFunctor<RelativePosePriorCostFunctor, 6, 4, 3, 4, 3> {
 public:
  explicit RelativePosePriorCostFunctor(const colmap::Rigid3d& i_from_j_prior) : j_from_i_prior_(Inverse(i_from_j_prior)) {}

  template <typename T>
  bool operator()(const T* const i_from_world_rotation,
                  const T* const i_from_world_translation,
                  const T* const j_from_world_rotation,
                  const T* const j_from_world_translation,
                  T* residuals_ptr) const {
    VLOG(1) << "Cost operator overlad!";
    VLOG(1) << "Cost operator overlad!";
    VLOG(1) << "Cost operator overlad!";
    VLOG(1) << "Cost operator overlad!";

    const Eigen::Quaternion<T> i_from_j_rotation =
        colmap::EigenQuaternionMap<T>(i_from_world_rotation) * colmap::EigenQuaternionMap<T>(j_from_world_rotation).inverse();
    const Eigen::Quaternion<T> param_from_prior_rotation = i_from_j_rotation * j_from_i_prior_.rotation.cast<T>();
    colmap::EigenQuaternionToAngleAxis(param_from_prior_rotation.coeffs().data(), residuals_ptr);

    const Eigen::Matrix<T, 3, 1> j_from_i_prior_translation =
        j_from_i_prior_.translation.cast<T>() - colmap::EigenVector3Map<T>(j_from_world_translation);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> param_from_prior_translation(residuals_ptr + 3);
    param_from_prior_translation = colmap::EigenVector3Map<T>(i_from_world_translation) + i_from_j_rotation * j_from_i_prior_translation;

    return true;
  }

 private:
  const colmap::Rigid3d j_from_i_prior_;
};

/**
 * @brief Derived colmap::CovarianceWeightedCostFunctor object, behaving identically as colmap parent but with addition of exposing its
 * calculated residuals to outside world. Can be used for accessing residuals during ceres iteration or evaluation callbacks.
 *
 * @tparam CostFunctor
 */
template <class CostFunctor>
class WeightedCostExposedResiduals : public colmap::CovarianceWeightedCostFunctor<CostFunctor> {
 public:
  // inherit constructor from colmap CostFunctor parent
  using colmap::CovarianceWeightedCostFunctor<CostFunctor>::CovarianceWeightedCostFunctor;
  // inherit ceres operator() overload from colmap CostFunctor parent
  using colmap::CovarianceWeightedCostFunctor<CostFunctor>::operator();
  // using colmap::CovarianceWeightedCostFunctor<CostFunctor>::Create;

  // Covariance or sqrt information matrix type.
  using CovMat = Eigen::Matrix<double,
                               colmap::CovarianceWeightedCostFunctor<CostFunctor>::kNumResiduals,
                               colmap::CovarianceWeightedCostFunctor<CostFunctor>::kNumResiduals>;

  template <typename... Args>
  static ceres::CostFunction* Create(const CovMat& cov, Args&&... args) {
    return colmap::CreateAutoDiffCostFunction(new WeightedCostExposedResiduals<CostFunctor>(cov, std::forward<Args>(args)...));
  }

  /// extend parents original operator() with additional section to store residuals (calculate by current ceres iteration)
  template <typename... Args>
  bool operator()(Args... args) const {
    VLOG(1) << "Cost operator overlad!";

    // -------------------- Safety cheks also found in parent method
    // if (!cost_(args...)) {
    //   return false;
    // }

    // -------------------- Call original parent function to calculate ceres residuals at current iteration
    VLOG(1) << "Cost operator overlad 2!";

    const bool eval_success = colmap::CovarianceWeightedCostFunctor<CostFunctor>::operator()(args...);

    // -------------------- Store calculated residuals of current iteration
    if (!eval_success) {
      return false;
    }
    // obtain residual ptr that should now be filled
    auto residuals_ptr = colmap::LastValueParameterPack(args...);
    typedef typename std::remove_reference<decltype(*residuals_ptr)>::type T;
    // load values at memory adress into variable
    Eigen::Map<Eigen::Matrix<T, colmap::CovarianceWeightedCostFunctor<CostFunctor>::kNumResiduals, 1>> residuals(residuals_ptr);
    // store copy of calculated residuals as member
    this->residuals_ = residuals;
    VLOG(1) << "Stored residuals of current ceres iteration!" << iter_count;
    VLOG(1) << residuals;

    // iter_count++;
    return true;
  }

  /// Get residuals calculated by last ceres iteration
  Eigen::Matrix<double, colmap::CovarianceWeightedCostFunctor<CostFunctor>::kNumResiduals, 1> GetResiduals() const { return residuals_; }

 protected:
  // NOTE: residuals are locked here to double data type. In parent, residual type datatype is templated
  Eigen::Matrix<double, colmap::CovarianceWeightedCostFunctor<CostFunctor>::kNumResiduals, 1> residuals_;  // store residuals per iteration for external access
  int iter_count = 0;
};

}  // namespace cost_functions
}  // namespace fuhe
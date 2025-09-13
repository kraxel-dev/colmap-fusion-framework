/**
 * @file cost_functions.h
 * @author kraxel
 * @brief Adjusted ceres cost functions (from glomap, colmap and ceres itself) to construct sensor-fusion based optimization
 * problems (High-lvl fusion + tightly coupled fusion). Is used to extend vanilla colmap bundle adjustment problems with fusion
 * capabilities of external odometry and more. Also introduces additional capabilities like cost-functors that can track residual
 * cost during optimization.
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
 * calculated residuals to outside world. Can be used for accessing residuals during ceres iteration or evaluation callbacks, if
 * an stalker obejct is attached to it during creation. Please check associated class in fusion_evaluation_callback.h to build an
 * understanding of how to use residual tracking.
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
      : residual_stalker{residual_stalker},
        colmap::CovarianceWeightedCostFunctor<CostFunctor>{cov, std::forward<Args>(args)...} {}

  /**
   * @brief Create ceres cost-function of any native colmap weighted cost-functor type with additional capabilites to pass down
   * its own residuals during optimization. Usage: Create exactly like standard colmap CovarianceWeightedCostFunctor but with
   * additional stalker object as additional input in front of the covariance matrix.
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
      LOG(ERROR) << "Trying to pass down ceres residuals to stalker without him being supervised by evaluation callback! Refer "
                    "to stalker "
                    "object docs to understand how to setup ceres residuals stalking during optimization.";
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
    /* NOTE: Every 2nd call of operator overlad results in residual ptr being of type ceres::Jet instead of typical numeric.
     * Stalker should have safety overalds to mitigate the Jet iterations since these would blow up residual stalking process
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

/**
 * @brief Sloppy way to create a covariance weighted camera cost function with exposed residuals during optimization for stalker
 * objects to copy. NOTE: Adapted colmaps macro expansion to generalize cost-functor to any colmap camera model, since this is
 * still missing for colmaps native covariance-weighted cost creation. In the future, colmap might deliver a CovarianceWeighted
 * cost creation for any camera type but this is the best we have at the moment.
 *
 * @tparam CostFunctor
 * @tparam Args
 * @param stalker attachable residual stalker ptr object which copies calculated residual vectors during iteration
 * @param CovMat
 * @param camera_model_id
 * @param args
 * @return ceres::CostFunction*
 */
template <template <typename> class CostFunctor, typename... Args>
ceres::CostFunction* CreateWeightedCamCostExposedResiduals(const std::shared_ptr<fuhe::ResidualStalker<2>> stalker,
                                                           const Eigen::Matrix<double, 2, 2>& CovMat,
                                                           const colmap::CameraModelId camera_model_id,
                                                           Args&&... args) {
  switch (camera_model_id) {
    using namespace colmap;
#define CAMERA_MODEL_CASE(CameraModel)                                                 \
  case CameraModel::model_id:                                                          \
    return fuhe::cost::WeightedCostExposedResiduals<CostFunctor<CameraModel>>::Create( \
        stalker, CovMat, std::forward<Args>(args)...);                                 \
    break;

    // macro expansion in same style as original colmap
    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

/**
 * @brief Relative pose + scale factor taken and slightly modified from sources:
 - https://github.com/colmap/glomap/blob/main/glomap/estimators/cost_function.h
 - https://github.com/colmap/colmap/blob/682ea9ac4020a143047758739259b3ff04dabe8d/src/colmap/estimators/cost_functions.h#L405
 6 + 1 DoF error between two absolute camera poses (with identical non-real-world scale for the translation) given a relative
 pose measurement from external odometry source with true metric scale. Estimated scale can be used to transform the colmap model
 afterwards with a Sim3 tf. The residual is computed in the frame of camera i. Its first and last three components correspond to
 the rotation and translation errors, respectively.
 * Derivation: i_T_w = ΔT_i·i_T_j_meas·j_T_w
 * where ΔT_i = exp(η_i) is the resjdual in SE(3) and η_i in tangent space.
 * Thus η_i = log(i_T_w·j_T_w⁻¹·j_T_i_measured)
 * Rotation term: ΔR = log(i_R_w·j_R_w⁻¹·j_R_i)
 * Translation term: Δt = scale * i_t_w + i_R_w·j_R_w⁻¹·(j_t_i_measured - scale * j_t_w)
 */
struct ScaleAwareRelativePoseCostFunctor
    : public colmap::AutoDiffCostFunctor<ScaleAwareRelativePoseCostFunctor, 6, 1, 4, 3, 4, 3> {
 public:
  /**
   * @brief Construct a new Scale Aware Relative Pose Cost Functor object
   *
   * @param T_i_from_j measured relative camera pose (j w.r.t to i)
   */
  explicit ScaleAwareRelativePoseCostFunctor(const colmap::Rigid3d& T_i_from_j) : j_from_i_measured_(Inverse(T_i_from_j)) {}

  template <typename T>
  bool operator()(const T* scale,
                  const T* const i_from_world_rotation,
                  const T* const i_from_world_translation,
                  const T* const j_from_world_rotation,
                  const T* const j_from_world_translation,
                  T* residuals_ptr) const {
    // Rotation term: ΔR = log(i_R_w·j_R_w⁻¹·j_R_i)
    const Eigen::Quaternion<T> i_from_j_rotation =
        colmap::EigenQuaternionMap<T>(i_from_world_rotation) * colmap::EigenQuaternionMap<T>(j_from_world_rotation).inverse();
    const Eigen::Quaternion<T> param_from_prior_rotation = i_from_j_rotation * j_from_i_measured_.rotation.cast<T>();
    colmap::EigenQuaternionToAngleAxis(param_from_prior_rotation.coeffs().data(), residuals_ptr);

    // Translation term (with scale estimation):
    // j_t_i_measured - scale*j_t_w -> apply scale onto cam pose translation
    const Eigen::Matrix<T, 3, 1> j_from_i_prior_translation =
        j_from_i_measured_.translation.cast<T>() - colmap::EigenVector3Map<T>(j_from_world_translation) * scale[0];
    Eigen::Map<Eigen::Matrix<T, 3, 1>> param_from_prior_translation(residuals_ptr + 3);
    // Δt = scale*i_t_w + i_R_w·j_R_w⁻¹·(j_t_i_measured - scale*j_t_w )
    param_from_prior_translation =
        colmap::EigenVector3Map<T>(i_from_world_translation) * scale[0] + i_from_j_rotation * j_from_i_prior_translation;

    return true;
  }

 private:
  // relative pose measurement with true scale obtained from external odometry (pose of j w.r.t. i)
  const colmap::Rigid3d j_from_i_measured_;
};

/**
 * @brief Cost functor to estimate extrinsic calibration between two sensor links given a set of relative pose measurements.
 * Estimates pose of target (e.g. cam) w.r.t. reference (e.g. lidar). Makse sure that rel pose segments are time synchronized
 * between sensor links.
 *
 */
struct ExtrinsicCalibCostFunctor : public colmap::AutoDiffCostFunctor<ExtrinsicCalibCostFunctor, 6, 4, 3> {
 public:
  /**
   * @brief Construct a new Extrinsic Calib Cost Functor object
   *
   * @param T_ij_ref 6DoF rel pose measurement from ref sensor (e.g. lidar)
   * @param T_ij_target 6DoF rel pose measurement from target sensor (e.g. camera)
   */
  explicit ExtrinsicCalibCostFunctor(const colmap::Rigid3d& T_ij_ref, const colmap::Rigid3d& T_ij_target)
      : i_from_j_ref_(T_ij_ref), i_from_j_target_(T_ij_target) {}

  template <typename T>
  bool operator()(const T* const q_ref_from_target, const T* const t_ref_from_target, T* residuals_ptr) const {
    // T_ref_from_target (extrinsics)
    colmap::EigenQuaternionMap<T> q_rt(q_ref_from_target);
    colmap::EigenVector3Map<T> t_rt(t_ref_from_target);

    // R_ij_target (predicted) = inv(q_ref_from_target) * q_ij_ref * q_ref_from_target
    Eigen::Quaternion<T> q_ij_pred = q_rt.conjugate() * i_from_j_ref_.rotation.cast<T>() * q_rt;

    // Rotation term: ΔR = log(i_R_j_target · i_R_j_predicted⁻¹)
    const Eigen::Quaternion<T> param_rot_diff = i_from_j_target_.rotation.cast<T>() * q_ij_pred.conjugate();
    colmap::EigenQuaternionToAngleAxis(param_rot_diff.coeffs().data(), residuals_ptr);

    // t_ij_target (predicted) = inv(q_ref_from_target) * t_ij_ref + inv(q_ref_from_target)*(q_ij_ref*t_rt - I*t_rt)
    Eigen::Matrix<T, 3, 1> t_ij_pred =
        q_rt.conjugate() * (i_from_j_ref_.translation.cast<T>() + (i_from_j_ref_.rotation.cast<T>() * t_rt) - t_rt);

    // Translation term
    Eigen::Map<Eigen::Matrix<T, 3, 1>> param_trans_diff(residuals_ptr + 3);
    // Δt = i_t_j_target - i_t_j_predicted
    param_trans_diff = i_from_j_target_.translation.cast<T>() - t_ij_pred;

    return true;
  }

 private:
  // relative pose measurement with true scale obtained from external odometry (pose of j w.r.t. i)
  const colmap::Rigid3d i_from_j_ref_;
  const colmap::Rigid3d i_from_j_target_;
};

/**
 * @brief Taken from colmap::AbsolutePosePositionPriorCostFunctor to avoid the world pose w.r.t. to cam notation. Normal Prior to
 * constrain the translation component of a 6DoF pose
 *
 */
struct TranslationPriorCostFunctor : public colmap::AutoDiffCostFunctor<TranslationPriorCostFunctor, 3, 3> {
 public:
  explicit TranslationPriorCostFunctor(const Eigen::Vector3d& tranlsation_prior, const Eigen::Vector3d& weight)
      : translation_prior_(tranlsation_prior), weight_(weight) {}

  template <typename T>
  bool operator()(const T* const t, T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residuals_ptr);
    residuals = translation_prior_.cast<T>() - colmap::EigenVector3Map<T>(t);
    residuals[0] *= weight_[0];
    residuals[1] *= weight_[1];
    residuals[2] *= weight_[2];
    return true;
  }

 private:
  const Eigen::Vector3d translation_prior_;
  const Eigen::Vector3d weight_;  // inverse cov -> large weight means large influence
};
}  // namespace cost
}  // namespace fuhe
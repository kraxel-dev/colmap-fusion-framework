/**
 * @file rr_utils.cpp
 * @author azuo
 * @brief Helpers to do rerun related tasks, such as convenience functions for converting colmap data to rerun types
 * @version 0.1
 * @date 2025-02-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "fusion_helper/rr_utils.h"

#include "fusion_helper/rr_collection_adapters.h"

std::pair<rerun::Vec3D, rerun::Mat3x3> fuhe::rr_utils::ToRerunPose3D(const colmap::Rigid3d& T, const bool inv) {
  // when required, invert to obtain pose of cam with respect to world.
  const colmap::Rigid3d T_temp = (inv) ? colmap::Inverse(T) : T;

  // downcast to float to match rerun adapter type.
  const Eigen::Vector3f t = T_temp.translation.cast<float>();
  const Eigen::Matrix<float, 3, 3> R = T_temp.rotation.toRotationMatrix().cast<float>();

  return std::pair<rerun::Vec3D, rerun::Mat3x3>(rerun::Vec3D(t.data()), rerun::Mat3x3(R.data()));
}
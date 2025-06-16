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

rerun::Arrows3D fuhe::rr_utils::FrameAxis(const float axis_len) {
  return rerun::Arrows3D::from_vectors({{axis_len, 0.0, 0.0}, {0.0, axis_len, 0.0}, {0.0, 0.0, axis_len}})
      .with_colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}});
}

const std::string fuhe::rr_utils::GetCamPosesName(const colmap::image_t img_id) {
  return "world/cams/cam" + std::to_string(img_id);
}

const std::string fuhe::rr_utils::GetPoints3DName(const bool is_subset) {
  const std::string pts3d_name = (is_subset) ? "world/pts_3D_local_bundle" : "world/pts_3D";
  return pts3d_name;
}

std::string fuhe::rr_utils::GetLabelNameEdge(const colmap::image_t img_id_i, const colmap::image_t img_id_j) {
  return std::string("edge_" + std::to_string(img_id_i) + "_to_" + std::to_string(img_id_j));
}

std::pair<std::string, std::string> fuhe::rr_utils::GetEntityNamesOdomEdge(const colmap::image_t img_id_i,
                                                                           const colmap::image_t img_id_j,
                                                                           const bool is_relative_pose) {
  const std::string source_frame = GetSourceFrameNameOdomEdges(is_relative_pose);

  std::string odom_pose_name = source_frame + "/cam_" + std::to_string(img_id_j);
  std::string edge_i_pred_j_name = source_frame + "/" + rr_utils::GetLabelNameEdge(img_id_i, img_id_j);

  return std::pair<std::string, std::string>(odom_pose_name, edge_i_pred_j_name);
}

std::string fuhe::rr_utils::GetSourceFrameNameOdomEdges(const bool is_relative_pose) {
  std::string source_frame = "world";

  // easy way in viewer to toggle between odometry view (absolute poses vs predicted ones)
  if (is_relative_pose) {
    source_frame += "/00_predicted_odom_poses";
  } else {
    source_frame += "/00_absolute_odom_poses";
  }

  return source_frame;
}

#pragma once

#include <colmap/geometry/rigid3.h>
#include <rerun.hpp>
// FIXME: file name rr to rerun
namespace fuhe {
namespace rr_utils {

std::pair<rerun::Vec3D, rerun::Mat3x3> ToRerunPose3D(const colmap::Rigid3d& T, const bool inv = false);

const std::string GetCamPosesName(const colmap::image_t img_id);
/// get name of points3d entity in rerun. Toggle 'is_subset' to signal that not all points3d are logged (e.g. only pts considerd by active
/// bunde adjustment), the name will changea accordingly
const std::string GetPoints3DName(const bool is_subset = false);

std::string GetLabelNameEdge(const colmap::image_t img_id_i, const colmap::image_t img_id_j);
/// get entity name for (predicted) odometry pose and linestrip odom edge between images
std::pair<std::string, std::string> GetEntityNamesOdomEdge(const colmap::image_t img_id_i,
                                                           const colmap::image_t img_id_j,
                                                           const bool is_relative_pose = true);

}  // namespace rr_utils

}  // namespace fuhe
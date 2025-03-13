#pragma once

#include <colmap/geometry/rigid3.h>
#include <rerun.hpp>
// FIXME: file name rr to rerun
namespace fuhe {
namespace rr_utils {

std::pair<rerun::Vec3D, rerun::Mat3x3> ToRerunPose3D(const colmap::Rigid3d& T, const bool inv = false);

std::string GetLabelNameEdge(const colmap::image_t img_id_i, const colmap::image_t img_id_j);
/// get entity name for (predicted) odometry pose and linestrip edge
std::pair<std::string, std::string> GetEntityNamesOdomEdge(const colmap::image_t img_id_i,
                                                           const colmap::image_t img_id_j,
                                                           const bool is_relative_pose = true);

}  // namespace rr_utils

}  // namespace fuhe
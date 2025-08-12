/**
 * @file rr_utils.h
 * @author kraxel
 * @brief Tiny utils collection for rerun logging such as retunrning unified entity names depending on the modality and data
 * category you want to stream.
 * @version 0.1
 * @date 2025-06-13
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <colmap/geometry/rigid3.h>
#include <rerun.hpp>

namespace fuhe {
namespace rr_utils {

/**
 * @brief Obtain rerun loggable pose from colmap camera pose.
 *
 * @param T 6DoF colmap pose.
 * @param inv invert the forwarded pose. Handy since colmap poses are world w.r.t camera per default.
 * @return std::pair<rerun::Vec3D, rerun::Mat3x3>
 */
std::pair<rerun::Vec3D, rerun::Mat3x3> ToRerunPose3D(const colmap::Rigid3d& T, const bool inv = false);

/**
 * @brief Create 3D arrows that represent a pose in 3D space. Draw it ontop a rerun 3D transform instead of using the transforms
 * own axis to avoid the pesky autoscale of the tf axis when going back in time in the rr viewer.
 *
 * @return rerun::Arrows3D
 */
rerun::Arrows3D FrameAxis(const float axis_len);

const std::string GetCamPosesName(const colmap::image_t img_id);
/// get name of points3d entity in rerun. Toggle 'is_subset' to signal that not all points3d are logged (e.g. only pts considerd
/// by active bunde adjustment), the name will changea accordingly
const std::string GetPoints3DName(const bool is_subset = false);

std::string GetLabelNameEdge(const colmap::image_t img_id_i, const colmap::image_t img_id_j);
/// get entity name for (predicted) odometry pose and linestrip odom edge between images
std::pair<std::string, std::string> GetEntityNamesOdomEdge(const colmap::image_t img_id_i,
                                                           const colmap::image_t img_id_j,
                                                           const bool is_relative_pose = true);

/// get the name of the source entity frame in which the actual odom edges will live under
std::string GetSourceFrameNameOdomEdges(const bool is_relative_pose = true);

}  // namespace rr_utils

}  // namespace fuhe
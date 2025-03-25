#pragma once

#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <colmap/scene/image.h>

namespace fuhe {
namespace types {

/// to store colmap img ids sorted by stamp [sec]
using MapOfImageIdsSec = std::map<const double, colmap::image_t>;

/// key: [uint64 nanosecond timestamp], entry: pose3d
using MapOfPosesNsec =
    std::map<const uint64_t, Eigen::Isometry3d, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3d>>>;

/// key: [dobule second stamp of pose], entry: pose3d
using MapOfPosesSec =
    std::map<const double, Eigen::Isometry3d, std::less<double>, Eigen::aligned_allocator<std::pair<const double, Eigen::Isometry3d>>>;

}  // namespace types
}  // namespace fuhe
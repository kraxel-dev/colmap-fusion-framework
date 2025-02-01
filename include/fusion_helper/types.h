#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>

namespace fuhe {
namespace types {

/// key: [uint64 nanosecond timestamp], entry: pose3d
using MapOfPosesNsec =
    std::map<uint64_t, Eigen::Isometry3d, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3d>>>;

/// key: [dobule second stamp of pose], entry: pose3d
using MapOfPosesSec =
    std::map<const double, Eigen::Isometry3d, std::less<double>, Eigen::aligned_allocator<std::pair<const double, Eigen::Isometry3d>>>;
}  // namespace types
}  // namespace fuhe
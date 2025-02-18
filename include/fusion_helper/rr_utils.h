#pragma once

#include <colmap/geometry/rigid3.h>
#include <rerun.hpp>

namespace fuhe {
namespace rr_utils {

std::pair<rerun::Vec3D, rerun::Mat3x3> ToRerunPose3D(const colmap::Rigid3d& T, const bool inv = false);

}

}  // namespace fuhe
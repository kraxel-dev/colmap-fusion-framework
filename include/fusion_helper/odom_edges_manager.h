#pragma once

#include <map>
#include <memory>
#include <string>

#include "fusion_helper/types.h"
#include <colmap/geometry/rigid3.h>

namespace fuhe {

struct OdomImagesEdge {
  OdomImagesEdge() = default;
  OdomImagesEdge(const double stamp_j,
                 const double time_diff,
                 const colmap::image_t i,
                 const colmap::image_t j,
                 const std::shared_ptr<colmap::Rigid3d> ptr_T_ij);

  double stamp_j = 0.0;                             // timestamp [secs] of image j
  double time_diff = 0.0;                           // time diff [secs] between node j and i
  colmap::image_t i = -1;                           // image id of source node i
  colmap::image_t j = -1;                           // image id of dest node j
  std::shared_ptr<colmap::Rigid3d> ptr_T_ij = nullptr;  // relative pose of j expressed in i measured by external odometry source.
  std::shared_ptr<Eigen::Matrix<double, 6, 6>> cov_ij = nullptr;  // 6x6 covariance of external relative odometry. First 3 entries are rotation and last 3 translation.
};

/**
 * @brief Structure that holds the mapping between relative odometry from external sensor sources and the colmap poses / nodes that they
 * constrain
 *
 */
class OdomEdgesManager {
 public:
  static std::map<const double, OdomImagesEdge> CreateOdomEdgesBetweenImages(const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp,
                                    const fuhe::types::MapOfPosesSec& odom_poses_by_stamp);

 // TODO: remove class if obslote
 private:
  std::string tum_file = "";
  std::shared_ptr<fuhe::types::MapOfPosesSec> odom_poses_by_stamp =
      nullptr;  // absolute poses from external tum file accessible by timestamps
  std::shared_ptr<fuhe::types::MapOfImageIdsSec> img_ids_by_stamp = nullptr;  // image ids of colmap model sorted by stamp

  std::map<const double, const OdomImagesEdge> edges;
};

}  // namespace fuhe

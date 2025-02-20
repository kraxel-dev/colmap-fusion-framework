#pragma once

#include <map>
#include <memory>
#include <string>

#include "fusion_helper/types.h"
#include <colmap/geometry/rigid3.h>

namespace fuhe {
namespace edges {

/**
 * @brief Holds data about an edge that constrains two nodes by an associated realtive pose measurement from an odometry sensor. Plays
 * important role in building the fusion factor graph with odometry constraints as between factors.
 *
 * Node j of each edge should be seen as current node to to which the external absolute odom pose would refer to. The starting edge has node i == j
 *
 * @param i : id of source node (prev image pose)
 * @param j : id of destination node (curr image pose)
 * @param T_ij : ptr to external relative pose of j with respect to i
 */
struct OdomEdge {
  OdomEdge() = default;
  OdomEdge(const double stamp_j,
           const double time_diff,
           const colmap::image_t i,
           const colmap::image_t j,
           const std::shared_ptr<colmap::Rigid3d> T_odom_ij_ptr);

  double stamp_j = 0.0;                                 // timestamp [secs] of image j
  double time_diff = 0.0;                               // time diff [secs] between node j and i
  colmap::image_t i = -1;                               // image id of source node i
  colmap::image_t j = -1;                               // image id of dest node j
  std::shared_ptr<colmap::Rigid3d> T_odom_ij_ptr = nullptr;  // relative pose of j expressed in i measured by external odometry source.
  std::shared_ptr<Eigen::Matrix<double, 6, 6>> cov_ij =
      nullptr;  // 6x6 covariance of external relative odometry. First 3 entries are rotation and last 3 translation.
};

/// type relief
using MapOfOdomEdges = std::map<const double, OdomEdge>;

/**
 * @brief Structure that holds the mapping between relative odometry from external sensor sources and the colmap poses / nodes that they
 * constrain
 *
 */
class OdomEdgesManager {
 public:
  static MapOfOdomEdges CreateOdomEdgesBetweenImages(const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp,
                                                     const fuhe::types::MapOfPosesSec& odom_poses_by_stamp);

  // TODO: remove class if obslote
 private:
  std::string tum_file = "";
  std::shared_ptr<fuhe::types::MapOfPosesSec> odom_poses_by_stamp =
      nullptr;  // absolute poses from external tum file accessible by timestamps
  std::shared_ptr<fuhe::types::MapOfImageIdsSec> img_ids_by_stamp = nullptr;  // image ids of colmap model sorted by stamp

  MapOfOdomEdges edges;
};

}  // namespace edges
}  // namespace fuhe

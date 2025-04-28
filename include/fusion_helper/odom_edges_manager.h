#pragma once

#include <memory>

#include "fusion_helper/types.h"
#include <Eigen/Core>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/geometry/rigid3.h>

namespace fuhe {
namespace edges {

/**
 * @brief Basic structure for a graph edge.
 *
 */
class DataEdge {
 public:
  DataEdge() = default;
  DataEdge(const double curr_stamp, const double time_diff, const colmap::image_t prev_id, const colmap::image_t curr_id);
  virtual ~DataEdge() = default;

  const bool IsValid() const;
  /// check if this edge is the source node of the whole graph (prev and curr id are the same)
  const bool IsSourceNode() const;

  inline const colmap::image_t PrevId() const { return id_i; }
  inline const colmap::image_t CurrId() const { return id_j; }

 protected:
  double stamp_j = -1.0;      // timestamp [secs] of curr image j
  double time_diff = -1.0;    // time diff [secs] between node j and i
  colmap::image_t id_i = -1;  //  image id of source node i
  colmap::image_t id_j = -1;  //  image id of destination node j
};

/**
 * @brief Holds data about an edge constraining two nodes by a relative pose (odometry) measurement. Plays
 * important role in building the fusion factor graph with odometry constraints between image poses.
 *
 * Node j of each edge should be seen as current node to to which the external absolute odom pose would refer to.
 *
 */
class OdometryEdge : public DataEdge {
 public:
  OdometryEdge() = default;
  /**
   * @brief Construct a new Odometry Edge object
   *
   * @param stamp_j time stamp of current image node j
   * @param time_diff time diff between current image node j and source node i
   * @param id_i id of source node i (prev image pose)
   * @param id_j id of destination node j (curr image pose)
   * @param T_odom_ij relative pose from external odometry source constraining j with respect to i
   */
  OdometryEdge(const double stamp_j,
               const double time_diff,
               const colmap::image_t id_i,
               const colmap::image_t id_j,
               const Eigen::Isometry3d T_odom_ij);

  /// obtain relative pose of j (curr) expressed in i (prev) measured by external odometry source
  inline const Eigen::Isometry3d T_i_from_j() const { return T_odom_ij; }
  inline const Eigen::Matrix<double, 6, 6> CovMat_ij() const { return cov_ij; }

 protected:
  // 6x6 covariance of external relative odometry. First 3 entries are rotation and last 3 translation.
  Eigen::Matrix<double, 6, 6> cov_ij = Eigen::Matrix<double, 6, 6>::Identity();
  // relative pose of j expressed in i measured by external odometry source.
  Eigen::Isometry3d T_odom_ij = Eigen::Isometry3d::Identity();
};

/**
 * @brief Virtual edge between two (time) consecutive image ids (prev_i with curr_j) from a colmap model OR a colmap database. Edge should
 * NOT constrain two imgs that are closest in time just because colmap has not registered all imgs yet during mapping process! This acts as
 * main data container to iterate over when constructing a odometry fusion bundle adjustment problem. SequentialImageEdge itself does not
 * represent an edge that will be actively registered in the ceres BA. BUT it can hold an odometry edge that constrains img j by an relative
 * pose to some source node THAT MIGHT NOT necessaririly be prev_i of this image edge!
 *
 * Node j should be seen as current image node. Very first edge of the graph acts as starting node with id i == j.
 *
 */
class SequentialImageEdge : public DataEdge {
 public:
  SequentialImageEdge() = default;

  /**
   * @brief Construct a new Sequential Image Edge object
   *
   * @param curr_stamp timestamp [secs] of curr image j
   * @param time_diff time diff [secs] between node j and i
   * @param prev_id image id of source node i (NOT necessaririly the origin of an attached odom edge)
   * @param curr_id image id of destination node j
   */
  SequentialImageEdge(const double curr_stamp, const double time_diff, const colmap::image_t prev_id, const colmap::image_t curr_id);

  /**
   * @brief Attach relative odometry pose to current image node of this edge.
   *
   * @param odom_edge odometry edge constraining img j with some source node (NOT necessaririly node i of this image edge!)
   */
  inline void AttachOdomEdge(const std::shared_ptr<OdometryEdge>& odom_edge) { this->odom_edge = odom_edge; }
  inline const std::shared_ptr<OdometryEdge> OdomEdge() const { return odom_edge; };

 protected:
  std::shared_ptr<OdometryEdge> odom_edge =
      nullptr;  // relative pose constraining img j with a source node (NOT necessaririly node i of this image edge!)
};

/// type relief
using MapOfOdomEdges = std::map<const double, OdometryEdge>;
using MapOfImageEdges = std::map<const double, SequentialImageEdge>;

/**
 * @brief Create (time) sorted sequence of image nodes from colmap model as main graph data edges. Finds (if availabe), for each current
 * image node j of an image edge, a relative pose reading from 2 consecutive tum file absolute poses. The source node of each odometry edge
 * may lie further back in time then the previous image node (i) of an image edge. NOTE: Please provide ALL images considered in this
 * reconstruction, not only currently registered ones. Otherwise image edges will link imgs that are not truly conescutive, which results in
 * odometry edges that span far away absolute poses.
 *
 * @param img_ids_by_stamp Please provide ids of ALL images considered by the reconstruction, not just currently registered ones.
 * @param odom_poses_by_stamp absolute poses of external odom source from tum file
 * @return fuhe::edges::MapOfImageEdges
 */
fuhe::edges::MapOfImageEdges CreateSequentialImageEdges(const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp,
                                                        const fuhe::types::MapOfPosesSec& odom_poses_by_stamp);
std::shared_ptr<fuhe::edges::MapOfImageEdges> CreateSequentialImageEdgesPtr(const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp,
                                                                            const fuhe::types::MapOfPosesSec& odom_poses_by_stamp);

/// Given the full range of fusion graph edges, subset the odometry edges that are active in the current bundle adjustment problem. An odom
/// edge is only valid, if both img source and destination to this odom are part of the active image set of the BA.
fuhe::edges::MapOfImageEdges SubsetActiveEdges(const colmap::BundleAdjustmentConfig& ba_config,
                                               const fuhe::edges::MapOfImageEdges& sequential_image_edges);
// FIXME: remove class if obslote
// FIXME: rename file if class gets killed
/**
 * @brief Structure that holds the mapping between relative odometry from external sensor sources and the colmap poses / nodes that they
 * constrain
 *
 */
class OdomEdgesManager {
 public:
  /// Build the data structure that holds the mapping between relative odometry from external sensor sources and the colmap images
  static fuhe::edges::MapOfOdomEdges CreateOdomEdgesBetweenImages(const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp,
                                                                  const fuhe::types::MapOfPosesSec& odom_poses_by_stamp);
  /// Lazy overload to return data as shared ptr
  static const std::shared_ptr<fuhe::edges::MapOfOdomEdges> CreateOdomEdgesBetweenImagesPtr(
      const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp, const fuhe::types::MapOfPosesSec& odom_poses_by_stamp);

 private:
  std::string tum_file = "";
  std::shared_ptr<fuhe::types::MapOfPosesSec> odom_poses_by_stamp =
      nullptr;  // absolute poses from external tum file accessible by timestamps
  std::shared_ptr<fuhe::types::MapOfImageIdsSec> img_ids_by_stamp = nullptr;  // image ids of colmap model sorted by stamp

  MapOfOdomEdges edges;
};

}  // namespace edges
}  // namespace fuhe

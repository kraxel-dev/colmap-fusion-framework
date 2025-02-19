#include "fusion_helper/odom_edges_manager.h"

fuhe::OdomImagesEdge::OdomImagesEdge(const double stamp_j,
                                     const double time_diff,
                                     const colmap::image_t i,
                                     const colmap::image_t j,
                                     const std::shared_ptr<colmap::Rigid3d> T_ij)
    : stamp_j{stamp_j}, time_diff{time_diff}, i{i}, j{j}, T_ij{T_ij} {}

std::map<const double, fuhe::OdomImagesEdge> fuhe::OdomEdgesManager::CreateOdomEdgesBetweenImages(
    const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp, const fuhe::types::MapOfPosesSec& odom_poses_by_stamp) {
  int i = 0;                               // image iteration counter
  double curr_img_stamp, prev_stamp = -1;  // stamps for successfully utilized external odoms
  std::map<const double, fuhe::OdomImagesEdge> edges;

  VLOG(2) << "Begin to find tum odometry edges between colmap images!";
  for (const auto pair : img_ids_by_stamp) {
    curr_img_stamp = pair.first;
    colmap::image_t curr_img_id = pair.second;

    VLOG(3) << "Image: " << curr_img_id << " of stamp " << curr_img_stamp;

    // -------------------- First iteration init condition
    if (i == 0) {
      // start graph only if synchronized meas from both sources are availalbe
      if (odom_poses_by_stamp.find(curr_img_stamp) != odom_poses_by_stamp.end()) {
        VLOG(2) << "Found matching pose in tumfile for image! Kickoff edges construction between image and odometry data!";

        // create source node which has has itself as source and destination
        edges[curr_img_stamp] = fuhe::OdomImagesEdge(curr_img_stamp, 0.0, curr_img_id, curr_img_id, std::make_shared<colmap::Rigid3d>());
        VLOG(2) << "This image will be source node with edge to itself!";

        // preparing next iteration
        prev_stamp = curr_img_stamp;
        i++;  // break init loop
      }
      continue;
    }

    // -------------------- Sequential edge construction
    // if no matching odometry between image nodes were found
    if (odom_poses_by_stamp.find(curr_img_stamp) == odom_poses_by_stamp.end()) {
      LOG(WARNING) << "Image without odometry edge! Id: " << curr_img_id;
      edges[curr_img_stamp] = OdomImagesEdge(curr_img_stamp, 0.0, curr_img_id, curr_img_id, nullptr);
      i++;
      continue;
    }

    // -------------------- Add edge constraining 2 images by relative odometry
    const colmap::image_t prev_img_id = img_ids_by_stamp.at(prev_stamp);
    VLOG(3) << "Found matching edge constraining 2 images! Id: " << prev_img_id << " and " << curr_img_id;

    // Get metric relative  pose of j (curr) expressed in i (prev) := i_from_j = world_from_i.inverse() * world_from_j
    const Eigen::Isometry3d T_i_from_j = odom_poses_by_stamp.at(prev_stamp).inverse() * odom_poses_by_stamp.at(curr_img_stamp);
    const colmap::Rigid3d T_ij(Eigen::Quaterniond(T_i_from_j.rotation()), T_i_from_j.translation());
    VLOG(4) << "Relative pose from tumfile: " << T_ij;

    // append edge between images with valid relative pose from external sensor
    edges[curr_img_stamp] =
        OdomImagesEdge(curr_img_stamp, (curr_img_stamp - prev_stamp), prev_img_id, curr_img_id, std::make_shared<colmap::Rigid3d>(T_ij));

    // preparing next iteration
    prev_stamp = curr_img_stamp;
    i++;
  }

  VLOG(2) << "Created nr of edges: " << edges.size();

  return edges;
}

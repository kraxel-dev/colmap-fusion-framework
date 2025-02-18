#include "fusion_helper/odom_edges_manager.h"

fuhe::OdomImagesEdge::OdomImagesEdge(const double stamp_j,
                                     const double time_diff,
                                     const colmap::image_t i,
                                     const colmap::image_t j,
                                     const std::shared_ptr<colmap::Rigid3d> T_ij)
    : OdomImagesEdge() {}

std::map<const double, fuhe::OdomImagesEdge> fuhe::OdomEdgesManager::CreateOdomEdgesBetweenImages(
    const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp, const fuhe::types::MapOfPosesSec& odom_poses_by_stamp) {
  int i = 0;                               // image iteration counter
  double curr_img_stamp, prev_stamp = -1;  // stamps for successfully utilized external odoms
  std::map<const double, OdomImagesEdge> edges;

  for (const auto pair : img_ids_by_stamp) {
    curr_img_stamp = pair.first;
    colmap::image_t curr_img_id = pair.second;

    VLOG(2) << "Iteration for image: " << curr_img_id << " of stamp " << curr_img_stamp;

    // -------------------- First iteration init condition
    if (i == 0) {
      // start graph only if synchronized meas from both sources are availalbe
      if (odom_poses_by_stamp.find(curr_img_stamp) != odom_poses_by_stamp.end()) {
        VLOG(1) << "Found matching pose in tumfile! Kickoff kickoff edges construction between image and odometry data!";

        // create source node which has has itself as source and destination
        edges[curr_img_stamp] = fuhe::OdomImagesEdge(curr_img_stamp, 0.0, curr_img_id, curr_img_id, std::make_shared<colmap::Rigid3d>());

        // preparing next iteration
        prev_stamp = curr_img_stamp;
        i++;  // break init loop
      }
      continue;
    }

    // -------------------- Sequential edge construction
    // if no matching odometry between image nodes were found
    if (odom_poses_by_stamp.find(curr_img_stamp) == odom_poses_by_stamp.end()) {
      VLOG(2) << "Image without odometry edge! Id: " << curr_img_id;
      edges[curr_img_stamp] = OdomImagesEdge(curr_img_stamp, 0.0, curr_img_id, curr_img_id, nullptr);
      i++;
      continue;
    }

    // -------------------- Add edge constraining 2 images by relative odometry
    VLOG(2) << "Found matching tumposes to use as edges constraining 2 images!";

    // Get metric relative  pose of j (curr) expressed in i (prev) := i_from_j = world_from_i.inverse() * world_from_j
    const Eigen::Isometry3d T_i_from_j = odom_poses_by_stamp.at(prev_stamp).inverse() * odom_poses_by_stamp.at(curr_img_stamp);

    // append edge between images with valid relative pose from external sensor
    const colmap::image_t prev_img_id = img_ids_by_stamp.at(prev_stamp);
    edges[curr_img_stamp] = OdomImagesEdge(
        curr_img_stamp,
        (curr_img_stamp - prev_stamp),
        prev_img_id,
        curr_img_id,
        std::make_shared<colmap::Rigid3d>(colmap::Rigid3d(Eigen::Quaterniond(T_i_from_j.rotation()), T_i_from_j.translation())));

    // preparing next iteration
    prev_stamp = curr_img_stamp;
    i++;
  }

  return edges;
}

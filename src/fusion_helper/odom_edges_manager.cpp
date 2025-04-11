#include "fusion_helper/odom_edges_manager.h"

////////////////////////////////////////////////////////////////////////////////
// Data Edge
////////////////////////////////////////////////////////////////////////////////

fuhe::edges::DataEdge::DataEdge(const double curr_stamp,
                                const double time_diff,
                                const colmap::image_t prev_id,
                                const colmap::image_t curr_id)
    : stamp_j{curr_stamp}, time_diff{time_diff}, id_i{prev_id}, id_j{curr_id} {}

const bool fuhe::edges::DataEdge::IsValid() const { return CurrId() != -1 && PrevId() != -1; }

const bool fuhe::edges::DataEdge::IsSourceNode() const { return CurrId() == PrevId(); }

////////////////////////////////////////////////////////////////////////////////
// Odometry Edge
////////////////////////////////////////////////////////////////////////////////

fuhe::edges::OdometryEdge::OdometryEdge(
    const double stamp_j, const double time_diff, const colmap::image_t id_i, const colmap::image_t id_j, const Eigen::Isometry3d T_odom_ij)
    : DataEdge(stamp_j, time_diff, id_i, id_j), T_odom_ij{T_odom_ij} {}

////////////////////////////////////////////////////////////////////////////////
// Sequential Image Edge
////////////////////////////////////////////////////////////////////////////////

fuhe::edges::SequentialImageEdge::SequentialImageEdge(const double curr_stamp,
                                                      const double time_diff,
                                                      const colmap::image_t prev_id,
                                                      const colmap::image_t curr_id)
    : DataEdge(curr_stamp, time_diff, prev_id, curr_id) {}

////////////////////////////////////////////////////////////////////////////////
// Create Sequential Image Edges
////////////////////////////////////////////////////////////////////////////////

fuhe::edges::MapOfImageEdges fuhe::edges::CreateSequentialImageEdges(const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp,
                                                                     const fuhe::types::MapOfPosesSec& odom_poses_by_stamp) {
  VLOG(2) << "Creating sequential image edges for the fusion graph!";
  fuhe::edges::MapOfImageEdges edges;

  double curr_img_stamp, prev_img_stamp = -1;  // stamps for image chain sequence
  double prev_odom_stamp = -1;  // stamp for previously successfully utilized external odoms (is not automatically = prev_img_stamp)
  int odom_edge_counter = 0;    // counter for odometry edges

  // first iteration init condition
  prev_img_stamp = img_ids_by_stamp.begin()->first;  // let origin edge be prev_id == curr_id

  // -------------------- iterate over all images sorted by time
  for (const auto pair : img_ids_by_stamp) {
    curr_img_stamp = pair.first;
    colmap::image_t curr_img_id = pair.second;

    VLOG(4) << "Image: " << curr_img_id << " of stamp " << curr_img_stamp;

    // -------------------- Add sequential edge between images
    auto prev_img_id = img_ids_by_stamp.at(prev_img_stamp);
    edges[curr_img_stamp] = fuhe::edges::SequentialImageEdge(curr_img_stamp, (curr_img_stamp - prev_img_stamp), prev_img_id, curr_img_id);
    prev_img_stamp = curr_img_stamp;

    // -------------------- Attach odometry edge if available
    // whether current img has associated absolute odometry pose in tum file
    if (odom_poses_by_stamp.find(curr_img_stamp) != odom_poses_by_stamp.end()) {
      // whether this absolute pose has some previous pose as source node
      if (prev_odom_stamp == -1) {
        prev_odom_stamp = curr_img_stamp;
        continue;
      }

      // -------------------- Prev source node is available, so we can attach odometry edge
      // Get metric relative  pose of j (curr) expressed in i (prev) := i_from_j = world_from_i.inverse() * world_from_j
      const Eigen::Isometry3d T_i_from_j = odom_poses_by_stamp.at(prev_odom_stamp).inverse() * odom_poses_by_stamp.at(curr_img_stamp);
      VLOG(4) << "Relative pose from tumfile: " << T_i_from_j.matrix();

      // append edge between current image and some source node  with valid relative pose from external sensor
      std::shared_ptr<OdometryEdge> odom_edge = std::make_shared<OdometryEdge>(
          curr_img_stamp, (curr_img_stamp - prev_odom_stamp), img_ids_by_stamp.at(prev_odom_stamp), curr_img_id, T_i_from_j);
      edges.at(curr_img_stamp).AttachOdomEdge(odom_edge);

      odom_edge_counter++;
      prev_odom_stamp = curr_img_stamp;
    }
  }

  VLOG(2) << "Created nr of sequential image edges: " << edges.size();
  VLOG(2) << "Created nr of odometry edges between images: " << odom_edge_counter;
  return edges;
}

std::shared_ptr<fuhe::edges::MapOfImageEdges> fuhe::edges::CreateSequentialImageEdgesPtr(
    const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp, const fuhe::types::MapOfPosesSec& odom_poses_by_stamp) {
  return std::make_shared<fuhe::edges::MapOfImageEdges>(fuhe::edges::CreateSequentialImageEdges(img_ids_by_stamp, odom_poses_by_stamp));
}

fuhe::edges::MapOfImageEdges fuhe::edges::SubsetActiveEdges(const colmap::BundleAdjustmentConfig& ba_config,
                                                            const fuhe::edges::MapOfImageEdges& sequential_image_edges) {
  VLOG(2) << "Taking subset of odometry edges that are active in current BA!";
  fuhe::edges::MapOfImageEdges active_edges;

  // subset of images that are active for this BA
  const std::unordered_set<colmap::image_t> active_images = ba_config.Images();

  // iterate over all sequential image edges in model and only keep odom edges that are active in the current BA
  for (const auto& [stamp, img_edge] : sequential_image_edges) {
    // check if external odom is available for current image node
    if (!img_edge.OdomEdge()) {
      continue;
    }

    // filter odometry edges that are not part of the active image set of this BA
    if (active_images.find(img_edge.CurrId()) == active_images.end()) {
      continue;
    }

    // filter odometry edges whose pose source node is not part of the active image set of this BA
    if (active_images.find(img_edge.OdomEdge()->PrevId()) == active_images.end()) {
      continue;
    }

    active_edges[stamp] = img_edge;
  }

  VLOG(2) << "Created nr of active edges: " << active_edges.size();
  return active_edges;
}

fuhe::edges::MapOfOdomEdges fuhe::edges::OdomEdgesManager::CreateOdomEdgesBetweenImages(
    const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp, const fuhe::types::MapOfPosesSec& odom_poses_by_stamp) {
  int i = 0;                               // image iteration counter
  double curr_img_stamp, prev_stamp = -1;  // stamps for successfully utilized external odoms
  fuhe::edges::MapOfOdomEdges edges;

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
        edges[curr_img_stamp] = fuhe::edges::OdometryEdge(curr_img_stamp, 0.0, curr_img_id, curr_img_id, Eigen::Isometry3d::Identity());
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
      edges[curr_img_stamp] = OdometryEdge(curr_img_stamp, 0.0, -1, curr_img_id, Eigen::Isometry3d::Identity());
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
    edges[curr_img_stamp] = OdometryEdge(curr_img_stamp, (curr_img_stamp - prev_stamp), prev_img_id, curr_img_id, T_i_from_j);

    // preparing next iteration
    prev_stamp = curr_img_stamp;
    i++;
  }

  VLOG(2) << "Created nr of edges: " << edges.size();

  return edges;
}

const std::shared_ptr<fuhe::edges::MapOfOdomEdges> fuhe::edges::OdomEdgesManager::CreateOdomEdgesBetweenImagesPtr(
    const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp, const fuhe::types::MapOfPosesSec& odom_poses_by_stamp) {
  return std::make_shared<fuhe::edges::MapOfOdomEdges>(
      fuhe::edges::OdomEdgesManager::CreateOdomEdgesBetweenImages(img_ids_by_stamp, odom_poses_by_stamp));
}
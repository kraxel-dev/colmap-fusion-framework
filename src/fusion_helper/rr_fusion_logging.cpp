#include "fusion_helper/rr_fusion_logging.h"

#include "fusion_helper/rr_collection_adapters.h"
#include "fusion_helper/rr_utils.h"
#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>

void rrfuse::LogCamPose(const std::shared_ptr<rerun::RecordingStream> rec,
                        const std::shared_ptr<rerun::Pinhole> rrpinhole,
                        const colmap::Image& img) {
  std::string cam_name = "world/cam" + std::to_string(img.ImageId());

  // match rerun adapter type.
  std::pair<rerun::Vec3D, rerun::Mat3x3> T = fuhe::rr_utils::ToRerunPose3D(img.CamFromWorld(), true);

  // log camera pose to rerun. data of t and R may go out of scope, but as rerun object they shall live on
  rec->log(cam_name, rerun::Transform3D(T.first, T.second).with_axis_length(rerun::components::AxisLength(rrfuse::AXIS_LENGTH_PINHOLE)));

  /*
  NOTE: For som weird reaseon, child entity display does not work in rerun viewer once an entity has been established a pinhole.
  Log pinhole as child entity of actual pose as workaround for now.
   */
  // establish camera for logged pose under same name as pose
  rec->log(cam_name + "/pinhole",
           rerun::Transform3D()
               .with_relation(rerun::components::TransformRelation::ParentFromChild)
               .with_axis_length(rerun::components::AxisLength(rrfuse::AXIS_LENGTH_PINHOLE)));
  rec->log(cam_name + "/pinhole", rerun::Pinhole(*rrpinhole));
  // add a point to slap a label to the image pose
  // rec->log(cam_name + "/tf_label",
  //          rerun::Transform3D()
  //              .with_relation(rerun::components::TransformRelation::ParentFromChild)
  //              .with_axis_length(rerun::components::AxisLength(rrfuse::AXIS_LENGTH)));
  // rec->log(cam_name + "/tf_label",
  //          rerun::Points3D({{0.05f, -0.1f, 0.0f}}).with_labels(rerun::components::Text("cam" + std::to_string(img.ImageId()))));
}

void rrfuse::LogCamPoints3D(const std::shared_ptr<rerun::RecordingStream> rec,
                            const colmap::Image& img,
                            const std::vector<colmap::Point3D>& pts3D) {
  std::vector<rerun::Position3D> rr_pts3D;
  rr_pts3D.reserve(pts3D.size());

  // iterate over all colmap 3d points and place them in rr vector with correct type
  std::transform(pts3D.begin(), pts3D.end(), std::back_inserter(rr_pts3D), [](const colmap::Point3D& pt) {
    return rerun::Position3D(pt.xyz.x(), pt.xyz.y(), pt.xyz.z());
  });

  rec->log("world/pts_" + std::to_string(img.ImageId()), rerun::Points3D(rr_pts3D));
}

void rrfuse::ClearAllCamPoints3D(const std::shared_ptr<rerun::RecordingStream> rec,
                                 const std::unordered_map<colmap::camera_t, colmap::Image>& images) {
  for (auto& [_, img] : images) {
    // clear 3D points associated to cam
    rec->log("world/pts_" + std::to_string(img.ImageId()), rerun::Points3D::clear_fields());
  }
}

void rrfuse::LogPoint3D(const std::shared_ptr<rerun::RecordingStream> rec, const colmap::point3D_t& pt3d_id, const Eigen::Vector3d& xyz) {
  std::string pt3d_name = "world/point3d/" + std::to_string(pt3d_id);
  rerun::Position3D pos(xyz.x(), xyz.y(), xyz.z());

  rec->log(pt3d_name, rerun::archetypes::Points3D(pos));
}

void rrfuse::LogOdometryEdge(const std::shared_ptr<rerun::RecordingStream> rec,
                             const colmap::Rigid3d& T_ij_odom,
                             const colmap::Image& img_i,
                             const colmap::Image& img_j,
                             const bool is_odom_a_relpose) {
  // rerun naming stuff
  std::string pred_cam_name = fuhe::rr_utils::GetEntityNamesOdomEdge(img_i.ImageId(), img_j.ImageId(), is_odom_a_relpose).first;
  std::string edge_i_pred_j_name = fuhe::rr_utils::GetEntityNamesOdomEdge(img_i.ImageId(), img_j.ImageId(), is_odom_a_relpose).second;

  colmap::Rigid3d T_odom_w_j;  // absolute pose odom associated with node j with respect to world

  if (is_odom_a_relpose) {
    // if odom pose is provided as relative pose between i and j (typical case)
    T_odom_w_j = colmap::Inverse(img_i.CamFromWorld()) * T_ij_odom;  // T_w_j_predicted = T_w_from_i * T_odometry_i_from_j
  } else {
    // if odom pose is provided as absolute pose (case only for logging)
    T_odom_w_j = T_ij_odom;
  }

  // obtain absolute poses of colmap nodes
  const rerun::Vec3D t_i = fuhe::rr_utils::ToRerunPose3D(colmap::Inverse(img_i.CamFromWorld())).first;
  const rerun::Vec3D t_j = fuhe::rr_utils::ToRerunPose3D(colmap::Inverse(img_j.CamFromWorld())).first;
  // convert odom pose to rerun type
  std::pair<rerun::Vec3D, rerun::Mat3x3> T_ij_pred = fuhe::rr_utils::ToRerunPose3D(T_odom_w_j);

  // line strip highlighting the edges between states and predicted pose
  std::vector<rerun::Vec3D> line_segments;   // vector containg xyz points of line semgents
  line_segments.push_back(t_i);              // origin in zero (parent entity is i cam pose)
  line_segments.push_back(T_ij_pred.first);  // pass predicted pose as control point
  line_segments.push_back(t_j);              // end line strip in pose of image j
  rerun::LineStrip3D line_strip(line_segments);

  // log predicted camera pose to rerun.
  rec->log(pred_cam_name,
           rerun::Transform3D::from_translation_mat3x3(T_ij_pred.first, T_ij_pred.second)
               .with_axis_length(rerun::components::AxisLength(rrfuse::AXIS_LENGTH_ODOM)));
  // add a point to slap a label to the image pose
  // rec->log(pred_cam_name + "/tf_label",
  //          rerun::Transform3D()
  //              .with_relation(rerun::components::TransformRelation::ParentFromChild)
  //              .with_axis_length(rerun::components::AxisLength(rrfuse::AXIS_LENGTH)));
  // rec->log(
  //     pred_cam_name + "/tf_label",
  //     rerun::Points3D({{0.05f, 0.1f, 0.0f}}).with_labels(rerun::components::Text("/cam" + std::to_string(img_j.ImageId()) +
  //     "_predicted")));
  // draw ellipsoid around precited pose
  // rec->log(pred_cam_name + "/ellipse",
  //          rerun::Ellipsoids3D::from_centers_and_half_sizes({{0.0f, 0.0f, 0.0f}}, {{0.03f, 0.01f, 0.04f}})
  //              .with_colors({rerun::Rgba32(255, 255, 0)})
  //              .with_labels(rerun::components::Text("cam" + std::to_string(img_j.ImageId()) + "_predicted")));

  // log linestrips connecting the factors
  rec->log(edge_i_pred_j_name, rerun::LineStrips3D(line_strip));
  //  rerun::LineStrips3D(line_strip).with_labels(rerun::Text(fuhe::rr_utils::GetLabelNameEdge(img_i.ImageId(), img_j.ImageId()))));
}

void rrfuse::LogTotalFactorCost(const std::shared_ptr<rerun::RecordingStream> rec,
                                const std::string& factor_type,
                                const double total_cost) {
  std::string cost_name = "total_" + factor_type + "_cost";
  rec->log(cost_name, rerun::Scalar(total_cost));
}

void rrfuse::LogReconstruction(const std::shared_ptr<rerun::RecordingStream> rec,
                               const std::shared_ptr<rerun::Pinhole> rrpinhole,
                               const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                               const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D) {
  // -------------------- Images
  const std::unordered_map<colmap::camera_t, colmap::Image> images_copy = images;
  // log all registered images
  for (auto& [_, img] : images_copy) {
    rrfuse::LogCamPose(rec, rrpinhole, img);
  }

  // -------------------- Tracks
  // clear rerun 3d points
  rec->log("world/pts_3D", rerun::Points3D::clear_fields());  // FIXME: decide if clearing all points neccessary to kill old ones

  std::vector<rerun::Position3D> points;
  // log all 3d points
  for (auto& [_, pt3D] : points3D) {
    // Should actually be `track.observations.size() < options_.min_num_view_per_track`.
    if (pt3D.track.Length() < 2) continue;

    const Eigen::Vector3f xyz = pt3D.xyz.cast<float>();
    points.emplace_back(xyz.x(), xyz.y(), xyz.z());
    // colors.emplace_back(track.color[0], track.color[1], track.color[2]);
  }
  rec->log("world/pts_3D", rerun::Points3D(points));
  // rec->log("world/pts_3D", rerun::Points3D().update_fields());
}

void rrfuse::LogReconstructionSorted(const std::shared_ptr<rerun::RecordingStream> rec,
                                     const std::shared_ptr<rerun::Pinhole> rrpinhole,
                                     const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                                     const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                                     const fuhe::edges::MapOfOdomEdges& odom_edges) {
  // -------------------- Images
  // log all registered images
  for (auto& [_, edge] : odom_edges) {
    rrfuse::LogCamPose(rec, rrpinhole, images.at(edge.j));
  }

  // -------------------- Tracks
  // clear rerun 3d points
  rec->log("world/pts_3D", rerun::Points3D::clear_fields());

  std::vector<rerun::Position3D> points;
  // log all 3d points
  for (auto& [_, pt3D] : points3D) {
    // Should actually be `track.observations.size() < options_.min_num_view_per_track`.
    if (pt3D.track.Length() < 2) continue;

    auto xyz = pt3D.xyz;
    points.emplace_back(xyz.x(), xyz.y(), xyz.z());
    // colors.emplace_back(track.color[0], track.color[1], track.color[2]);
  }
  rec->log("world/pts_3D", rerun::Points3D(points));
  // rec->log("world/pts_3D", rerun::Points3D().update_fields());
}

void rrfuse::LogOdometryEdges(const std::shared_ptr<rerun::RecordingStream> rec,
                              const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                              const std::map<const double, fuhe::edges::OdomEdge> edges) {
  const bool log_odom_as_predicted_pose = true;  // do not log odometry measurement as absolute pose
  // -------------------- Edges
  // log all registered images
  for (auto& [_, edge] : edges) {
    // skip source node
    if ((edge.i == edge.j)) {
      VLOG(4) << "Source node detected! Skip logging this one! ";
      continue;
    } else if (edge.T_odom_ij_ptr == nullptr) {
      LOG(WARNING) << "Edge between images without valid relative odometry detected! Id: " << edge.j;
    }

    VLOG(5) << "Rerun logging relpose factor with rigid: " << *edge.T_odom_ij_ptr;
    rrfuse::LogOdometryEdge(rec, *(edge.T_odom_ij_ptr), images.at(edge.i), images.at(edge.j), log_odom_as_predicted_pose);
  }
}

void rrfuse::LogOdometryEdgesAsTrajectory(const std::shared_ptr<rerun::RecordingStream> rec,
                                          const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                                          const std::map<const double, fuhe::edges::OdomEdge> edges,
                                          const bool log_traj_as_linestrip) {
  const bool log_odom_as_absolute_pose = true;  // log odometry measurement as absolute pose

  colmap::Rigid3d T_world_from_odom =
      colmap::Rigid3d();  // absolute odometry pose, incremented per interation with each consecutive relpose

  std::vector<rerun::Vec3D> line_segments;  // vector containg xyz points of line semgents
  // -------------------- Edges
  // log all registered images
  for (auto& [_, edge] : edges) {
    // skip source node
    if ((edge.i == edge.j)) {
      VLOG(4) << "Source node detected!";
      // Grabbing first colmap pose as origin of tum trajectory
      T_world_from_odom = colmap::Inverse(images.at(edge.j).CamFromWorld());

      if (log_traj_as_linestrip) {
        const rerun::Vec3D t_j = fuhe::rr_utils::ToRerunPose3D(T_world_from_odom, false).first;  // pos of j img in world
        // line strip connecting odometry poses
        line_segments.push_back(t_j);  // node i as origin
      }

      continue;

    } else if (edge.T_odom_ij_ptr == nullptr) {
      LOG(WARNING) << "Edge between images without valid relative odometry detected! Id: " << edge.j;
    }

    // increment rel pose to obtain absolute pose for current node
    T_world_from_odom = T_world_from_odom * *(edge.T_odom_ij_ptr);
    VLOG(5) << "Rerun logging: " << T_world_from_odom;
    rrfuse::LogOdometryEdge(rec, T_world_from_odom, images.at(edge.i), images.at(edge.j), !log_odom_as_absolute_pose);

    if (log_traj_as_linestrip) {
      const rerun::Vec3D t_j = fuhe::rr_utils::ToRerunPose3D(T_world_from_odom, false).first;  // pos of j img in world
      // line strip connecting odometry poses
      line_segments.push_back(t_j);  // node i as origin
    }
  }

  if (log_traj_as_linestrip) {
    rerun::LineStrip3D line_strip(line_segments);
    rec->log("world/odometry", rerun::LineStrips3D(line_strip).with_labels(rerun::components::Text("Odometry")));
  }
}

void rrfuse::ClearAllOdometryEdges(const std::shared_ptr<rerun::RecordingStream> rec) {
  rec->log("/", rerun::LineStrips3D::clear_fields());
}

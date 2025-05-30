#include "fusion_helper/rr_fusion_logging.h"

#include "fusion_helper/col_utils.h"
#include "fusion_helper/rr_collection_adapters.h"
#include "fusion_helper/rr_utils.h"
#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>

namespace fuhe {

void rrfuse::LogInfo(const std::shared_ptr<rerun::RecordingStream> rec, const std::string& msg) {
  rec->log("logs", rerun::TextLog(msg).with_level(rerun::TextLogLevel::Info));
}

void rrfuse::LogCamPose(const std::shared_ptr<rerun::RecordingStream> rec,
                        const std::shared_ptr<rerun::Pinhole> rrpinhole,
                        const colmap::Image& img,
                        const bool highlight) {
  std::string cam_name = rr_utils::GetCamPosesName(img.ImageId());

  // match rerun adapter type.
  std::pair<rerun::Vec3D, rerun::Mat3x3> T = fuhe::rr_utils::ToRerunPose3D(img.CamFromWorld(), /*inv=*/true);

  // log camera pose to rerun. data of t and R may go out of scope, but as rerun object they shall live on
  rec->log(cam_name, rerun::Transform3D(T.first, T.second));

  /*
  NOTE: For som weird reaseon, child entity display does not work in rerun viewer once an entity has been established a pinhole.
  Log pinhole as child entity of actual pose as workaround for now.
   */
  rec->log(cam_name + "/pinhole", rerun::Pinhole(*rrpinhole));
  // // establish camera for logged pose under same name as pose
  // rec->log(cam_name + "/pinhole",
  //          rerun::Transform3D()
  //              .with_relation(rerun::components::TransformRelation::ParentFromChild)
  //              .with_axis_length(rerun::components::AxisLength(rrfuse::AXIS_LENGTH_PINHOLE)));
  // add a point to slap a label to the image pose
  // rec->log(cam_name + "/tf_label",
  //          rerun::Transform3D()
  //              .with_relation(rerun::components::TransformRelation::ParentFromChild)
  //              .with_axis_length(rerun::components::AxisLength(rrfuse::AXIS_LENGTH)));
  // rec->log(cam_name + "/tf_label",
  //          rerun::Points3D({{0.05f, -0.1f, 0.0f}}).with_labels(rerun::components::Text("cam" + std::to_string(img.ImageId()))));

  if (highlight) {
    // log bounding box around camera pose
    const float ax_length = rr_utils::IMG_PLANE_DIST / 2.6f;
    rec->log(cam_name + "/bb", rerun::Boxes3D::from_half_sizes({{ax_length, ax_length, ax_length}}));
  }
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

void rrfuse::LogPoints3D(const std::shared_ptr<rerun::RecordingStream> rec,
                         const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                         const bool is_subset,
                         const bool ignore_far_away_points) {
  // clear rerun 3d points
  rec->log(rr_utils::GetPoints3DName(is_subset),
           rerun::Points3D::clear_fields());  // FIXME: decide if clearing all points neccessary to kill old ones

  std::vector<rerun::Position3D> points;
  // log all 3d points
  for (auto& [_, pt3D] : points3D) {
    // Should actually be `track.observations.size() < options_.min_num_view_per_track`.
    if (pt3D.track.Length() < 2) continue;

    const Eigen::Vector3f xyz = pt3D.xyz.cast<float>();
    if (ignore_far_away_points) {
      // ignore points that are outside the bounding box of the camera
      if (std::abs(xyz.x()) > rr_utils::XY_BOUND || std::abs(xyz.y()) > rr_utils::XY_BOUND || std::abs(xyz.z()) > rr_utils::Z_BOUND) {
        continue;
      }
    }
    points.emplace_back(xyz.x(), xyz.y(), xyz.z());
    // colors.emplace_back(track.color[0], track.color[1], track.color[2]);
  }

  rec->log(rr_utils::GetPoints3DName(is_subset), rerun::Points3D(points));
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

  // -------------------- log predicted camera pose to rerun.
  // transform without visible axis to propagate correct pose
  rec->log(pred_cam_name, rerun::Transform3D::from_translation_mat3x3(T_ij_pred.first, T_ij_pred.second));
  // draw coord frame axis onto predicted pose for visiblity
  rec->log(pred_cam_name + "/frame", rr_utils::FrameAxis());

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
                               const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                               const bool is_subset) {
  // -------------------- Images
  const std::unordered_map<colmap::camera_t, colmap::Image> images_copy = images;
  // log all registered images
  for (auto& [_, img] : images_copy) {
    rrfuse::LogCamPose(rec, rrpinhole, img, /*highlight=*/is_subset);
  }

  // -------------------- Tracks
  rrfuse::LogPoints3D(rec, points3D, is_subset, /*ignore_far_away_points=*/true);
}

void rrfuse::LogActivBundle(const std::shared_ptr<rerun::RecordingStream> rec,
                            const std::shared_ptr<rerun::Pinhole> rrpinhole,
                            const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                            const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                            const colmap::BundleAdjustmentConfig* ba_config,
                            const bool highlight_cams) {
  // -------------------- Images
  std::unordered_map<colmap::image_t, colmap::Image> active_imgs;       // active images in current BA problem
  std::unordered_map<colmap::point3D_t, colmap::Point3D> active_pts3D;  // active 3d points in current BA problem
  // populated BA config knows which images and points are considered for this BA problem. log only those
  col_utils::ImagesAndPointsInActiveBA(*ba_config, images, points3D, active_imgs, active_pts3D);
  rrfuse::LogReconstruction(rec, rrpinhole, active_imgs, active_pts3D, highlight_cams);
}

void rrfuse::ClearActiveBundle(const std::shared_ptr<rerun::RecordingStream> rec, const std::unordered_set<colmap::camera_t>& cam_ids) {
  // clear rerun 3d points
  rec->log(rr_utils::GetPoints3DName(/*is_subset=*/true), rerun::Points3D::clear_fields());

  // clear pinholes highlighted with bounding boxes in local bundle
  for (auto const id : cam_ids) {
    std::string cam_name = rr_utils::GetCamPosesName(id);
    rec->log(cam_name + "/bb", rerun::Boxes3D::clear_fields());
  }
}

void rrfuse::LogOdometryEdges(const std::shared_ptr<rerun::RecordingStream> rec,
                              const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                              const edges::MapOfImageEdges graph_data_edges) {
  const bool log_odom_as_predicted_pose = true;  // do not log odometry measurement as absolute pose
  // -------------------- Edges
  // log all registered images
  for (auto& [_, img_edge] : graph_data_edges) {
    // skip image not entailed by selected images
    if (images.find(img_edge.CurrId()) == images.end()) {
      continue;
    }

    // skip image without odometry constraint
    if (!img_edge.OdomEdge()) {
      continue;
    }

    auto odom_edge = img_edge.OdomEdge();

    // skip relative poses whose source node is not entailed in selected images
    if (images.find(odom_edge->PrevId()) == images.end()) {
      continue;
    }

    // obtain relative pose from odometry edge
    const colmap::Rigid3d T_ij_rigid =
        colmap::Rigid3d(Eigen::Quaterniond(odom_edge->T_i_from_j().rotation()), odom_edge->T_i_from_j().translation());
    VLOG(5) << "Rerun logging relpose factor with rigid: " << T_ij_rigid;

    rrfuse::LogOdometryEdge(rec, T_ij_rigid, images.at(odom_edge->PrevId()), images.at(odom_edge->CurrId()), log_odom_as_predicted_pose);
  }
}

void rrfuse::LogOdometryEdgesAsTrajectory(const std::shared_ptr<rerun::RecordingStream> rec,
                                          const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                                          const edges::MapOfImageEdges graph_data_edges,
                                          const bool log_traj_as_linestrip) {
  const bool log_odom_as_absolute_pose = true;  // log odometry measurement as absolute pose

  colmap::Rigid3d T_world_from_odom =
      colmap::Rigid3d();  // absolute odometry pose, incremented per interation with each consecutive relpose

  std::vector<rerun::Vec3D> line_segments;  // vector containg xyz points of line semgents
  bool is_init = true;                      // for fetching origin absolute pose
  // -------------------- Edges
  // log all registered images
  for (auto& [_, edge] : graph_data_edges) {
    // skip source node
    if (is_init) {
      VLOG(4) << "Source node detected!";

      // Grabbing first colmap pose as origin of tum trajectory
      T_world_from_odom = colmap::Inverse(images.at(edge.CurrId()).CamFromWorld());

      if (log_traj_as_linestrip) {
        const rerun::Vec3D t_j = fuhe::rr_utils::ToRerunPose3D(T_world_from_odom, false).first;  // pos of j img in world
        // line strip connecting odometry poses
        line_segments.push_back(t_j);  // node i as origin
      }

      continue;

    } else if (is_init) {
    }

    // skip if no viable odometry edge availabe for this image
    if (!edge.OdomEdge()) {
      continue;
    }

    auto odom_edge = edge.OdomEdge();

    // whether its the very first odometry edge we received
    if (is_init) {
      VLOG(4) << "Source node detected!";

      // skip if pose origin is not entailed by current image set
      if (images.find(odom_edge->PrevId()) == images.end()) {
        continue;
      }

      // Grabbing first colmap pose as origin of tum trajectory
      T_world_from_odom = colmap::Inverse(images.at(odom_edge->PrevId()).CamFromWorld());

      if (log_traj_as_linestrip) {
        const rerun::Vec3D t_j = fuhe::rr_utils::ToRerunPose3D(T_world_from_odom, false).first;  // pos of j img in world
        // line strip connecting odometry poses
        line_segments.push_back(t_j);  // node i as origin
      }
      is_init = false;
    }

    const colmap::Rigid3d T_ij_rigid =
        colmap::Rigid3d(Eigen::Quaterniond(odom_edge->T_i_from_j().rotation()), odom_edge->T_i_from_j().translation());
    // increment rel pose to obtain absolute pose for current node
    T_world_from_odom = T_world_from_odom * T_ij_rigid;
    VLOG(5) << "Rerun logging: " << T_world_from_odom;
    rrfuse::LogOdometryEdge(
        rec, T_world_from_odom, images.at(odom_edge->PrevId()), images.at(odom_edge->CurrId()), !log_odom_as_absolute_pose);

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
  rec->log("/world/00_predicted_odom_poses/", rerun::archetypes::Clear(true));
  rec->log("/world/00_absolute_odom_poses/", rerun::archetypes::Clear(true));
}

}  // namespace fuhe
#include "high_level_fusion/rerun_interface.h"

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
  rec->log(cam_name, rerun::Transform3D(T.first, T.second).with_axis_length(rerun::components::AxisLength(0.25f)));

  /*
  NOTE: For som weird reaseon, child entity display does not work in rerun viewer once an entity has been established a pinhole.
  Log pinhole as child entity of actual pose as workaround for now.
   */
  // establish camera for logged pose under same name as pose
  rec->log(cam_name + "/pinhole",
           rerun::Transform3D()
               .with_relation(rerun::components::TransformRelation::ParentFromChild)
               .with_axis_length(rerun::components::AxisLength(0.25f)));
  rec->log(cam_name + "/pinhole", rerun::Pinhole(*rrpinhole));
  // add a point to slap a label to the image pose
  rec->log(cam_name + "/tf_label",
           rerun::Transform3D()
               .with_relation(rerun::components::TransformRelation::ParentFromChild)
               .with_axis_length(rerun::components::AxisLength(0.25f)));
  rec->log(cam_name + "/tf_label",
           rerun::Points3D({{0.0f, 0.0f, 0.0f}}).with_labels(rerun::components::Text("cam" + std::to_string(img.ImageId()))));
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

void rrfuse::LogRelPoseFactor(const std::shared_ptr<rerun::RecordingStream> rec,
                              const colmap::Rigid3d& T_ij_odom,
                              const colmap::Image& img_i,
                              const colmap::Image& img_j) {
  // obtain relative pose of node j with respect to node i from current states in images (non measurements)
  const colmap::Rigid3d T_ij = img_i.CamFromWorld() * colmap::Inverse(img_j.CamFromWorld());  // T_i_from_j = T_iw * T_wj

  // rerun naming stuff
  const std::string source_cam_name = "world/cam" + std::to_string(img_i.ImageId());
  const std::string pred_cam_name = source_cam_name + "/cam" + std::to_string(img_j.ImageId()) + "_predicted";
  const std::string edges_i_pred_j = source_cam_name + "/edges_" + std::to_string(img_j.ImageId());

  // obtain predicted rel pose and actual rel pose between states
  const rerun::Vec3D t_ij = fuhe::rr_utils::ToRerunPose3D(T_ij).first;
  std::pair<rerun::Vec3D, rerun::Mat3x3> T_ij_pred = fuhe::rr_utils::ToRerunPose3D(T_ij_odom, false);

  // line strip highlighting the edges between states and predicted pose
  std::vector<rerun::Vec3D> line_segments;   // vector containg xyz points of line semgents
  line_segments.push_back(rerun::Vec3D());   // origin in zero (parent entity is i cam pose)
  line_segments.push_back(T_ij_pred.first);  // pass predicted pose as control point
  line_segments.push_back(t_ij);             // end line strip in pose of image j
  rerun::LineStrip3D line_strip(line_segments);

  // log predicted camera pose to rerun.
  rec->log(pred_cam_name,
           rerun::Transform3D::from_translation_mat3x3(T_ij_pred.first, T_ij_pred.second)
               .with_relation(rerun::components::TransformRelation::ParentFromChild)
               .with_axis_length(rerun::components::AxisLength(0.25f)));
  // add a point to slap a label to the image pose
  rec->log(pred_cam_name + "/tf_label",
           rerun::Transform3D()
               .with_relation(rerun::components::TransformRelation::ParentFromChild)
               .with_axis_length(rerun::components::AxisLength(0.25f)));
  rec->log(pred_cam_name + "/tf_label",
           rerun::Points3D({{0.0f, 0.0f, 0.0f}}).with_labels(rerun::components::Text("/cam" + std::to_string(img_j.ImageId()) + "_predicted")));
  // draw ellipsoid around precited pose
  // rec->log(pred_cam_name + "/ellipse",
  //          rerun::Ellipsoids3D::from_centers_and_half_sizes({{0.0f, 0.0f, 0.0f}}, {{0.03f, 0.01f, 0.04f}})
  //              .with_colors({rerun::Rgba32(255, 255, 0)})
  //              .with_labels(rerun::components::Text("cam" + std::to_string(img_j.ImageId()) + "_predicted")));

  // log linestrips connecting the factors
  rec->log(edges_i_pred_j, rerun::LineStrips3D(line_strip));
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
  rec->log("world/pts_3D", rerun::Points3D::clear_fields());

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
                                     const fuhe::types::MapOfImageIdsSec& ids_by_stamp) {
  // -------------------- Images
  // log all registered images
  for (auto& [_, id] : ids_by_stamp) {
    rrfuse::LogCamPose(rec, rrpinhole, images.at(id));
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
                              const std::map<const double, fuhe::OdomImagesEdge> edges) {
  // -------------------- Images
  // log all registered images
  for (auto& [_, edge] : edges) {
    // skip source node
    if ((edge.i == edge.j)) {
      VLOG(4) << "Source node detected! Skip logging this one! ";
      continue;

    } else if (edge.T_ij == nullptr) {
      LOG(WARNING) << "Edge between images without valid relative odometry detected! Id: " << edge.j;
    }

    VLOG(5) << "Rerun logging relpose factor with rigid: " << *edge.T_ij;
    rrfuse::LogRelPoseFactor(rec, *(edge.T_ij), images.at(edge.i), images.at(edge.j));
  }
}

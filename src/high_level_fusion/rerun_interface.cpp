#include "high_level_fusion/rerun_interface.h"

#include "fusion_helper/rr_collection_adapters.h"
#include "fusion_helper/rr_utils.h"
#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>

void rrfuse::LogCamPose(const std::shared_ptr<rerun::RecordingStream>& rec,
                        const std::shared_ptr<rerun::Pinhole> rrpinhole,
                        const colmap::Image& img) {
  std::string cam_name = "world/cam" + std::to_string(img.ImageId());

  // match rerun adapter type.
  std::pair<rerun::Vec3D, rerun::Mat3x3> T = fuhe::rr_utils::ToRerunPose3D(img.CamFromWorld(), true);

  // log camera pose to rerun. data of t and R may go out of scope, but as rerun object they shall live on
  rec->log(cam_name, rerun::Transform3D(T.first, T.second));

  // establish camera for logged pose under same name as pose
  /* NOTE: For som weird reaseon, child entity display does not work in rerun viewer once an entity has been established a pinhole.
  Log pinhole as child entity of actual pose as workaround for now.
   */
  rec->log(cam_name + "/pinhole", *rrpinhole);
  rec->log(cam_name + "/pinhole", rerun::Transform3D().with_relation(rerun::components::TransformRelation::ParentFromChild));
  rec->log(cam_name + "/point_label", rerun::Transform3D().with_relation(rerun::components::TransformRelation::ParentFromChild));
  rec->log(cam_name + "/point_label", rerun::Points3D({{0.0f, 0.0f, 0.0f}}).with_labels(rerun::components::Text("cam" + std::to_string(img.ImageId()))));
}

void rrfuse::LogCamPoints3D(const std::shared_ptr<rerun::RecordingStream>& rec,
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

void rrfuse::LogPoint3D(const std::shared_ptr<rerun::RecordingStream>& rec, const colmap::point3D_t& pt3d_id, const Eigen::Vector3d& xyz) {
  std::string pt3d_name = "world/point3d/" + std::to_string(pt3d_id);
  rerun::Position3D pos(xyz.x(), xyz.y(), xyz.z());

  rec->log(pt3d_name, rerun::archetypes::Points3D(pos));
}

void rrfuse::LogRelPoseFactor(const std::shared_ptr<rerun::RecordingStream>& rec,
                              const colmap::Rigid3d& T_ij_odom,
                              const colmap::Image& img_i,
                              const colmap::Image& img_j) {
  // obtain relative pose of node j with respect to node i from current states in images (non measurements)
  const colmap::Rigid3d T_ij = img_i.CamFromWorld() * colmap::Inverse(img_j.CamFromWorld());  // T_i_from_j = T_iw * T_wj

  // rerun naming stuff
  std::string source_cam_name = "world/cam" + std::to_string(img_i.ImageId());
  std::string pred_cam_name = "world/cam" + std::to_string(img_i.ImageId()) + "/cam" + std::to_string(img_j.ImageId()) + "_predicted";
  std::string edge_origin_to_pred_pose = source_cam_name + std::to_string(img_j.ImageId()) + "_to_pred_pose";
  std::string edges_i_pred_j = source_cam_name + "/edges_" + std::to_string(img_j.ImageId());

  // downcast to float to match rerun adapter type. Invert to obtain pose of cam with respect to world.
  const rerun::Vec3D t_ij = fuhe::rr_utils::ToRerunPose3D(T_ij).first;
  std::pair<rerun::Vec3D, rerun::Mat3x3> T_ij_pred = fuhe::rr_utils::ToRerunPose3D(T_ij_odom, false);

  std::vector<rerun::Vec3D> line_segments;   // vector containg xyz points of line semgents
  line_segments.push_back(rerun::Vec3D());   // origin in zero (parent entity is i cam pose)
  line_segments.push_back(T_ij_pred.first);  // pass predicted pose as control point
  line_segments.push_back(t_ij);             // end line strip in pose of image j
  rerun::LineStrip3D line_strip(line_segments);

  // log predicted camera pose to rerun.
  rec->log(pred_cam_name,
           rerun::Transform3D::from_translation_mat3x3(T_ij_pred.first, T_ij_pred.second)
               .with_relation(rerun::components::TransformRelation::ParentFromChild)
               .with_axis_length(rerun::components::AxisLength(0.4f)));
  // draw ellipsoid around precited pose
  rec->log(pred_cam_name + "/ellipse",
           rerun::Ellipsoids3D::from_centers_and_half_sizes({{0.0f, 0.0f, 0.0f}}, {{0.03f, 0.01f, 0.04f}})
               .with_colors({rerun::Rgba32(255, 255, 0)})
               .with_labels(rerun::components::Text("cam" + std::to_string(img_j.ImageId()) + "_predicted")));

  // log linestrips connecting the factors
  rec->log(edges_i_pred_j, rerun::LineStrips3D(line_strip));
}
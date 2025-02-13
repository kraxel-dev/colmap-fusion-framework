#include "high_level_fusion/rerun_interface.h"

#include "fusion_helper/rr_collection_adapters.h"
#include "fusion_helper/rr_utils.h"
#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>

void rrfuse::LogCamPose(const std::shared_ptr<rerun::RecordingStream>& rec,
                        const std::shared_ptr<rerun::Pinhole> rrpinhole,
                        const colmap::Image& img,
                        const colmap::image_t& id) {
  std::string cam_name = "world/cam" + std::to_string(id);

  // match rerun adapter type.
  std::pair<rerun::Vec3D, rerun::Mat3x3> T = fuhe::rr_utils::ToRerunPose3D(img.CamFromWorld(), true);

  // log camera pose to rerun. data of t and R may go out of scope, but as rerun object they shall live on
  rec->log(cam_name, rerun::Transform3D(T.first, T.second));

  // establish camera for logged pose under same name as pose
  rec->log(cam_name, *rrpinhole);
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
                              const std::shared_ptr<rerun::Pinhole>& rrpinhole,
                              const colmap::Rigid3d& T_ij,
                              const colmap::Image& img_i,
                              const colmap::Image& img_j) {
  // increment pose of source node i with relative odometry to obtain predicted pose of node j with respect to world
  colmap::Rigid3d predicted_w_from_j = colmap::Inverse(img_i.CamFromWorld()) * T_ij;  // T_world_from_j = inverse(T_i_from_w) * T_i_from_j

  // rerun naming stuff
  std::string source_cam_name = "world/cam" + std::to_string(img_i.ImageId());
  std::string pred_cam_name = "world/cam" + std::to_string(img_i.ImageId()) + "_predicted";
  std::string edge_origin_to_pred_pose = source_cam_name + std::to_string(img_j.ImageId()) + "_to_pred_pose";
  std::string edge_pred_pose_to_dest = source_cam_name + "_pred_pose_to_" + std::to_string(img_j.ImageId());

  // downcast to float to match rerun adapter type. Invert to obtain pose of cam with respect to world.
  const rerun::Vec3D t_i = fuhe::rr_utils::ToRerunPose3D(img_i.CamFromWorld(), true).first;
  const rerun::Vec3D t_j = fuhe::rr_utils::ToRerunPose3D(img_j.CamFromWorld(), true).first;
  std::pair<rerun::Vec3D, rerun::Mat3x3> T_ij_pred = fuhe::rr_utils::ToRerunPose3D(predicted_w_from_j, false);

  std::vector<rerun::Vec3D> line_segments;  // vector containg xyz points of line semgents
  line_segments.push_back(t_i);
  line_segments.push_back(T_ij_pred.first);
  line_segments.push_back(t_j);
  rerun::LineStrip3D line_strip(line_segments);

  // log predicted camera pose to rerun.
  rec->log(pred_cam_name, rerun::Transform3D(T_ij_pred.first, T_ij_pred.second).with_scale(rerun::components::Scale3D(3)));
  // establish camera for logged pose under same name as pose
  // rec->log(pred_cam_name, *rrpinhole); // TODO: think about better visual object for predicted pose

  // log linestrips connecting the factors
  rec->log(edge_pred_pose_to_dest, rerun::LineStrips3D(line_strip));
}

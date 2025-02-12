#include "high_level_fusion/rerun_interface.h"

#include "fusion_helper/rr_collection_adapters.h"
#include "fusion_helper/rr_utils.h"
#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>

void rrfuse::LogCamPose(std::shared_ptr<rerun::RecordingStream>& rec,
                        std::shared_ptr<rerun::Pinhole>& rrpinhole,
                        const colmap::Image& img,
                        const colmap::image_t& id) {
  std::string cam_name = "world/cam" + std::to_string(id);

  // downcast to float to match rerun adapter type. Invert to obtain pose of cam with respect to world.
  Eigen::Vector3f t = colmap::Inverse(img.CamFromWorld()).translation.cast<float>();
  Eigen::Matrix<float, 3, 3> R = colmap::Inverse(img.CamFromWorld()).rotation.toRotationMatrix().cast<float>();

  // log camera pose to rerun. data of t and R may go out of scope, but as rerun object they shall live on
  rec->log(cam_name, rerun::Transform3D(rerun::Vec3D(t.data()), rerun::Mat3x3(R.data())));

  // establish camera for logged pose under same name as pose
  rec->log(cam_name, *rrpinhole);
}

void rrfuse::LogRelPoseFactor(std::shared_ptr<rerun::RecordingStream>& rec,
                              std::shared_ptr<rerun::Pinhole>& rrpinhole,
                              const colmap::Rigid3d& T_ij,
                              const colmap::Image& img_i,
                              const colmap::image_t& id_i,
                              const colmap::Image& img_j,
                              const colmap::image_t& id_j) {
  // increment pose of source node i with relative odometry to obtain predicted pose of node j with respect to world
  colmap::Rigid3d predicted_w_from_j = colmap::Inverse(img_i.CamFromWorld()) * T_ij;  // T_world_from_j = inverse(T_i_from_w) * T_i_from_j

  // rerun naming stuff
  std::string source_cam_name = "world/cam" + std::to_string(id_i);
  std::string pred_cam_name = "world/cam" + std::to_string(id_i) + "_predicted";
  std::string edge_origin_to_pred_pose = source_cam_name + std::to_string(id_i) + "_to_pred_pose";
  std::string edge_pred_pose_to_dest = source_cam_name + "_pred_pose_to_" + std::to_string(id_j);

  // downcast to float to match rerun adapter type. Invert to obtain pose of cam with respect to world.
  Eigen::Vector3f t_i = colmap::Inverse(img_i.CamFromWorld()).translation.cast<float>();
  Eigen::Vector3f t_j = colmap::Inverse(img_j.CamFromWorld()).translation.cast<float>();
  Eigen::Vector3f t_pred = predicted_w_from_j.translation.cast<float>();
  Eigen::Matrix<float, 3, 3> R_pred = predicted_w_from_j.rotation.toRotationMatrix().cast<float>();

  std::vector<rerun::Vec3D> line_segments;  // vector containg xyz points of line semgents
  line_segments.emplace_back(t_i.data());
  line_segments.emplace_back(t_pred.data());
  line_segments.emplace_back(t_j.data());
  rerun::LineStrip3D line_strip(line_segments);

  // log predicted camera pose to rerun.
  rec->log(pred_cam_name, rerun::Transform3D(rerun::Vec3D(t_pred.data()), rerun::Mat3x3(R_pred.data())));
  // establish camera for logged pose under same name as pose
  // rec->log(pred_cam_name, *rrpinhole);

  // log linestrips connecting the factors
  rec->log(edge_pred_pose_to_dest, rerun::LineStrips3D(line_strip));
}

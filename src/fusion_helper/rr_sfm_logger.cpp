#include "fusion_helper/rr_sfm_logger.h"

#include "fusion_helper/col_utils.h"
#include "fusion_helper/rr_utils.h"

namespace fuhe {
namespace rr {

////////////////////////////////////////////////////////////////////////////////
// Rerun Sfm Logger
////////////////////////////////////////////////////////////////////////////////

RerunSfmLogger::RerunSfmLogger(const RerunVisualizationOptions& rr_opts, std::shared_ptr<colmap::Reconstruction> reconstruction)
    : rr_options_{rr_opts}, reconstruction_{reconstruction} {
  // -------------------- Set rerun context
  VLOG(2) << "Initializing rerun viewer for fusion graph!";
  rr_rec_ = std::make_shared<rerun::RecordingStream>("bundle", "shared");
  this->GetRerunRec()->spawn().exit_on_failure();

  this->GetRerunRec()->log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);  // Set an z-up-axis

  // -------------------- Save rerun recording to disk if specified
  if (this->rr_options_.is_save_rerun_to_disk) {
    VLOG(2) << "Saving rerun recording to rr file!";
    this->GetRerunRec()->save(std::string_view(this->rr_options_.recording_path + "/recording.rrd")).exit_on_failure();
    VLOG(2) << "Path: " << std::string_view(this->rr_options_.recording_path + "/recording.rrd");
  }

  // -------------------- Set shape of rerun camera view objects for visualizer
  // NOTE: currently we assume that the same pinhole model was used for ALL images
  colmap::Camera cam = this->Reconstruction()->Camera(this->Reconstruction()->Images().begin()->second.CameraId());

  const float focal_length_x = cam.FocalLengthX(), focal_length_y = cam.FocalLengthY();
  const float width = cam.width, height = cam.height;
  VLOG(2) << "Focal length of first camera in model: " << focal_length_x << " and " << focal_length_y;
  VLOG(2) << "Resolution of first camera in model [pxl]: " << width << " and " << height;

  // create rerun pinhole object needed to visualize camera poses in rerun
  rr_pinhole_ = std::make_shared<rerun::Pinhole>(
      rerun::Pinhole::from_focal_length_and_resolution({focal_length_x, focal_length_y}, {width, height})
          .with_image_plane_distance(rr_options_.img_plane_dist));
}

std::shared_ptr<rerun::RecordingStream> RerunSfmLogger::GetRerunRec() const { return this->rr_rec_; }
std::shared_ptr<rerun::Pinhole> RerunSfmLogger::GetRerunPinhole() const { return this->rr_pinhole_; }

/// increase time sequence of rerun logger by one
void RerunSfmLogger::UpdateRerunTimeStep() {
  this->time_step_ += 1;
  rr_rec_->set_time_sequence("step", this->time_step_);
}

void RerunSfmLogger::LogFullReconstruction() {
  UpdateRerunTimeStep();

  // -------------------- Registered images
  // log all registered images
  for (auto& [_, img] : col_utils::RegisteredImages(this->Reconstruction())) {
    this->LogCamPose(img);
  }

  // -------------------- Tracks
  this->LogPoints3D(this->Reconstruction()->Points3D());
}

void RerunSfmLogger::LogActivBundle(const colmap::BundleAdjustmentConfig* ba_config) {
  UpdateRerunTimeStep();

  // -------------------- Filter images and points3D based on active BA problem
  std::unordered_map<colmap::image_t, colmap::Image> active_imgs;       // active images in current BA problem
  std::unordered_map<colmap::point3D_t, colmap::Point3D> active_pts3D;  // active 3d points in current BA problem
  // filter points and imgs for this BA problem by populated BA config. log only those
  col_utils::ImagesAndPointsInActiveBA(
      *ba_config, this->Reconstruction()->Images(), this->Reconstruction()->Points3D(), active_imgs, active_pts3D);

  // -------------------- Registered images
  // log all registered images
  for (auto& [_, img] : active_imgs) {
    this->LogCamPose(img, /*highlight=*/true);
  }

  // -------------------- Tracks
  this->LogPoints3D(active_pts3D, /*is_subset=*/true);
}

void RerunSfmLogger::ClearActiveBundle() {
  // -------------------- Clear subsetted rerun 3d points
  this->GetRerunRec()->log(rr_utils::GetPoints3DName(/*is_subset=*/true), rerun::Points3D::clear_fields());

  // -------------------- Clear Pinhole Bounding Boxes
  // clear pinholes highlighted with bounding boxes in local bundle
  for (auto const id : this->Reconstruction()->RegImageIds()) {
    std::string cam_name = rr_utils::GetCamPosesName(id);
    this->GetRerunRec()->log(cam_name + "/bb", rerun::Boxes3D::clear_fields());
  }
}

void RerunSfmLogger::LogCamPose(const colmap::Image& img, const bool highlight) {
  // -------------------- Log camera pose
  std::string cam_name = rr_utils::GetCamPosesName(img.ImageId());

  // match rerun adapter type.
  std::pair<rerun::Vec3D, rerun::Mat3x3> T = fuhe::rr_utils::ToRerunPose3D(img.CamFromWorld(), /*inv=*/true);

  // log camera pose to rerun. data of t and R may go out of scope, but as rerun object they shall live on
  this->GetRerunRec()->log(cam_name, rerun::Transform3D(T.first, T.second));

  /*
  NOTE: For som weird reaseon, child entity display does not work in rerun viewer once an entity has been established a pinhole.
  Log pinhole as child entity of actual pose as workaround for now.
   */
  // establish camera for logged pose under same name as pose
  this->GetRerunRec()->log(cam_name + "/pinhole", rerun::Pinhole(*this->GetRerunPinhole().get()));

  // -------------------- Text label for camera pose
  // add a point to slap a label to the image pose
  if (rr_options_.is_show_cam_labels) {
    this->GetRerunRec()->log(cam_name + "/tf_label",
                             rerun::Transform3D().with_relation(rerun::components::TransformRelation::ParentFromChild));
    this->GetRerunRec()->log(
        cam_name + "/tf_label",
        rerun::Points3D({{0.05f, -0.1f, 0.0f}}).with_labels(rerun::components::Text("cam" + std::to_string(img.ImageId()))));
  }

  // -------------------- Bounding box around camera pose
  // log bounding box around camera pose (handy to see active cams in local BA)
  if (highlight) {
    const float ax_length = rr_options_.img_plane_dist / 2.6f;
    this->GetRerunRec()->log(cam_name + "/bb", rerun::Boxes3D::from_half_sizes({{ax_length, ax_length, ax_length}}));
  }
}

void RerunSfmLogger::UpdateModelBBox() {
  VLOG(2) << "Updating to reonstruction bounding box given its current 3d points. If toggled in Options, points outside of new "
             "BBox will be omitted in rerun viewer!";

  // compute new model bbox based on its 3d Points. Note the lower and upper bound of omitted percentile.
  const fuhe::types::ColmapBBox bbox =
      this->Reconstruction()->ComputeBoundingBox(rr_options_.model_bbox_lb, rr_options_.model_bbox_ub);
  VLOG(3) << "Model Bbox size is: " << bbox.first << " and " << bbox.second;

  if (!model_bbox_) {
    model_bbox_ = std::make_shared<fuhe::types::ColmapBBox>(bbox);
  } else {
    *model_bbox_.get() = bbox;
  }
}

void RerunSfmLogger::LogCamPose(const colmap::image_t& id, const bool highlight) {
  this->LogCamPose(this->Reconstruction()->Image(id), highlight);
}

std::shared_ptr<colmap::Reconstruction> RerunSfmLogger::Reconstruction() const { return reconstruction_; }

RerunVisualizationOptions RerunSfmLogger::RerunOptions() const { return rr_options_; }

void RerunSfmLogger::LogPoints3D(const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D, const bool is_subset) {
  // clear old rerun 3d points
  this->GetRerunRec()->log(rr_utils::GetPoints3DName(is_subset), rerun::Points3D::clear_fields());

  std::vector<rerun::Position3D> points;
  int n_skipped = 0;

  // log all 3d points
  for (auto& pt3D : points3D) {
    // Should actually be `track.observations.size() < options_.min_num_view_per_track`.
    if (pt3D.second.track.Length() < 2) continue;
    const Eigen::Vector3f xyz = pt3D.second.xyz.cast<float>();

    // ignore points that are outside the bounding box of colmap model
    if (rr_options_.is_ignore_pts_beyond_model_bbox) {
      if (model_bbox_ && !col_utils::IsPointInBBox(pt3D, *model_bbox_.get())) {
        n_skipped++;
        continue;
      }
    }

    points.emplace_back(xyz.x(), xyz.y(), xyz.z());
    // FIXME: bring colors to rerun
    // colors.emplace_back(track.color[0], track.color[1], track.color[2]);
  }

  this->GetRerunRec()->log(rr_utils::GetPoints3DName(is_subset), rerun::Points3D(points));

  if (n_skipped > 0) {
    VLOG(3) << "Skipped total of " << n_skipped << " points for rerun streaming cause they're beyond model BB!";
  }
}

void RerunSfmLogger::LogInfoMsg(const std::string& msg) {
  this->GetRerunRec()->log("logs", rerun::TextLog(msg).with_level(rerun::TextLogLevel::Info));
}

////////////////////////////////////////////////////////////////////////////////
// Fusion Graph Logger
////////////////////////////////////////////////////////////////////////////////

void RerunFusionGraphLogger::LogFullReconstruction() { GetSfmLogger()->LogFullReconstruction(); }

void RerunFusionGraphLogger::LogActivBundle(const colmap::BundleAdjustmentConfig* ba_config) {
  GetSfmLogger()->LogActivBundle(ba_config);
}

void RerunFusionGraphLogger::LogOdometryEdges() {
  const bool log_odom_as_predicted_pose = true;  // do not log odometry measurement as absolute pose
  // -------------------- Edges
  const auto& reg_images = col_utils::RegisteredImages(GetSfmLogger()->Reconstruction());

  // iterate over all graph nodes and
  for (auto& [_, img_edge] : active_fusion_graph_edges_) {
    // skip image if not entailed by COLMAP model (e.g. not yet registered during reconstr). Note that an upstream instance
    // should have already filtered the odom edges through a valid ba_config to only inlcude edges of currently active BA. This
    // following check is just a last-resort safety measure.
    if (reg_images.find(img_edge.CurrId()) == reg_images.end()) {
      continue;
    }

    // skip image without odometry constraint
    if (!img_edge.OdomEdge()) {
      continue;
    }

    auto odom_edge = img_edge.OdomEdge();

    // skip relative poses whose source node is not entailed in selected reg_images
    if (reg_images.find(odom_edge->PrevId()) == reg_images.end()) {
      continue;
    }

    // obtain relative pose from odometry edge
    const colmap::Rigid3d T_ij_rigid =
        colmap::Rigid3d(Eigen::Quaterniond(odom_edge->T_i_from_j().rotation()), odom_edge->T_i_from_j().translation());
    VLOG(5) << "Rerun logging relpose factor with rigid: " << T_ij_rigid;

    // -------------------- Log Edge to Rerun
    this->LogOdometryEdge(
        T_ij_rigid, reg_images.at(odom_edge->PrevId()), reg_images.at(odom_edge->CurrId()), /*is_odom_as_pred_pose=*/true);
  }
}

void RerunFusionGraphLogger::LogOdometryEdgesAsTrajectory() {
  // absolute odometry pose, incremented per interation with each consecutive relpose
  colmap::Rigid3d T_world_from_odom = colmap::Rigid3d();

  std::vector<rerun::Vec3D> line_segments;  // vector containg xyz points of line semgents
  bool is_init = true;                      // for fetching origin absolute pose

  // -------------------- Edges
  // iterate over all active fusion graph edges
  for (auto& [_, edge] : active_fusion_graph_edges_) {
    // -------------------- Safety checks
    // skip if no viable odometry edge availabe for this image
    if (!edge.OdomEdge()) {
      continue;
    }

    auto odom_edge = edge.OdomEdge();

    // -------------------- Init condition
    if (is_init) {
      // grabbing first colmap pose (i) of valid odom edge as origin of tum trajectory
      T_world_from_odom = colmap::Inverse(this->Reconstruction()->Image(edge.PrevId()).CamFromWorld());

      const rerun::Vec3D t_i = fuhe::rr_utils::ToRerunPose3D(T_world_from_odom, false).first;  // pos of j img in world
      // line strip connecting odometry poses
      line_segments.push_back(t_i);  // node i as origin

      is_init = false;
    }

    // -------------------- Increment curr edge onto existing trajectory
    const colmap::Rigid3d T_ij_rigid =
        colmap::Rigid3d(Eigen::Quaterniond(odom_edge->T_i_from_j().rotation()), odom_edge->T_i_from_j().translation());

    // increment rel pose to obtain absolute pose for current node
    T_world_from_odom = T_world_from_odom * T_ij_rigid;

    // stream absolute odom pose, connected to associated colmap cam poses to rerun
    this->LogOdometryEdge(T_world_from_odom,
                          this->Reconstruction()->Image(odom_edge->PrevId()),
                          this->Reconstruction()->Image(odom_edge->CurrId()),
                          /*is_odom_as_pred_pose=*/false);

    // extend line strip connecting odometry poses
    const rerun::Vec3D t_j = fuhe::rr_utils::ToRerunPose3D(T_world_from_odom, false).first;  // pos of j img in world
    line_segments.push_back(t_j);                                                            // node i as origin
  }

  rerun::LineStrip3D line_strip(line_segments);
  this->GetRerunRec()->log("world/odometry", rerun::LineStrips3D(line_strip).with_labels(rerun::components::Text("Odometry")));
}

void RerunFusionGraphLogger::ClearAllOdometryEdges() {
  // just brute force clear the whole entity path
  this->GetRerunRec()->log(rr_utils::GetSourceFrameNameOdomEdges(/*is_relative_pose=*/true), rerun::archetypes::Clear(true));
  this->GetRerunRec()->log(rr_utils::GetSourceFrameNameOdomEdges(/*is_relative_pose=*/false), rerun::archetypes::Clear(true));
}

void RerunFusionGraphLogger::LogOdometryEdge(const colmap::Rigid3d& T_ij_odom,
                                             const colmap::Image& img_i,
                                             const colmap::Image& img_j,
                                             const bool is_odom_as_pred_pose) {
  // rerun naming stuff
  std::string pred_cam_name =
      fuhe::rr_utils::GetEntityNamesOdomEdge(img_i.ImageId(), img_j.ImageId(), is_odom_as_pred_pose).first;
  std::string edge_i_pred_j_name =
      fuhe::rr_utils::GetEntityNamesOdomEdge(img_i.ImageId(), img_j.ImageId(), is_odom_as_pred_pose).second;

  // absolute pose odom (that is associated with node j) w.r.t world
  colmap::Rigid3d T_odom_w_j;

  // if odom pose is provided as relative pose between i and j (typical case)
  if (is_odom_as_pred_pose) {
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
  this->GetRerunRec()->log(pred_cam_name, rerun::Transform3D::from_translation_mat3x3(T_ij_pred.first, T_ij_pred.second));
  // draw coord frame axis onto predicted pose for visiblity
  this->GetRerunRec()->log(pred_cam_name + "/axis-frame", this->FrameAxis());

  // add a point to slap a label to the predicted image pose
  if (GetSfmLogger()->RerunOptions().is_show_edge_labels) {
    this->GetRerunRec()->log(
        pred_cam_name + "/axis-frame/pt",
        rerun::Points3D({{0.05f, 0.2f, -0.2f}})
            .with_labels(rerun::components::Text(fuhe::rr_utils::GetLabelNameEdge(img_i.ImageId(), img_j.ImageId()))));
  }

  // log linestrips connecting the factors
  this->GetRerunRec()->log(edge_i_pred_j_name, rerun::LineStrips3D(line_strip));
  // FIXME: kick edge label here if predicted cam label performs better
  // GetRerunRec()->log(edge_i_pred_j_name,
  //          rerun::LineStrips3D(line_strip)
  //              .with_labels(rerun::Text(fuhe::rr_utils::GetLabelNameEdge(img_i.ImageId(), img_j.ImageId()))));
}

rerun::Arrows3D RerunFusionGraphLogger::FrameAxis() {
  const float axis_len_odom = GetSfmLogger()->RerunOptions().img_plane_dist * 0.63f;  // TODO: magic number fix in future
  return rerun::Arrows3D::from_vectors({{axis_len_odom, 0.0, 0.0}, {0.0, axis_len_odom, 0.0}, {0.0, 0.0, axis_len_odom}})
      .with_colors({{255, 0, 0}, {0, 255, 0}, {0, 0, 255}});
}
}  // namespace rr
}  // namespace fuhe

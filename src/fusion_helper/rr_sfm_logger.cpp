#include "fusion_helper/rr_sfm_logger.h"

#include "fusion_helper/col_utils.h"

namespace fuhe {
namespace rr {

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
    this->LogCamPose(img);
  }

  // -------------------- Tracks
  this->LogPoints3D(active_pts3D);
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
  // compute new model bbox based on its 3d Points. Note the lower and upper bound of omitted percentile.
  const fuhe::types::ColmapBBox bbox =
      this->Reconstruction()->ComputeBoundingBox(rr_options_.model_bbox_lb, rr_options_.model_bbox_ub);

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

void RerunSfmLogger::LogPoints3D(const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D, const bool is_subset) {
  // clear old rerun 3d points
  this->GetRerunRec()->log(rr_utils::GetPoints3DName(is_subset), rerun::Points3D::clear_fields());

  std::vector<rerun::Position3D> points;
  // log all 3d points
  for (auto& pt3D : points3D) {
    // Should actually be `track.observations.size() < options_.min_num_view_per_track`.
    if (pt3D.second.track.Length() < 2) continue;
    const Eigen::Vector3f xyz = pt3D.second.xyz.cast<float>();

    // ignore points that are outside the bounding box of colmap model
    if (rr_options_.is_ignore_pts_beyond_model_bbox) {
      if (model_bbox_ && col_utils::IsPointInBBox(pt3D, *model_bbox_.get())) {
        continue;
      }
    }

    points.emplace_back(xyz.x(), xyz.y(), xyz.z());
    // FIXME: bring colors to rerun
    // colors.emplace_back(track.color[0], track.color[1], track.color[2]);
  }

  this->GetRerunRec()->log(rr_utils::GetPoints3DName(is_subset), rerun::Points3D(points));
}

void RerunSfmLogger::LogInfoMsg(const std::string& msg) {
  this->GetRerunRec()->log("logs", rerun::TextLog(msg).with_level(rerun::TextLogLevel::Info));
}

}  // namespace rr
}  // namespace fuhe

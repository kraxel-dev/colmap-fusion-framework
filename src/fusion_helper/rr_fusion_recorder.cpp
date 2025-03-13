#include "fusion_helper/rr_fusion_recorder.h"

namespace fuhe {
namespace rrfuse {

RerunFusionRecorder::RerunFusionRecorder(const RerunFusionVisOptions& rr_opts, const colmap::Reconstruction& reconstruction) : options(rr_opts) {
  // -------------------- Set rerun context
  VLOG(2) << "Initializing rerun viewer for fusion graph!";
  this->rr_rec = std::make_shared<rerun::RecordingStream>("bundle", "shared");
  this->rr_rec->spawn().exit_on_failure();

  this->rr_rec->log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);  // Set an up-axis

  // -------------------- Save rerun recording to disk if specified
  if (this->options.is_save_rerun_to_disk) {
    VLOG(2) << "Saving rerun recording to rr file!";
    this->rr_rec->save(std::string_view(this->options.recording_path + "/recording.rrd")).exit_on_failure();
    VLOG(2) << "Path: " << std::string_view(this->options.recording_path + "/recording.rrd");
  }

  // -------------------- Set shape of rerun camera view objects for visualizer
  // TODO: make generic for all registered cameras (currently we assume that the same pinhole model was used)
  // obtain camera params from first image in colmap model
  colmap::Camera cam = reconstruction.Camera(reconstruction.Images().begin()->second.CameraId());

  const float focal_length_x = cam.FocalLengthX(), focal_length_y = cam.FocalLengthY();
  const float width = cam.width, height = cam.height;
  VLOG(2) << "Focal length of first camera in model: " << focal_length_x << " and " << focal_length_y;
  VLOG(2) << "Resolution of first camera in model [pxl]: " << width << " and " << height;

  // create rerun pinhole object needed to visualize camera poses in rerun
  this->rr_pinhole = std::make_shared<rerun::Pinhole>(
      rerun::Pinhole::from_focal_length_and_resolution({focal_length_x, focal_length_y}, {width, height}).with_image_plane_distance(0.1));
}

}  // namespace rrfuse
}  // namespace fuhe

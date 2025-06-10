/**
 * @file rr_fusion_logger.h
 * @author kraxel
 * @brief helper class for rerun viewer initialization and other stuff
 * @version 0.1
 * @date 2025-03-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "fusion_helper/rr_utils.h"
#include <colmap/geometry/rigid3.h>
#include <colmap/scene/reconstruction.h>
#include <rerun.hpp>

namespace fuhe {
namespace rrfuse {

/// rerun visualization options for colmap models, bundle adjustment and fusion graph optimization process
struct RerunVisualizationOptions {
  // enable logging and visualization to rerun
  bool is_log_to_rerun = true;
  // enable saving rerun logged data to disk as rrd file. Note that real-time logging deactivates with this.
  bool is_save_rerun_to_disk = false;
  std::string recording_path = "";
  float img_plane_dist = 0.2f;  // controls size of cam pinhole in rerun viewer //FIXME: CURRENTLY NOT USED please kick

  // whether to draw external odometry as predicted poses with respect to source camera or as absolute poses
  bool draw_rerun_odom_as_predicted_poses = true;

  // whether to highlight active images of BA in rerun with bounding boxes. Deactivate for use cases of BA in which the full
  // model is part of the optimization and the view can get cluttered as a result (high-lvl fusion for example).
  bool is_highlight_active_cams = true;
};

// FIXME: rename to RerunLoggingManager or RerunRecordingManager
/// TODO: write brief
class RerunFusionRecorder {
 public:
  RerunFusionRecorder(const RerunVisualizationOptions& rr_opts, const colmap::Reconstruction& reconstruction);
  ~RerunFusionRecorder() = default;

  std::shared_ptr<rerun::RecordingStream> GetRerunRec() const;
  std::shared_ptr<rerun::Pinhole> GetRerunPinhole() const;

  /// increment time sequence for rerun recording stream. can be called in every ceres iteration or before registering a new
  /// image
  void UpdateRerunTimeStep();

  inline int& TimeStep() { return time_step; }
  inline void SetTimeStep(int time_step) { time_step = time_step; }

 private:
  const RerunVisualizationOptions options;
  std::shared_ptr<rerun::RecordingStream> rr_rec = nullptr;  // rerun logger / streamer and viewer object
  std::shared_ptr<rerun::Pinhole> rr_pinhole = nullptr;      // rerun pinhole model representing the camera used in colmap model
  int time_step = 0;
};

}  // namespace rrfuse

}  // namespace fuhe
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

/// rerun visualization options for fusion graph optimization process
struct RerunFusionVisOptions {
  bool is_log_to_rerun = true;         // enable logging and visualization of graph construction and optimization to rerun
  bool is_save_rerun_to_disk = false;  // enable saving rerun logged data to disk as rrd file
  std::string recording_path = "";

  // whether to draw external odometry as predicted poses with respect to source camera or as absolute poses
  bool draw_rerun_odom_as_predicted_poses = true;
};

// FIXME: file name rr to rerun
/// TODO: write brief
class RerunFusionRecorder {
 public:
  RerunFusionRecorder(const RerunFusionVisOptions& rr_opts, const colmap::Reconstruction& reconstruction);
  ~RerunFusionRecorder() = default;

  inline const std::shared_ptr<rerun::RecordingStream> GetRerunRec() const { return this->rr_rec; }
  inline const std::shared_ptr<rerun::Pinhole> GetRerunPinhole() const { return this->rr_pinhole; }

  /// increment time sequence for rerun recording stream. can be called in every ceres iteration or before registering a new image
  void UpdateRerunTimeStep();

  inline int TimeStep() const { return time_step; }
  inline void SetTimeStep(int time_step) { time_step = time_step; }

 private:
  const RerunFusionVisOptions options;
  std::shared_ptr<rerun::RecordingStream> rr_rec = nullptr;  // rerun logger and viewer object
  std::shared_ptr<rerun::Pinhole> rr_pinhole = nullptr;      // rerun pinhole model representing the camera used in colmap model
  int time_step = 0;
};

}  // namespace rrfuse

}  // namespace fuhe
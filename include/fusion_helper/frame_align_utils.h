/**
 * @file frame_align_utils.h
 * @author kraxel
 * @brief collection of helper tools to perform colmap model frame and coordinate transformations, given use-cases like
 * trajectory evaluation. Handy during active reconstruction or as post-processing step to bring models into a non-bogus global
 * frame (e.g. align vehicle motion to the x-y plane).
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <colmap/estimators/coordinate_frame.h>
#include <colmap/scene/reconstruction.h>

namespace fuhe {
namespace align {

struct AlignmentOptions {
  // perform all alignment duties listed below on colmap model once when specified amount of images were registered. Set to 0 if
  // alignment is not desired (any alignment will be ignored)
  uint n_reg_for_alignment = 12;

  // align colmap reconstruction through PCA to the axes along the motion. This works well to find xy plane for vehicle data
  // (that didnt endlessly drive up a parking garage ramp) but will probably less useful on drone data.
  bool pca_align = true;

  // transform colmap model such that the pose of the first image assumes the pose specified by xyz and euler angle values below.
  // Important to let the camera trajectory start at the pose of your camera with respect to your base vehicle. E.g. if your
  // camera is mounted 1 m in front and 0.5 m above your vehicle baselink, the cam trajectory should start at that position
  // instead of coordinate origin. In other words, your first image pose should be aligned to the extrinsic calibration of your
  // mounted cam with respect to your vechicle. NOTE: if given bad extrinsics, the direction of the resulting colmap trajectory
  // will be off or tilted.
  bool align_first_cam_to_specific_pose = false;
  double specified_x = 1.58;         // w.r.t global world frame
  double specified_y = 0.06;         // w.r.t global world frame
  double specified_z = 1.566;        // w.r.t global world frame
  double specified_roll = 0.0155;    // w.r.t global world frame (zyx intrinsic euler angle (rad) convention)
  double specified_pitch = -0.1705;  // w.r.t global world frame (zyx intrinsic euler angle (rad) convention)
  double specified_yaw = 0.0;        // w.r.t global world frame (zyx intrinsic euler angle (rad) convention)
  // apply final rotation of camera center onto the optical lense frame (from x forward to z forward y down)
  bool auto_rot_into_optical = true;

  // orient the colmap trajectory such that the initial motion (direction of 1st to 2nd colmap pose) aligns with the x-axis of
  // the global coordinate frame. Important to better compare with ground truth data from other sensor links that set the start
  // position of the vechicle as coordinate origin. NOTE: this strategy is brittle with backwards or turning motion as trajectory
  // start.
  bool rotate_init_motion_onto_global_x_axis = false;
};

/// simple check on whether to run coordinage frame alignment strategies on colmap model. Returns true when specified nr of
/// images were registered
bool CheckRunAlignment(const int n_registered_imgs, const AlignmentOptions& align_opts);

/// orient the colmap trajectory such that init motion (pose of 2nd img w.r.t to 1st) aligns with global x axis. NOTE: this
/// strategy is brittle with backwards or turning motion as trajectory start.
void AlignInitMotionWithGlobalXAxis(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                                    const colmap::image_t id1,
                                    const colmap::image_t id2);

/// align first colmap pose to some initial pose (e.g. cam extrinsics) specified by alignment options given a translation and
/// euler angles. Call after pca alignment to make things easier.
void AlignFirstPoseToSpecified(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                               const colmap::image_t& id1,
                               const AlignmentOptions& align_opts);

/// perform all coordinate frame alignment strategies on a colmap model specified by alignment options. Use CheckRunAlignment()
/// before calling this function.
void PerformAlignmentStrategies(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                                const AlignmentOptions& align_opts);

}  // namespace align
}  // namespace fuhe

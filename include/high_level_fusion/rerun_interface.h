/**
 * @file rerun_interface.h
 * @author kraxel
 * @brief TODO: write brief
 * @version 0.1
 * @date 2025-02-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "fusion_helper/odom_edges_manager.h"
#include "fusion_helper/types.h"
#include <colmap/scene/reconstruction.h>
#include <rerun.hpp>

namespace rrfuse {  // rerun interface namespace

// some constant params for shared entity size
inline constexpr float AXIS_LENGTH_PINHOLE = 0.15f;
inline constexpr float AXIS_LENGTH_ODOM = 0.6 * AXIS_LENGTH_PINHOLE;

/**
 * @brief // TODO: write brief
 *
 * @param rec Rerun logger and streamer object
 * @param rrpinhole Correctly configured rerun pinhole object corresponding to camera model used in colmap reconstruction (focal and
 * resolution must be set)
 * @param img
 * @param id
 */
void LogCamPose(const std::shared_ptr<rerun::RecordingStream> rec,
                const std::shared_ptr<rerun::Pinhole> rrpinhole,
                const colmap::Image& img);

/// log only 3D points for a single image to rerun
void LogCamPoints3D(const std::shared_ptr<rerun::RecordingStream> rec, const colmap::Image& img, const std::vector<colmap::Point3D>& pts3D);

/// clear manually and incrementally registered 3D points that were logged per image
void ClearAllCamPoints3D(const std::shared_ptr<rerun::RecordingStream> rec,
                         const std::unordered_map<colmap::camera_t, colmap::Image>& images);

/// log a single 3D point to rerun
void LogPoint3D(const std::shared_ptr<rerun::RecordingStream> rec, const colmap::point3D_t& pt3d_id, const Eigen::Vector3d& xyz);

void LogOdometryEdge(const std::shared_ptr<rerun::RecordingStream> rec,
                     const colmap::Rigid3d& T_ij_odom,
                     const colmap::Image& img_i,
                     const colmap::Image& img_j,
                     const bool is_odom_a_relpose = true);

/// Log relataive pose from external odom sensor as predicted pose seen from node i. Connect lindestrips between nodes i j and predicted
/// pose to highlight them as factor graph edge
void LogOdometryEdgeAsPredictedPose(const std::shared_ptr<rerun::RecordingStream> rec,
                                    const colmap::Rigid3d& T_ij_odom,
                                    const colmap::Image& img_i,
                                    const colmap::Image& img_j);

/**
 * @brief Log odometry edge constraining to colmap nodes i j but with the absolute pose of odometry reading with respect to colmap coords
system. User must provide the correct absolute pose.
 *
 * @param rec
 * @param T_w_from_odom Absolute pose of odometry with respect to some world frame.
 * @param img_i
 * @param img_j
 */
void LogOdometryEdgeWithAbsolutePose(const std::shared_ptr<rerun::RecordingStream> rec,
                                     const colmap::Rigid3d& T_w_from_odom,
                                     const colmap::Image& img_i,
                                     const colmap::Image& img_j);

/**
 * @brief Log whole colmap reconstruction to rerun. Can be used in ceres iteration callback.
 * @ref
 * https://github.com/colmap/glomap/commit/5115de482dc0a72b5c6d01d39da3524b7a296608#diff-2139378b2a5608827fa60ae83cfc220b18a3c68e7972248aeb715da8b60594fc
 */
void LogReconstruction(const std::shared_ptr<rerun::RecordingStream> rec,
                       const std::shared_ptr<rerun::Pinhole> rrpinhole,
                       const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                       const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D);

/// Log whole colmap reconstruction to rerun with image sorted by their time stamps. Can be used in ceres iteration callback.
void LogReconstructionSorted(const std::shared_ptr<rerun::RecordingStream> rec,
                             const std::shared_ptr<rerun::Pinhole> rrpinhole,
                             const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                             const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                             const fuhe::types::MapOfImageIdsSec& ids_by_stamp);

/// Log all relative poses of external odometry as predicted poses as seen from node i for all nodes i j
void LogOdometryEdges(const std::shared_ptr<rerun::RecordingStream> rec,
                      const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                      const std::map<const double, fuhe::OdomImagesEdge> edges);

/// Draw all external odometry measurements as absolute poses and visually constrain the absolute colmap image poses with them as edge.
void LogOdometryEdgesAsTrajectory(const std::shared_ptr<rerun::RecordingStream> rec,
                                  const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                                  const std::map<const double, fuhe::OdomImagesEdge> edges,
                                  const bool log_traj_as_linestrip = false);
}  // namespace rrfuse
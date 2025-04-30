/**
 * @file rerun_fusion_logging.h
 * @author kraxel
 * @brief TODO: Convencience functions to stream classic colmap data (imgs, 3d points or whole reconstruction) and fusion graph data to
 * rerun viewer. Most exciting when used during active ceres optimization step in conjunction with the FusionIterationCallback class.
 * @version 0.1
 * @date 2025-02-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "fusion_helper/odom_edges_manager.h"
#include "fusion_helper/types.h"
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/scene/reconstruction.h>
#include <rerun.hpp>
namespace fuhe {
namespace rrfuse {  // rerun interface namespace

/// Log a text log message to rerun viewer
void LogInfo(const std::shared_ptr<rerun::RecordingStream> rec, const std::string& msg);

/**
 * @brief // TODO: write brief
 *
 * @param rec Rerun logger and streamer object
 * @param rrpinhole Correctly configured rerun pinhole object corresponding to camera model used in colmap reconstruction (focal and
 * resolution must be set)
 * @param img
 * @param highlight draw bouning box around pinhole to highlight this image
 */
void LogCamPose(const std::shared_ptr<rerun::RecordingStream> rec,
                const std::shared_ptr<rerun::Pinhole> rrpinhole,
                const colmap::Image& img,
                const bool highlight = false);

/// log only 3D points for a single image to rerun
void LogCamPoints3D(const std::shared_ptr<rerun::RecordingStream> rec, const colmap::Image& img, const std::vector<colmap::Point3D>& pts3D);

/// clear manually and incrementally registered 3D points that were logged per image
void ClearAllCamPoints3D(const std::shared_ptr<rerun::RecordingStream> rec,
                         const std::unordered_map<colmap::camera_t, colmap::Image>& images);

/// log a single 3D point to rerun
void LogPoint3D(const std::shared_ptr<rerun::RecordingStream> rec, const colmap::point3D_t& pt3d_id, const Eigen::Vector3d& xyz);

/**
 * @brief log whole set of 3D points to rerun. Switches rerun entity name of points3D, if marked as subset. This is only relevant in local
 * and global BA and controlled by upsteram logic.
 *
 * @param rec
 * @param points3D
 * @param is_subset
 * @param ignore_far_away_points Do not visualize far away 3d points that mess up the 3D viewer.
 */
void LogPoints3D(const std::shared_ptr<rerun::RecordingStream> rec,
                 const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                 const bool is_subset = false,
                 const bool ignore_far_away_points = false);

/// Log odometry edge constraining to colmap nodes i j as linestrip and the pose of the odometry measruement to highlight them as factor
/// graph edge. Pose can be drawn as relative increment (seen from source image i) or as absolute pose with respect to some coord frame.
/// When choosing to draw odometry reading as absolute pose, user must provide the correct absolute pose himself. Linestrip drawing of edges
/// works automatically for both cases.
void LogOdometryEdge(const std::shared_ptr<rerun::RecordingStream> rec,
                     const colmap::Rigid3d& T_ij_odom,
                     const colmap::Image& img_i,
                     const colmap::Image& img_j,
                     const bool is_odom_a_relpose = true);

void LogTotalFactorCost(const std::shared_ptr<rerun::RecordingStream> rec, const std::string& factor_type, const double total_cost);

/**
 * @brief Log full colmap reconstruction to rerun. Can be used in ceres iteration callback. Should be straightforward to use on any colmap
 * model. Can be used to log a subset of a full model (e.g. only images and points3D that are part of the current BA problem). In that case
 * an upstream helper function should help with subsetting the data.
 * @ref
 * https://github.com/colmap/glomap/commit/5115de482dc0a72b5c6d01d39da3524b7a296608#diff-2139378b2a5608827fa60ae83cfc220b18a3c68e7972248aeb715da8b60594fc
 *
 * @param rec
 * @param rrpinhole
 * @param images
 * @param points3D
 * @param is_subset
 */
void LogReconstruction(const std::shared_ptr<rerun::RecordingStream> rec,
                       const std::shared_ptr<rerun::Pinhole> rrpinhole,
                       const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                       const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                       const bool is_subset = false);

/// Given full range of imgs and 3d points, log only active points and images in an ongoing Bundle Adjustment (local and global) to rerun.
/// Active subset of imgs and points are obtained through a populated ba_config (make sure its populated correctly which is normally done by
/// colmap itself). Almost as LogReconstruction but with different colors for this subset. Can be used in ceres iteration callback.
void LogActivBundle(const std::shared_ptr<rerun::RecordingStream> rec,
                    const std::shared_ptr<rerun::Pinhole> rrpinhole,
                    const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                    const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                    const colmap::BundleAdjustmentConfig* ba_config,
                    const bool highlight_cams = true);

/// Use in destructors of ceres marathon iteration callbacks to clear all temporary visualizations of an active bundle adjustment
/// process in rerun.
void ClearActiveBundle(const std::shared_ptr<rerun::RecordingStream> rec, const std::unordered_set<colmap::camera_t>& cam_ids);

/// Log all relative poses of external odometry as predicted poses as seen from node i for all nodes i j
void LogOdometryEdges(const std::shared_ptr<rerun::RecordingStream> rec,
                      const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                      const edges::MapOfImageEdges graph_data_edges);

/// Draw all external odometry measurements as absolute poses and visually constrain the absolute colmap image poses with them as edge.
void LogOdometryEdgesAsTrajectory(const std::shared_ptr<rerun::RecordingStream> rec,
                                  const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                                  const edges::MapOfImageEdges graph_data_edges,
                                  const bool log_traj_as_linestrip = false);

void ClearAllOdometryEdges(const std::shared_ptr<rerun::RecordingStream> rec);

}  // namespace rrfuse
}  // namespace fuhe
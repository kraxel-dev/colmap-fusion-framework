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

#include "fusion_helper/types.h"
#include <colmap/scene/reconstruction.h>
#include <rerun.hpp>

namespace rrfuse {  // rerun interface namespace

/**
 * @brief // TODO: write brief
 *
 * @param rec Rerun logger and streamer object
 * @param rrpinhole Correctly configured rerun pinhole object corresponding to camera model used in colmap reconstruction (focal and
 * resolution must be set)
 * @param img
 * @param id
 */
void LogCamPose(const std::shared_ptr<rerun::RecordingStream>& rec,
                const std::shared_ptr<rerun::Pinhole> rrpinhole,
                const colmap::Image& img);

/// log only 3D points for a single image to rerun
void LogCamPoints3D(const std::shared_ptr<rerun::RecordingStream>& rec,
                    const colmap::Image& img,
                    const std::vector<colmap::Point3D>& pts3D);

/// log a single 3D point to rerun
void LogPoint3D(const std::shared_ptr<rerun::RecordingStream>& rec, const colmap::point3D_t& pt3d_id, const Eigen::Vector3d& xyz);

void LogRelPoseFactor(const std::shared_ptr<rerun::RecordingStream>& rec,
                      const colmap::Rigid3d& T_ij_odom,
                      const colmap::Image& img_i,
                      const colmap::Image& img_j);

/**
 * @brief Log whole colmap reconstruction to rerun. Can be used in ceres iteration callback.
 * @ref
 * https://github.com/colmap/glomap/commit/5115de482dc0a72b5c6d01d39da3524b7a296608#diff-2139378b2a5608827fa60ae83cfc220b18a3c68e7972248aeb715da8b60594fc
 */
void LogReconstruction(const std::shared_ptr<rerun::RecordingStream>& rec,
                       const std::shared_ptr<rerun::Pinhole>& rrpinhole,
                       const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                       const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D);

}  // namespace rrfuse
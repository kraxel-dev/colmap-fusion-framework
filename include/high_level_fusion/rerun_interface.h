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
                const colmap::Image& img,
                const colmap::image_t& id);

void LogCamPoints3D(const std::shared_ptr<rerun::RecordingStream>& rec,
                    const colmap::Image& img,
                    const std::vector<colmap::Point3D>& pts3D);

void LogPoint3D(const std::shared_ptr<rerun::RecordingStream>& rec, const colmap::point3D_t& pt3d_id, const Eigen::Vector3d& xyz);

void LogRelPoseFactor(const std::shared_ptr<rerun::RecordingStream>& rec,
                      const colmap::Rigid3d& T_ij_odom,
                      const colmap::Image& img_i,
                      const colmap::Image& img_j);

}  // namespace rrfuse
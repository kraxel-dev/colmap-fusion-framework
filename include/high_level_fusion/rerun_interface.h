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
void LogCamPose(std::shared_ptr<rerun::RecordingStream>& rec,
                std::shared_ptr<rerun::Pinhole>& rrpinhole,
                const colmap::Image& img,
                const colmap::image_t& id);

void LogRelPoseFactor(std::shared_ptr<rerun::RecordingStream>& rec,
                      std::shared_ptr<rerun::Pinhole>& rrpinhole,
                      const colmap::Rigid3d& T_ij,
                      const colmap::Image& img_i,
                      const colmap::image_t& id_i,
                      const colmap::Image& img_j,
                      const colmap::image_t& id_j);

}  // namespace rrfuse
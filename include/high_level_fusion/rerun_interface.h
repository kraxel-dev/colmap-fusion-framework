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

namespace rrfuse {  // rerun interface namespace

void LogCamPose(const colmap::Image& img);
void LogRelPoseFactor(const colmap::Rigid3d& T_ij, const colmap::Image& img_i);

}  // namespace rrfuse
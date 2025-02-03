/**
 * @file io.h
 * @author kraxel
 * @brief Helper functions for reading and writing tum trajectory data or other sensor / pose related files (imu, gps?)
 * @version 0.1
 * @date 2025-02-01
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <string>

#include "fusion_helper/tum_benchmark/tum_benchmark.hpp"
#include "fusion_helper/types.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <colmap/geometry/rigid3.h>
#include <stdlib.h>

typedef tum_benchmark::FileReader<tum_benchmark::Trajectory> TumPoseReader;

namespace fuhe {  // fusion helper
namespace io {

// keep 2 digits after decimal points for ten millisecond precision
constexpr int DIGIT_PRECISION = 2;

/**
 * @brief cut off decimal digits of double value after specified precision. Use this to round off large timestamps for comparison of
 incredibly large timestamps that were parsed with precision loss
 *
 * @param stamp
 * @param precision digit after decimal point to round to
 * @return double
 */
double TruncateDouble(const double stamp, int precision = DIGIT_PRECISION);

/**
 * @brief Parse TUM benchmark file and extract timestamps of poses
 *
 * @param tum_file path to TUM benchmark file
 * @param timestamps vector to store timestamps
 */
void TumToStamps(const std::string& tum_file, std::vector<double>& timestamps);

/**
 * @brief Parse tum trajectory file and extract poses as Eigen::Isometry3d
 *
 * @param tum_file path to TUM benchmark file
 * @param out_poses vector to store poses
 */
void TumToPosesEigen(const std::string& tum_file, std::vector<Eigen::Isometry3d>& out_file);

/**
 * @brief Parse poses from tum trajectory file and store in a map, accessible and sorted by their timestamps in seconds
 *
 * @param tum_file tum trajectory file path
 * @param out_poses_map map to store poses
 * @param cut_precision cut off precision of timestamps to x amount of digits specified by DIGIT_PRECISION
 */
void TumToPosesEigen(const std::string& tum_file, types::MapOfPosesSec& out_poses_map, const bool cut_precision = true);

void Rigid3dToTum(std::vector<colmap::Rigid3d>& X, const std::string& tum_file, const bool inv = false);
}  // namespace io
}  // namespace fuhe
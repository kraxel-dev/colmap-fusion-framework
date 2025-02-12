/**
 * @file stream_utils.h
 * @author kraxel
 * @brief TODO: fill brief
 * @version 0.1
 * @date 2025-02-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <iomanip>

#include <Eigen/Geometry>
#include <glog/logging.h>

/*
NOTE:
Keeping stream operator overloads restrictive to Pose Matrices and Covariances of certain dimension for now.
Might template them in future to make them more generic. But keep in mind that overriding stream
operators in global namespace and in header files in general is not recommended.
 */

/**
 * @brief stream operator overload to format Eigen Isometry3d matrices in a more readable way when using with glog.
 *
 * @param os std::ostream&
 * @param mat to be printed Eigen matrix
 * @return std::ostream&
 */
inline std::ostream& operator<<(std::ostream& os, const Eigen::Transform<double, 3, 1>& mat) {
  os << "\n" << std::setfill(' ') << mat.matrix();  // get rid of these pesky zeros in front of matrix elements when logging
  return os;
}

/**
 * @brief stream operator overload to format ixj Covariances matrices in a more readable way when using with glog.
 *
 * @param os std::ostream&
 * @param cov to be printed covariance matrix
 * @return std::ostream&
 */
template <int Rows, int Cols>
inline std::ostream& operator<<(std::ostream& os, const Eigen::Matrix<double, Rows, Cols>& cov) {
  os << "\n" << std::setfill(' ') << cov.matrix();  // get rid of these pesky zeros in front of matrix elements when printing logging
  return os;
}
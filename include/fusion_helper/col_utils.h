// TODO: docstring
#pragma once

#include <map>
#include <set>

#include <colmap/scene/reconstruction.h>

namespace fuhe {
namespace col_utils {

/**
 * @brief Return image ids of colmap model sorted and accessible by their timestamps [seconds] to have them in ascending order.
 This is requred to match images of colmap model to metric poses in tumfile.
 *
 * @param image_ids
 * @param reconstruction colmap model
 * @return std::map<double, colmap::image_t>
 */
std::map<const double, colmap::image_t> ImageIdsByStamp(const std::set<colmap::image_t>& image_ids,
                                                        std::shared_ptr<colmap::Reconstruction> reconstruction);

/// obtain all 3d points associated to given image
const std::vector<colmap::Point3D> GetPoints3D(const colmap::image_t& image_id,
                                               const std::shared_ptr<colmap::Reconstruction>& reconstruction);

/// crop out far away 3d points from colmap model by computing bounding box of all points omitting the last percentile
void CropFarAwayPoints(const std::shared_ptr<colmap::Reconstruction>& reconstruction);

/**
    * @brief From forwarded colmap image, get pointers to image pose (cam_from_world: world pose expressed in cam) objects required by ceres
    for adding image to optimization problem in the factor graph. Takes care of quaternion normalization for
    convenience.

    * @param img Reference to colmap image whose pose we want retrieve as paramter for optimization.
    * @param q_c_from_w pointer to first quaternion value (double) in memory
    * @param t_c_from_w pointer to first translation value (double) in memory
    */
void GetPointersToPose(colmap::Image& img, double*& q_c_from_w, double*& t_c_from_w);
}  // namespace col_utils
}  // namespace fuhe
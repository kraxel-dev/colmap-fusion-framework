// TODO: docstring
#pragma once

#include <set>
#include <map>

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

}  // namespace col_utils
}  // namespace fuhe
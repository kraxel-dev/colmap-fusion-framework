/**
 * @file col_utils.h
 * @author kraxel
 * @brief utilitiy functions to help with colmap model handling
 * @version 0.1
 * @date 2025-03-26
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "fusion_helper/types.h"
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/scene/reconstruction.h>

namespace fuhe {
namespace col_utils {

/**
 * @brief Return image ids of colmap model sorted and accessible by their timestamps [seconds] to have them in ascending order.
 * Required to match images of colmap model to metric poses in tumfile.
 *
 * @param model_images map of colmap images in a reconstruction, accessible by their ids
 * @return fuhe::types::MapOfImageIdsSec
 */
fuhe::types::MapOfImageIdsSec ImageIdsByStamp(const std::unordered_map<colmap::image_t, colmap::Image>& images_by_id);

/**
 * @brief Return a map of registerd images in colmap model, accessed by their ids. Convencience function to not only have the ids
 * as set from native model getter. Be careful when returning directly into functions that accept (non const) references, since
 * they'll reference a map that directly goes out of scope.
 *
 * @param reconstruction
 * @return const std::unordered_map<colmap::image_t, colmap::Image>
 */
std::unordered_map<colmap::image_t, colmap::Image> RegisteredImages(const colmap::Reconstruction* reconstruction);
/// convenience overload for shared ptr reconstructions
std::unordered_map<colmap::image_t, colmap::Image> RegisteredImages(
    const std::shared_ptr<colmap::Reconstruction> reconstruction);

/**
 * @brief Given a full set of images and their ids, return a subset of images chosen by target ids.
 *
 * @param target_ids
 * @param images
 * @return std::unordered_map<colmap::image_t, colmap::Image>
 */
const std::unordered_map<colmap::image_t, colmap::Image> SubsetOfImages(
    const std::unordered_set<colmap::image_t>& target_ids, const std::unordered_map<colmap::image_t, colmap::Image>& images);

/**
 * @brief Given a full set of 3D points and their ids, return a subset of images chosen by target ids.
 *
 * @param target_ids
 * @param points3D
 * @return const std::unordered_map<colmap::point3D_t, colmap::Point3D>
 */
const std::unordered_map<colmap::point3D_t, colmap::Point3D> SubsetOfPoints3D(
    const std::unordered_set<colmap::point3D_t>& target_ids,
    const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D);

/// obtain all 3d points associated to given image. filter out points with not enough track length
const std::vector<colmap::Point3D> GetPoints3DForImage(const colmap::image_t& image_id,
                                                       const int min_track_len,
                                                       const std::shared_ptr<colmap::Reconstruction> reconstruction);
/**
 * @brief Given all images and 3d points contained in a colmap model, obtain a subset of images and points that are active in a
 * ceres Bundle Adjustment problem problem.
 *
 * @param ba_config
 * @param images all images of reconstruction
 * @param points3D all 3d pts of reconstruction
 * @param active_images (out) filtered imgs
 * @param active_points3D (out) filtered pts
 * @return true
 * @return false
 */
bool ImagesAndPointsInActiveBA(const colmap::BundleAdjustmentConfig& ba_config,
                               const std::unordered_map<colmap::image_t, colmap::Image>& images,
                               const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                               std::unordered_map<colmap::image_t, colmap::Image>& active_images,
                               std::unordered_map<colmap::point3D_t, colmap::Point3D>& active_points3D);

/// crop out bogus 3d points from colmap model by computing bounding box of all points omitting the last percentile. Very handy
/// for easening rerun visualization.
void CropBBoxOutlierPoints(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                           const float min_percentile = 0.01,
                           const float max_percentile = 0.95);

/**
 * @brief Check if a 3D point is inside a bounding box defined by the 3D points of colmap model. Check orignial colmap lib for
 * details.
 *
 * @param point3D pair of id and colmap Point3D object
 * @param bbox bbox object obtained through native colmap ComputeBoundingBox function.
 * @return true
 * @return false
 */
const bool IsPointInBBox(const std::pair<colmap::point3D_t, colmap::Point3D>& point3D,
                         const fuhe::types::ColmapBBox& bbox);

/**
* @brief From forwarded colmap image, get pointers to image pose (cam_from_world: world pose expressed in cam) objects
required by ceres for adding image to optimization problem in the factor graph. Takes care of quaternion normalization for
convenience.

* @param img Reference to colmap image whose pose we want retrieve as paramter for optimization.
* @param q_c_from_w pointer to first quaternion value (double) in memory
* @param t_c_from_w pointer to first translation value (double) in memory
*/
void GetPointersToPose(colmap::Image& img, double*& q_c_from_w, double*& t_c_from_w);

/**
 * @brief Print statistics of two view geometry that matter to intialization criterias of reconstruction initial pairs.
 *
 * @param tvg two view geometry that should be printed
 */
void PrintTwoViewStatistics(const colmap::TwoViewGeometry& tvg);

/// given a finalized colmap model, export the image poses w.r.t to world as tum trajectory
void ToTum(const colmap::Reconstruction* reconstruction, const std::string& out_folder);

}  // namespace col_utils
}  // namespace fuhe
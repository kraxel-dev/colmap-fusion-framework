#pragma once

#include "fusion_helper/rr_utils.h"
#include "high_level_fusion/rerun_interface.h"  // TODO: move rerun_interface to fusion helper
#include <ceres/ceres.h>
#include <ceres/iteration_callback.h>
#include <colmap/scene/image.h>
#include <colmap/scene/point3d.h>
#include <rerun.hpp>

namespace fuhe {

/**
 * @brief Ceres Callback, called during every iteration of ceres optimization. Derived to log colmap reconstruction to rerun.
 * @ref 1. https://github.com/rerun-io/glomap/blob/main/glomap/estimators/global_positioning.cc#L26
 *      2. http://ceres-solver.org/nnls_solving.html#_CPPv4N5ceres17IterationCallbackE
 *
 */
class FusionIterationCallback : public ceres::IterationCallback {
 public:
  FusionIterationCallback(const std::shared_ptr<rerun::RecordingStream> rr_rec,
                          const std::shared_ptr<rerun::Pinhole> rrpinhole,
                          const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                          const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D)
      : rr_rec{rr_rec}, rrpinhole{rrpinhole}, images{images}, points3D{points3D} {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    // -------------------- iteration summary logging
    // Instead of logging iteration manually here, make use of default ceres per iteration printer by setting:
    //  solver_options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    // in the main executable

    // -------------------- Log full reconstruction
    this->UpdateIterationRerun(summary, this->images);
    return ceres::SOLVER_CONTINUE;
  }

 protected:
  const std::shared_ptr<rerun::RecordingStream> rr_rec;
  const std::shared_ptr<rerun::Pinhole> rrpinhole;
  const std::unordered_map<colmap::camera_t, colmap::Image>& images;
  const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D;

  void UpdateIterationRerun(const ceres::IterationSummary& summary,
                            const std::unordered_map<colmap::camera_t, colmap::Image>& target_images) {
    // -------------------- Update Rerun data
    rr_rec->set_time_sequence("step", summary.iteration);
    rrfuse::LogReconstruction(rr_rec, rrpinhole, target_images, points3D);
  }
};

/**
 * @brief Ceres Callback, called during every iteration of ceres optimization. Derived to log colmap reconstruction to rerun.
 Logs colmap images in sorted order.
 * @ref 1. https://github.com/rerun-io/glomap/blob/main/glomap/estimators/global_positioning.cc#L26
 *      2. http://ceres-solver.org/nnls_solving.html#_CPPv4N5ceres17IterationCallbackE
 *
 */
class FusionIterationCallbackSorted : public FusionIterationCallback {
 public:
  FusionIterationCallbackSorted(const std::shared_ptr<rerun::RecordingStream> rr_rec,
                                const std::shared_ptr<rerun::Pinhole> rrpinhole,
                                const std::unordered_map<colmap::camera_t, colmap::Image>& images,
                                const std::unordered_map<colmap::point3D_t, colmap::Point3D>& points3D,
                                const fuhe::types::MapOfImageIdsSec& img_ids_by_stamp)
      : FusionIterationCallback(rr_rec, rrpinhole, images, points3D), img_ids_by_stamp{img_ids_by_stamp} {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    std::unordered_map<colmap::camera_t, colmap::Image> imgs_sorted_by_stamp;
    // iterate over all stamp sorted ids to have 'images' in sequential order
    // for (auto& [_, id] : this->img_ids_by_stamp) {
    //   imgs_sorted_by_stamp[id] = this->images.at(id);
    // }

    // this->UpdateIterationRerun(summary, imgs_sorted_by_stamp);
    rr_rec->set_time_sequence("step", summary.iteration);
    rrfuse::LogReconstructionSorted(rr_rec, rrpinhole, images, points3D, img_ids_by_stamp);
    return ceres::SOLVER_CONTINUE;
  }

 protected:
  fuhe::types::MapOfImageIdsSec img_ids_by_stamp;
};

}  // namespace fuhe

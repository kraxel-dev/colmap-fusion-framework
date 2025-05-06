/**
 * @file incremental_mapper.h
 * @author kraxel
 * @brief Drived versions of colmaps default IncrementalMapper to:
 -  introduce rerun visualization of colmaps standard incremental mapping process.
 -  introduce custom sensor fusion bundle adjustment into colmaps standard incremental mapping process.
 The incremental mapper class acts as library with clean APIs to perform a colmap mapping process from scratch. You can either use it in
 your own script to perform incremental mapping from scratch or have a look at the incremental_pipeline class in original colmap lib, that
 acts as an wrapper to perform the mapping duties you see in the default colmap gui.
 * @source: https://github.com/colmap/colmap/blob/3.11.1/src/colmap/sfm/incremental_mapper.h
 * @version 0.1
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "tightly_coupled_fusion/estimators/bundle_adjustment.h"
#include <colmap/sfm/incremental_mapper.h>

namespace tcf {

/**
 * @brief Colmap's native IncrementalMapper (default behavior) with additional rerun logging during local and global BA.
 *
 */
class IncrementalMapperRerun : public colmap::IncrementalMapper {
 public:
  using colmap::IncrementalMapper::IncrementalMapper;

  /**
   * @brief Derived to swap out DefaultBA with rerun DefaultBA (besides that, exact implementation as parent). Adjust locally connected
   * images and points of a reference image. In addition, refine the provided 3D points. Only images connected to the reference image are
   * optimized. If the provided 3D points are not locally connected to the reference image, their observing images are set as constant in
   * the adjustment.
   *
   * @param options
   * @param ba_options
   * @param tri_options
   * @param image_id
   * @param point3D_ids
   * @return LocalBundleAdjustmentReport
   */
  colmap::IncrementalMapper::LocalBundleAdjustmentReport AdjustLocalBundle(const Options& options,
                                                                           const colmap::BundleAdjustmentOptions& ba_options,
                                                                           const colmap::IncrementalTriangulator::Options& tri_options,
                                                                           colmap::image_t image_id,
                                                                           const std::unordered_set<colmap::point3D_t>& point3D_ids);

  /**
   * @brief Derived to call custom local BA with rerun logging inside of vanilla iterative scheme. Perform multiple rounds of local bundle
   * adjustment.
   *
   * @param max_num_refinements
   * @param max_refinement_change
   * @param options
   * @param ba_options
   * @param tri_options
   * @param image_id
   */
  void IterativeLocalRefinement(int max_num_refinements,
                                double max_refinement_change,
                                const Options& options,
                                const colmap::BundleAdjustmentOptions& ba_options,
                                const colmap::IncrementalTriangulator::Options& tri_options,
                                colmap::image_t image_id);

  /// same scheme as local refinement
  bool AdjustGlobalBundle(const Options& options, const colmap::BundleAdjustmentOptions& ba_options);
  void IterativeGlobalRefinement(int max_num_refinements,
                                 double max_refinement_change,
                                 const Options& options,
                                 const colmap::BundleAdjustmentOptions& ba_options,
                                 const colmap::IncrementalTriangulator::Options& tri_options,
                                 bool normalize_reconstruction = true);

  /// Attach RerunRecorder object to mapper. Required if rerun logging is desired.
  inline void AttachRerunRecorder(const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder>& rr_recorder) { rr_recorder_ = rr_recorder; }
  inline std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> RerunRecorder() const { return rr_recorder_; }

 protected:
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder_ = nullptr;
};

////////////////////////////////////////////////////////////////////////////////
// Incremental Fusion Mapper
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Derived incremental mapper to extend colmaps default mapping behavior with sensor fusion (e.g. relative odometry constraints)
capabilities. The fusion is integrated into the local and global Bundle Adjustemnt steps of the mapping process.

The incremental mapper class acts as library with clean APIs to perform a colmap mapping process from scratch and the fusion mapper mirrors
this. Order of images processed for mapping is decided externally (either manually or by incremental_pipeline class). You can either use
this class in your own script to perform incremental mapping from scratch or have a look at the incremental_pipeline class in original
colmap lib, that acts as an wrapper to perform the mapping duties you see in the default colmap gui.

@soruce: https://github.com/colmap/colmap/blob/3.11.1/src/colmap/sfm/incremental_mapper.h
* See the original docs of the incremental mapper class to get a feel:
// Class that provides all functionality for the incremental reconstruction
// procedure. Example usage:
//
//  IncrementalMapper mapper(&database_cache);
//  mapper.BeginReconstruction(&reconstruction);
//  TwoViewGeometry tvg;
//  THROW_CHECK(
//      mapper.FindInitialImagePair(options, tvg, image_id1, image_id2));
//  mapper.RegisterInitialImagePair(options, tvg, image_id1, image_id2);
//  while (...) {
//    const auto next_image_ids = mapper.FindNextImages(options);
//    for (const auto image_id : next_image_ids) {
//      THROW_CHECK(mapper.RegisterNextImage(options, image_id));
//      if (...) {
//        mapper.AdjustLocalBundle(...);
//      } else {
//        mapper.AdjustGlobalBundle(...);
//      }
//    }
//  }
//  mapper.EndReconstruction(false);
//
 *
 */
class IncrementalFusionMapper : public colmap::IncrementalMapper {
 public:
  /**
   * @brief Construct a new Incremental Fusion Mapper object. Fusion graph edges containing the odometry edges are created from the given
   * database cache during object construction.
   *
   * @param database_cache colmap database cache containing images, features and matches that will be same as parent.
   * @param fusion_options options for fusion graph bundle adjustment.
   * @param tum_file Mandatory path to tum file with absolute pose odometry data utilized for fusion. Required if mapper with odometry
   * fusion capabilities is desired.
   * @param rr_options rerun visualization options.
   */
  IncrementalFusionMapper(std::shared_ptr<const colmap::DatabaseCache> database_cache,
                          FusionGraphBundleAdjustmentOptions& fusion_options,
                          const std::string& tum_file,
                          fuhe::rrfuse::RerunFusionVisOptions& rr_options);

  /// Derived to call multiple rounds of derived local bundle adjustment with fusion capabilities.
  void IterativeLocalRefinement(int max_num_refinements,
                                double max_refinement_change,
                                const Options& options,
                                const colmap::BundleAdjustmentOptions& ba_options,
                                const colmap::IncrementalTriangulator::Options& tri_options,
                                colmap::image_t image_id);

  /// Derived to swap out DefaultBA with custom FusionGraphBA (besides that, exact implementation as parent). Adjust locally connected
  /// images and points of a reference image. In addition, refine the provided 3D points. Only images connected to the reference image are
  /// optimized.
  colmap::IncrementalMapper::LocalBundleAdjustmentReport AdjustLocalBundle(const Options& options,
                                                                           const colmap::BundleAdjustmentOptions& ba_options,
                                                                           const colmap::IncrementalTriangulator::Options& tri_options,
                                                                           colmap::image_t image_id,
                                                                           const std::unordered_set<colmap::point3D_t>& point3D_ids);

  /// Derived to call multiple rounds of derived global bundle adjustment with fusion capabilities.  // FIXME: kick method if parent method
  /// safely calls custom internal call
  void IterativeGlobalRefinement(int max_num_refinements,
                                 double max_refinement_change,
                                 const Options& options,
                                 const colmap::BundleAdjustmentOptions& ba_options,
                                 const colmap::IncrementalTriangulator::Options& tri_options,
                                 bool normalize_reconstruction = false);

  /// Derived Global Bundle Adjustment with fusion capabilities.
  bool AdjustGlobalBundle(const Options& options, const colmap::BundleAdjustmentOptions& ba_options);

  /// Set fusion graph edges (time sorted image node sequence with odometry edges) that will be utilized by ceres optimization. Required if
  /// mapper with odometry fusion capabilities is desired. Provide the unfiltered range of sequential edges that would span the full
  /// colmap model after succ. reconstruction.
  inline void SetFusionGraphEdges(std::shared_ptr<fuhe::edges::MapOfImageEdges> fusion_graph_data_edges) {
    fusion_graph_data_edges_ = fusion_graph_data_edges;
  };
  inline const std::shared_ptr<fuhe::edges::MapOfImageEdges> FusionGraphDataEdges() const { return fusion_graph_data_edges_; }

  /// Attach RerunRecorder object to mapper. Required if rerun logging is desired.
  inline void AttachRerunRecorder(const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder>& rr_recorder) { rr_recorder_ = rr_recorder; }
  inline std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> RerunRecorder() const { return rr_recorder_; }

 protected:
  bool is_fusion_mapping_ = true;                      // if not, switch to default vision only ba in whole mapping process
  FusionGraphBundleAdjustmentOptions fusion_options_;  // options for fusion graph bundle adjustment
  const std::string tum_file_ = "";                    // path to tum file with odometry data

  const fuhe::rrfuse::RerunFusionVisOptions rr_options_;                      // rerun visualization options
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder_ = nullptr;  // custom RerunRecorder object if visualization is desired

  std::shared_ptr<fuhe::edges::MapOfImageEdges> fusion_graph_data_edges_ =
      nullptr;  // time sorted image node sequence with odometry edges constraining images
};

}  // namespace tcf
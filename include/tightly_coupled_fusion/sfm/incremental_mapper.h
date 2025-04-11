/**
 * @file incremental_mapper.h
 * @author kraxel
 * @brief Drived version of colmaps IncrementalMapper to introduce custom fusion graph bundle adjustment into colmaps standard incremental
 * mapping process.
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

  /// Attach RerunRecorder object to mapper. Is required if rerun logging is desired.
  inline void AttachRerunRecorder(const std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder>& rr_recorder) { rr_recorder_ = rr_recorder; }
  inline std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> RerunRecorder() const { return rr_recorder_; }

 protected:
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_recorder_ = nullptr;
};

////////////////////////////////////////////////////////////////////////////////
// Incremental Fusion Mapper
////////////////////////////////////////////////////////////////////////////////

class IncrementalFusionMapper : public colmap::IncrementalMapper {
 public:
  /// parent constructor
  using colmap::IncrementalMapper::IncrementalMapper;

  /// Derived to call multiple rounds of derived global bundle adjustment.  // FIXME: kick method if parent method safely calls custom
  /// internal call
  void IterativeGlobalRefinement(int max_num_refinements,
                                 double max_refinement_change,
                                 const Options& options,
                                 const colmap::BundleAdjustmentOptions& ba_options,
                                 const colmap::IncrementalTriangulator::Options& tri_options,
                                 bool normalize_reconstruction = false);

  /// Derived adjustment with fusion bundle adjsutment
  bool AdjustGlobalBundle(const Options& options, const colmap::BundleAdjustmentOptions& ba_options, const bool is_init_pair = false);

  /// Set fusion graph edges (time sorted image node sequence with odometry edges) that will be utilized by ceres optimization. Required if
  /// mapper with odometry fusion capabilities is desired. Provide the full unfiltered range of sequential edges that would span the full
  /// colmap model after succ. reconstruction.
  inline void SetFusionGraphEdges(std::shared_ptr<fuhe::edges::MapOfImageEdges> fusion_graph_data_edges) {
    fusion_graph_data_edges_ = fusion_graph_data_edges;
  };
  inline const std::shared_ptr<fuhe::edges::MapOfImageEdges> FusionGraphDataEdges() const { return fusion_graph_data_edges_; }

  inline bool isInitPair() const { return is_init_pair; }
  inline void setIsInitPair(bool isInitPair) { is_init_pair = isInitPair; }

 protected:
  bool is_init_pair = true;
  std::shared_ptr<fuhe::edges::MapOfImageEdges> fusion_graph_data_edges_ =
      nullptr;  // time sorted image node sequence with odometry edges constraining images
};

}  // namespace tcf
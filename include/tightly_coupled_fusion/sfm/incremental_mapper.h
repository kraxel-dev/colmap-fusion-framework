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

#include "tightly_coupled_fusion/estimators/fusion_bundle_adjustment.h"
#include <colmap/sfm/incremental_mapper.h>

namespace tcf {

class IncrementalFusionMapper : public colmap::IncrementalMapper {
 public:
  /// parent constructor
  using colmap::IncrementalMapper::IncrementalMapper;

  /// Derived to call multiple rounds of derived global bundle adjustment.  // FIXME: kick method if parent method safely calls custom internal call
  void IterativeGlobalRefinement(int max_num_refinements,
                                 double max_refinement_change,
                                 const Options& options,
                                 const colmap::BundleAdjustmentOptions& ba_options,
                                 const colmap::IncrementalTriangulator::Options& tri_options,
                                 bool normalize_reconstruction = false);

  /// Derived adjustment with fusion bundle adjsutment
  bool AdjustGlobalBundle(const Options& options, const colmap::BundleAdjustmentOptions& ba_options, const bool is_init_pair = false);

  inline bool isInitPair() const { return is_init_pair; }
  inline void setIsInitPair(bool isInitPair) { is_init_pair = isInitPair; }

 protected:
  bool is_init_pair = true;
};

}  // namespace tcf
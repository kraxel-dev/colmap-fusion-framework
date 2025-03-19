/**
 * @file incremental_pipeline.h
 * @author kraxel
 * @brief Derived colmap pipeline class for mapping with odometry fusion bundle adjustment.
 * @version 0.1
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "tightly_coupled_fusion/sfm/incremental_mapper.h"
#include <colmap/controllers/incremental_pipeline.h>

namespace tcf {

class IncrementalFusionPipeline : public colmap::IncrementalPipeline {
 public:
  /// inherit parent constructor
  using colmap::IncrementalPipeline::IncrementalPipeline;

  /// swapped internal parent Reconstruct method with derived one
  void Run();

  /// swapped internal IncrementalMapper with custom IncrementalFusionMapper
  void Reconstruct(const colmap::IncrementalMapper::Options& mapper_options);

  Status ReconstructSubModel(tcf::IncrementalFusionMapper& mapper,
                             const colmap::IncrementalMapper::Options& mapper_options,
                             const std::shared_ptr<colmap::Reconstruction>& reconstruction);

  Status InitializeReconstruction(tcf::IncrementalFusionMapper& mapper,
                                  const colmap::IncrementalMapper::Options& mapper_options,
                                  colmap::Reconstruction& reconstruction);

 protected:
  // amount of sub-models that already failed to select 2 suitable initial images
  int num_failed_submodels_bad_initial_pair = 0;
  // amount of sub-models that had a successful initial pair but failed to register more images than the min amount required
  int num_failed_submodels_discont_reg = 0;
};

}  // namespace tcf

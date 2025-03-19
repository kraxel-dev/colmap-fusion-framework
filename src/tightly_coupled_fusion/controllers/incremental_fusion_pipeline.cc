/**
 * @file incremental_pipeline.cc
 * @author your name (you@domain.com)
 * @brief TODO: write brief
 * @version 0.1
 * @date 2025-03-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "tightly_coupled_fusion/controllers/incremental_fusion_pipeline.h"

#include <colmap/util/file.h>
#include <colmap/util/timer.h>

namespace tcf {

void IterativeGlobalRefinement(const colmap::IncrementalPipelineOptions& options,
                               const colmap::IncrementalMapper::Options& mapper_options,
                               tcf::IncrementalFusionMapper& mapper) {
  LOG(INFO) << "Retriangulation and Global bundle adjustment";
  mapper.IterativeGlobalRefinement(options.ba_global_max_refinements,
                                   options.ba_global_max_refinement_change,
                                   mapper_options,
                                   options.GlobalBundleAdjustment(),
                                   options.Triangulation());
  mapper.FilterImages(mapper_options);
}

void ExtractColors(const std::string& image_path, const colmap::image_t image_id, colmap::Reconstruction& reconstruction) {
  if (!reconstruction.ExtractColorsForImage(image_id, image_path)) {
    LOG(WARNING) << colmap::StringPrintf(
        "Could not read image %s at path %s.", reconstruction.Image(image_id).Name().c_str(), image_path.c_str());
  }
}

void WriteSnapshot(const colmap::Reconstruction& reconstruction, const std::string& snapshot_path) {
  LOG(INFO) << "Creating snapshot";
  // Get the current timestamp in milliseconds.
  const size_t timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  // Write reconstruction to unique path with current timestamp.
  const std::string path = colmap::JoinPaths(snapshot_path, colmap::StringPrintf("%010d", timestamp));
  colmap::CreateDirIfNotExists(path);
  VLOG(1) << "=> Writing to " << path;
  reconstruction.Write(path);
}

}  // namespace tcf

// --------------------

void tcf::IncrementalFusionPipeline::Run() {
  VLOG(1) << "Run thread of IncrementalFusionPipeline deployed!";

  using namespace colmap;

  Timer run_timer;
  run_timer.Start();
  if (!LoadDatabase()) {
    return;
  }

  IncrementalMapper::Options init_mapper_options = this->Options()->Mapper();
  Reconstruct(init_mapper_options);

  const size_t kNumInitRelaxations = 2;
  for (size_t i = 0; i < kNumInitRelaxations; ++i) {
    if (this->ReconstructionManager()->Size() > 0 || CheckIfStopped()) {
      break;
    }

    LOG(INFO) << "=> Relaxing the initialization constraints.";
    init_mapper_options.init_min_num_inliers /= 2;
    this->Reconstruct(init_mapper_options);

    if (this->ReconstructionManager()->Size() > 0 || CheckIfStopped()) {
      break;
    }

    LOG(INFO) << "=> Relaxing the initialization constraints.";
    init_mapper_options.init_min_tri_angle /= 2;
    this->Reconstruct(init_mapper_options);
  }

  run_timer.PrintMinutes();
}

void tcf::IncrementalFusionPipeline::Reconstruct(const colmap::IncrementalMapper::Options& mapper_options) {
  VLOG(2) << "Custom fusion reconstruction triggered!";

  using namespace colmap;
  tcf::IncrementalFusionMapper mapper(this->DatabaseCache());

  // Is there a sub-model before we start the reconstruction? I.e. the user
  // has imported an existing reconstruction.
  const bool initial_reconstruction_given = this->ReconstructionManager()->Size() > 0;
  THROW_CHECK_LE(this->ReconstructionManager()->Size(), 1) << "Can only resume from a "
                                                              "single reconstruction, but "
                                                              "multiple are given.";

  for (int num_trials = 0; num_trials < this->Options()->init_num_trials; ++num_trials) {
    if (CheckIfStopped()) {
      break;
    }
    size_t reconstruction_idx;
    if (!initial_reconstruction_given || num_trials > 0) {
      reconstruction_idx = this->ReconstructionManager()->Add();
    } else {
      reconstruction_idx = 0;
    }
    std::shared_ptr<Reconstruction> reconstruction = this->ReconstructionManager()->Get(reconstruction_idx);

    const Status status = this->ReconstructSubModel(mapper, mapper_options, reconstruction);
    switch (status) {
      case Status::INTERRUPTED: {
        mapper.EndReconstruction(/*discard=*/false);
        return;
      }

      case Status::NO_INITIAL_PAIR:
      case Status::BAD_INITIAL_PAIR: {
        this->num_failed_submodels_bad_initial_pair++;
        VLOG(2) << "Number of failed sub-models due to unsuitable intial pairs: " << this->num_failed_submodels_bad_initial_pair;
        mapper.EndReconstruction(/*discard=*/true);
        this->ReconstructionManager()->Delete(reconstruction_idx);
        // If both initial images are manually specified, there is no need for
        // further initialization trials.
        if (this->Options()->IsInitialPairProvided()) {
          return;
        }
        break;
      }

      case Status::SUCCESS: {
        // Remember the total number of registered images before potentially
        // discarding it below due to small size, so we can out of the main
        // loop, if all images were registered.
        const size_t total_num_reg_images = mapper.NumTotalRegImages();

        // If the total number of images is small then do not enforce the
        // minimum model size so that we can reconstruct small image
        // collections. Always keep the first reconstruction, independent of
        // size.
        const size_t min_model_size = std::min<size_t>(0.8 * this->DatabaseCache()->NumImages(), this->Options()->min_model_size);
        if ((this->Options()->multiple_models && this->ReconstructionManager()->Size() > 1 &&
             reconstruction->NumRegImages() < min_model_size) ||
            reconstruction->NumRegImages() == 0) {
          this->num_failed_submodels_discont_reg++;  // increase count of sub models failed due to not enough registrations
          VLOG(2) << "Number of failed sub-models due unsufficiant image registrations after good intial pair: "
                  << this->num_failed_submodels_discont_reg;
          mapper.EndReconstruction(/*discard=*/true);
          this->ReconstructionManager()->Delete(reconstruction_idx);
        } else {
          mapper.EndReconstruction(/*discard=*/false);
        }

        Callback(LAST_IMAGE_REG_CALLBACK);

        if (initial_reconstruction_given || !this->Options()->multiple_models ||
            this->ReconstructionManager()->Size() >= static_cast<size_t>(this->Options()->max_num_models) ||
            total_num_reg_images >= this->DatabaseCache()->NumImages() - 1) {
          return;
        }

        break;
      }

      default:
        LOG(FATAL_THROW) << "Unknown reconstruction status.";
    }
  }
}

colmap::IncrementalPipeline::Status tcf::IncrementalFusionPipeline::ReconstructSubModel(
    tcf::IncrementalFusionMapper& mapper,
    const colmap::IncrementalMapper::Options& mapper_options,
    const std::shared_ptr<colmap::Reconstruction>& reconstruction) {
  // bring namespace into function scope
  using namespace colmap;
  VLOG(3) << "Custom submodule reconstruction triggered!";

  mapper.BeginReconstruction(reconstruction);

  ////////////////////////////////////////////////////////////////////////////
  // Register initial pair
  ////////////////////////////////////////////////////////////////////////////

  if (reconstruction->NumRegImages() == 0) {
    const Status init_status = this->InitializeReconstruction(mapper, mapper_options, *reconstruction);  // KRAXEL EDIT
    if (init_status != Status::SUCCESS) {
      return init_status;
    }
  }
  Callback(INITIAL_IMAGE_PAIR_REG_CALLBACK);

  ////////////////////////////////////////////////////////////////////////////
  // Incremental mapping
  ////////////////////////////////////////////////////////////////////////////

  size_t snapshot_prev_num_reg_images = reconstruction->NumRegImages();
  size_t ba_prev_num_reg_images = reconstruction->NumRegImages();
  size_t ba_prev_num_points = reconstruction->NumPoints3D();

  bool reg_next_success = true;
  bool prev_reg_next_success = true;

  do {
    if (CheckIfStopped()) {
      break;
    }

    prev_reg_next_success = reg_next_success;
    reg_next_success = false;

    const std::vector<image_t> next_images = mapper.FindNextImages(mapper_options);

    if (next_images.empty()) {
      break;
    }

    image_t next_image_id;
    for (size_t reg_trial = 0; reg_trial < next_images.size(); ++reg_trial) {
      next_image_id = next_images[reg_trial];

      LOG(INFO) << StringPrintf("Registering image #%d (%d)", next_image_id, reconstruction->NumRegImages() + 1);
      LOG(INFO) << StringPrintf("=> Image sees %d / %d points",
                                mapper.ObservationManager().NumVisiblePoints3D(next_image_id),
                                mapper.ObservationManager().NumObservations(next_image_id));

      reg_next_success = mapper.RegisterNextImage(mapper_options, next_image_id);

      if (reg_next_success) {
        break;
      } else {
        LOG(INFO) << "=> Could not register, trying another image.";

        // If initial pair fails to continue for some time,
        // abort and try different initial pair.
        const size_t kMinNumInitialRegTrials = 30;
        if (reg_trial >= kMinNumInitialRegTrials && reconstruction->NumRegImages() < static_cast<size_t>(this->Options()->min_model_size)) {
          break;
        }
      }
    }

    if (reg_next_success) {
      mapper.setIsInitPair(false);  // atleast 3 images registered. allow global ba with odometry fusion
      mapper.TriangulateImage(this->Options()->Triangulation(), next_image_id);
      mapper.IterativeLocalRefinement(this->Options()->ba_local_max_refinements,
                                      this->Options()->ba_local_max_refinement_change,
                                      mapper_options,
                                      this->Options()->LocalBundleAdjustment(),
                                      this->Options()->Triangulation(),
                                      next_image_id);

      if (CheckRunGlobalRefinement(*reconstruction, ba_prev_num_reg_images, ba_prev_num_points)) {
        tcf::IterativeGlobalRefinement(*this->Options(), mapper_options, mapper);
        ba_prev_num_points = reconstruction->NumPoints3D();
        ba_prev_num_reg_images = reconstruction->NumRegImages();
      }

      if (this->Options()->extract_colors) {
        ExtractColors(this->ImagePath(), next_image_id, *reconstruction);
      }

      if (this->Options()->snapshot_images_freq > 0 &&
          reconstruction->NumRegImages() >= this->Options()->snapshot_images_freq + snapshot_prev_num_reg_images) {
        snapshot_prev_num_reg_images = reconstruction->NumRegImages();
        WriteSnapshot(*reconstruction, this->Options()->snapshot_path);
      }

      Callback(NEXT_IMAGE_REG_CALLBACK);
    }

    const size_t max_model_overlap = static_cast<size_t>(this->Options()->max_model_overlap);
    if (mapper.NumSharedRegImages() >= max_model_overlap) {
      break;
    }

    // If no image could be registered, try a single final global iterative
    // bundle adjustment and try again to register one image. If this fails
    // once, then exit the incremental mapping.
    if (!reg_next_success && prev_reg_next_success) {
      mapper.setIsInitPair(false);  // failed to register another image for submodule
      IterativeGlobalRefinement(*this->Options(), mapper_options, mapper);
    }
  } while (reg_next_success || prev_reg_next_success);

  if (CheckIfStopped()) {
    return Status::INTERRUPTED;
  }

  // Only run final global BA, if last incremental BA was not global.
  if (reconstruction->NumRegImages() >= 2 && reconstruction->NumRegImages() != ba_prev_num_reg_images &&
      reconstruction->NumPoints3D() != ba_prev_num_points) {
    IterativeGlobalRefinement(*this->Options(), mapper_options, mapper);
  }
  return Status::SUCCESS;
}

colmap::IncrementalPipeline::Status tcf::IncrementalFusionPipeline::InitializeReconstruction(
    tcf::IncrementalFusionMapper& mapper,
    const colmap::IncrementalMapper::Options& mapper_options,
    colmap::Reconstruction& reconstruction) {
  LOG(INFO) << "Custom reconstruction initialization triggered!";

  using namespace colmap;

  image_t image_id1 = static_cast<image_t>(this->Options()->init_image_id1);
  image_t image_id2 = static_cast<image_t>(this->Options()->init_image_id2);

  // Try to find good initial pair.
  TwoViewGeometry two_view_geometry;
  if (!this->Options()->IsInitialPairProvided()) {
    LOG(INFO) << "Finding good initial image pair";
    const bool find_init_success = mapper.FindInitialImagePair(mapper_options, two_view_geometry, image_id1, image_id2);
    if (!find_init_success) {
      LOG(INFO) << "=> No good initial image pair found.";
      return Status::NO_INITIAL_PAIR;
    }
  } else {
    if (!reconstruction.ExistsImage(image_id1) || !reconstruction.ExistsImage(image_id2)) {
      LOG(INFO) << StringPrintf("=> Initial image pair #%d and #%d do not exist.", image_id1, image_id2);
      return Status::BAD_INITIAL_PAIR;
    }
    const bool provided_init_success = mapper.EstimateInitialTwoViewGeometry(mapper_options, two_view_geometry, image_id1, image_id2);
    if (!provided_init_success) {
      LOG(INFO) << "Provided pair is insuitable for intialization.";
      return Status::BAD_INITIAL_PAIR;
    }
  }

  LOG(INFO) << StringPrintf("Initializing with image pair #%d and #%d", image_id1, image_id2);
  mapper.RegisterInitialImagePair(mapper_options, two_view_geometry, image_id1, image_id2);

  LOG(INFO) << "Global bundle adjustment";
  const bool is_init_pair = true;
  mapper.AdjustGlobalBundle(mapper_options, this->Options()->GlobalBundleAdjustment(), is_init_pair);
  reconstruction.Normalize();
  mapper.FilterPoints(mapper_options);
  mapper.FilterImages(mapper_options);

  // Initial image pair failed to register.
  if (reconstruction.NumRegImages() == 0 || reconstruction.NumPoints3D() == 0) {
    return Status::BAD_INITIAL_PAIR;
  }

  if (this->Options()->extract_colors) {
    ExtractColors(this->ImagePath(), image_id1, reconstruction);
  }
  return Status::SUCCESS;
}

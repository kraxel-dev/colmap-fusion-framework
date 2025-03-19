#include "tightly_coupled_fusion/sfm/incremental_mapper.h"

#include <colmap/estimators/pose.h>
#include <colmap/util/misc.h>
#include <fusion_helper/rr_fusion_recorder.h>

void tcf::IncrementalFusionMapper::IterativeGlobalRefinement(int max_num_refinements,
                                                             double max_refinement_change,
                                                             const Options& options,
                                                             const colmap::BundleAdjustmentOptions& ba_options,
                                                             const colmap::IncrementalTriangulator::Options& tri_options,
                                                             bool normalize_reconstruction) {
  CompleteAndMergeTracks(tri_options);
  VLOG(1) << "Iterative Global Refinement with fusion bundle adjustment!";
  VLOG(1) << "=> Retriangulated observations: " << Retriangulate(tri_options);
  for (int i = 0; i < max_num_refinements; ++i) {
    const size_t num_observations = this->Reconstruction()->ComputeNumObservations();
    this->AdjustGlobalBundle(options, ba_options, this->isInitPair());
    if (normalize_reconstruction && !options.use_prior_position) {
      // Normalize scene for numerical stability and
      // to avoid large scale changes in the viewer.
      this->Reconstruction()->Normalize();
    }
    size_t num_changed_observations = CompleteAndMergeTracks(tri_options);
    num_changed_observations += FilterPoints(options);
    const double changed = num_observations == 0 ? 0 : static_cast<double>(num_changed_observations) / num_observations;
    VLOG(1) << colmap::StringPrintf("=> Changed observations: %.6f", changed);
    if (changed < max_refinement_change) {
      break;
    }
  }
  ClearModifiedPoints3D();
}

bool tcf::IncrementalFusionMapper::AdjustGlobalBundle(const Options& options,
                                                      const colmap::BundleAdjustmentOptions& ba_options,
                                                      const bool is_init_pair) {
  using namespace colmap;
  VLOG(1) << "Adjusting global bundle with fusion capapbilites!";
  THROW_CHECK_NOTNULL(this->Reconstruction());

  const std::set<image_t>& reg_image_ids = this->Reconstruction()->RegImageIds();

  THROW_CHECK_GE(reg_image_ids.size(), 2) << "At least two images must be "
                                             "registered for global "
                                             "bundle-adjustment";

  BundleAdjustmentOptions custom_ba_options = ba_options;
  // Use stricter convergence criteria for first registered images.
  const size_t kMinNumRegImagesForFastBA = 10;
  if (reg_image_ids.size() < kMinNumRegImagesForFastBA) {
    custom_ba_options.solver_options.function_tolerance /= 10;
    custom_ba_options.solver_options.gradient_tolerance /= 10;
    custom_ba_options.solver_options.parameter_tolerance /= 10;
    custom_ba_options.solver_options.max_num_iterations *= 2;
    custom_ba_options.solver_options.max_linear_solver_iterations = 200;
  }

  // Avoid degeneracies in bundle adjustment.
  this->ObservationManager().FilterObservationsWithNegativeDepth();

  // Configure bundle adjustment.
  BundleAdjustmentConfig ba_config;
  for (const colmap::image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  // Fix the existing images, if option specified.
  if (options.fix_existing_images) {
    for (const colmap::image_t image_id : reg_image_ids) {
      if (this->ExistingImageIds().count(image_id)) {
        ba_config.SetConstantCamPose(image_id);
      }
    }
  }

  // Only use prior pose if at least 3 images have been registered.
  const bool use_prior_position = options.use_prior_position && reg_image_ids.size() > 2;

  // -------------------- Custom BA fusion code
  std::unique_ptr<BundleAdjuster> bundle_adjuster;
  tcf::FusionGraphBundleAdjustmentOptions fusion_options;
  fuhe::rrfuse::RerunFusionVisOptions rr_options;

  // FIXME: investigate and bring back in
  // if (fusion_options.fix_first_campose) {
  //   // Fix 7-DOFs of the bundle adjustment problem.
  //   auto reg_image_ids_it = reg_image_ids.begin();
  //   ba_config.SetConstantCamPose(*(reg_image_ids_it++));  // 1st image
  //   if (!options.fix_existing_images || !this->ExistingImageIds().count(*reg_image_ids_it)) {
  //     ba_config.SetConstantCamPositions(*reg_image_ids_it, {0});  // 2nd image
  //   }
  // }

  if (is_init_pair) {
    // during initial image pair registration do not use fusion BA // FIXME: investigate in future
    bundle_adjuster = CreateDefaultBundleAdjuster(std::move(custom_ba_options), std::move(ba_config), *this->Reconstruction());
  } else {
    // bundle adjuster with odometry fusion capabilities
    bundle_adjuster =
        std::make_unique<tcf::FusionGraphBundleAdjuster>(custom_ba_options, fusion_options, rr_options, ba_config, *this->Reconstruction());
  }

  return bundle_adjuster->Solve().termination_type != ceres::FAILURE;
}

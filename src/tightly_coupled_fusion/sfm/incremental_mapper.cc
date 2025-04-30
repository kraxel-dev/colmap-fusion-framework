#include "tightly_coupled_fusion/sfm/incremental_mapper.h"

#include "fusion_helper/io.h"
#include <colmap/estimators/pose.h>
#include <colmap/util/misc.h>
#include <fusion_helper/rr_fusion_recorder.h>

////////////////////////////////////////////////////////////////////////////////
// Incremental Mapper Rerun
////////////////////////////////////////////////////////////////////////////////

colmap::IncrementalMapper::LocalBundleAdjustmentReport tcf::IncrementalMapperRerun::AdjustLocalBundle(
    const Options& options,
    const colmap::BundleAdjustmentOptions& ba_options,
    const colmap::IncrementalTriangulator::Options& tri_options,
    colmap::image_t image_id,
    const std::unordered_set<colmap::point3D_t>& point3D_ids) {
  using namespace colmap;
  THROW_CHECK_NOTNULL(this->Reconstruction());
  THROW_CHECK(options.Check());
  LocalBundleAdjustmentReport report;

  // Find images that have most 3D points with given image in common.
  const std::vector<image_t> local_bundle = FindLocalBundle(options, image_id);

  // Do the bundle adjustment only if there is any connected images.
  if (local_bundle.size() > 0) {
    BundleAdjustmentConfig ba_config;
    ba_config.AddImage(image_id);
    for (const image_t local_image_id : local_bundle) {
      ba_config.AddImage(local_image_id);
    }

    // Fix the existing images, if option specified.
    if (options.fix_existing_images) {
      for (const image_t local_image_id : local_bundle) {
        if (ExistingImageIds().count(local_image_id)) {
          ba_config.SetConstantCamPose(local_image_id);
        }
      }
    }

    // Determine which cameras to fix, when not all the registered images
    // are within the current local bundle.
    std::unordered_map<camera_t, size_t> num_images_per_camera;
    for (const image_t image_id : ba_config.Images()) {
      const Image& image = this->Reconstruction()->Image(image_id);
      num_images_per_camera[image.CameraId()] += 1;
    }

    for (const auto& [camera_id, num_images] : num_images_per_camera) {
      const size_t num_reg_images_for_camera = NumRegImagesPerCamera().at(camera_id);
      if (num_images < num_reg_images_for_camera) {
        ba_config.SetConstantCamIntrinsics(camera_id);
      }
    }

    // Fix 7 DOF to avoid scale/rotation/translation drift in bundle adjustment.
    if (local_bundle.size() == 1) {
      ba_config.SetConstantCamPose(local_bundle[0]);
      ba_config.SetConstantCamPositions(image_id, {0});
    } else if (local_bundle.size() > 1) {
      const image_t image_id1 = local_bundle[local_bundle.size() - 1];
      const image_t image_id2 = local_bundle[local_bundle.size() - 2];
      ba_config.SetConstantCamPose(image_id1);
      if (!options.fix_existing_images || !ExistingImageIds().count(image_id2)) {
        ba_config.SetConstantCamPositions(image_id2, {0});
      }
    }

    // Make sure, we refine all new and short-track 3D points, no matter if
    // they are fully contained in the local image set or not. Do not include
    // long track 3D points as they are usually already very stable and adding
    // to them to bundle adjustment and track merging/completion would slow
    // down the local bundle adjustment significantly.
    std::unordered_set<point3D_t> variable_point3D_ids;
    for (const point3D_t point3D_id : point3D_ids) {
      const Point3D& point3D = this->Reconstruction()->Point3D(point3D_id);
      const size_t kMaxTrackLength = 15;
      if (!point3D.HasError() || point3D.track.Length() <= kMaxTrackLength) {
        ba_config.AddVariablePoint(point3D_id);
        variable_point3D_ids.insert(point3D_id);
      }
    }

    // Adjust the local bundle.
    std::unique_ptr<BundleAdjuster> bundle_adjuster = nullptr;
    // if rerun recorder object was given by user, log optimnization to rerun
    if (rr_recorder_) {
      bundle_adjuster = tcf::CreateDefaultBundleAdjusterRerun(
          ba_options,
          std::move(ba_config),
          *this->Reconstruction(),
          rr_recorder_);  // custom bundle adjuster with capability to log to rerun during optimization
    } else {              // default bundle adjuster without rerun logging
      bundle_adjuster = CreateDefaultBundleAdjuster(ba_options, std::move(ba_config), *this->Reconstruction());
    }
    const ceres::Solver::Summary summary = bundle_adjuster->Solve();

    report.num_adjusted_observations = summary.num_residuals / 2;

    // Merge refined tracks with other existing points.
    report.num_merged_observations = Triangulator().MergeTracks(tri_options, variable_point3D_ids);
    // Complete tracks that may have failed to triangulate before refinement
    // of camera pose and calibration in bundle-adjustment. This may avoid
    // that some points are filtered and it helps for subsequent image
    // registrations.
    report.num_completed_observations = Triangulator().CompleteTracks(tri_options, variable_point3D_ids);
    report.num_completed_observations += Triangulator().CompleteImage(tri_options, image_id);
  }

  // Filter both the modified images and all changed 3D points to make sure
  // there are no outlier points in the model. This results in duplicate work as
  // many of the provided 3D points may also be contained in the adjusted
  // images, but the filtering is not a bottleneck at this point.
  std::unordered_set<image_t> filter_image_ids;
  filter_image_ids.insert(image_id);
  filter_image_ids.insert(local_bundle.begin(), local_bundle.end());
  report.num_filtered_observations =
      this->ObservationManager().FilterPoints3DInImages(options.filter_max_reproj_error, options.filter_min_tri_angle, filter_image_ids);
  report.num_filtered_observations +=
      this->ObservationManager().FilterPoints3D(options.filter_max_reproj_error, options.filter_min_tri_angle, point3D_ids);

  return report;
}

void tcf::IncrementalMapperRerun::IterativeLocalRefinement(int max_num_refinements,
                                                           double max_refinement_change,
                                                           const Options& options,
                                                           const colmap::BundleAdjustmentOptions& ba_options,
                                                           const colmap::IncrementalTriangulator::Options& tri_options,
                                                           colmap::image_t image_id) {
  using namespace colmap;

  BundleAdjustmentOptions custom_ba_options = ba_options;
  for (int i = 0; i < max_num_refinements; ++i) {
    const auto report = this->AdjustLocalBundle(options, custom_ba_options, tri_options, image_id, GetModifiedPoints3D());
    VLOG(1) << "=> Merged observations: " << report.num_merged_observations;
    VLOG(1) << "=> Completed observations: " << report.num_completed_observations;
    VLOG(1) << "=> Filtered observations: " << report.num_filtered_observations;
    const double changed = report.num_adjusted_observations == 0
                               ? 0
                               : (report.num_merged_observations + report.num_completed_observations + report.num_filtered_observations) /
                                     static_cast<double>(report.num_adjusted_observations);
    VLOG(1) << StringPrintf("=> Changed observations: %.6f", changed);
    if (changed < max_refinement_change) {
      break;
    }
    // Only use robust cost function for first iteration.
    custom_ba_options.loss_function_type = BundleAdjustmentOptions::LossFunctionType::TRIVIAL;
  }
  ClearModifiedPoints3D();
}

bool tcf::IncrementalMapperRerun::AdjustGlobalBundle(const Options& options, const colmap::BundleAdjustmentOptions& ba_options) {
  using namespace colmap;

  THROW_CHECK_NOTNULL(this->Reconstruction());

  if (rr_recorder_) {
    fuhe::rrfuse::LogInfo(rr_recorder_->GetRerunRec(), "Global bundle adjustment!");
  }

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
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  // Fix the existing images, if option specified.
  if (options.fix_existing_images) {
    for (const image_t image_id : reg_image_ids) {
      if (this->ExistingImageIds().count(image_id)) {
        ba_config.SetConstantCamPose(image_id);
      }
    }
  }

  // Only use prior pose if at least 3 images have been registered.
  const bool use_prior_position = options.use_prior_position && reg_image_ids.size() > 2;

  std::unique_ptr<BundleAdjuster> bundle_adjuster;
  if (!use_prior_position) {
    // Fix 7-DOFs of the bundle adjustment problem.
    auto reg_image_ids_it = reg_image_ids.begin();
    ba_config.SetConstantCamPose(*(reg_image_ids_it++));  // 1st image
    if (!options.fix_existing_images || !this->ExistingImageIds().count(*reg_image_ids_it)) {
      ba_config.SetConstantCamPositions(*reg_image_ids_it, {0});  // 2nd image
    }

    if (rr_recorder_) {
      bundle_adjuster = CreateDefaultBundleAdjusterRerun(ba_options, std::move(ba_config), *this->Reconstruction(), rr_recorder_);
    } else {
      bundle_adjuster = CreateDefaultBundleAdjuster(ba_options, std::move(ba_config), *this->Reconstruction());
    }

  } else {
    LOG(ERROR) << "Pose prior bundle adjustment specified but not implemented with rerun yet!";
    //   PosePriorBundleAdjustmentOptions prior_options;
    //   prior_options.use_robust_loss_on_prior_position = options.use_robust_loss_on_prior_position;
    //   prior_options.prior_position_loss_scale = options.prior_position_loss_scale;
    //   bundle_adjuster = CreatePosePriorBundleAdjuster(
    //       std::move(custom_ba_options), prior_options, std::move(ba_config),  database_cache_->PosePriors(), *this->Reconstruction());
    throw std::runtime_error("Pose prior bundle adjustment not implemented with rerun yet!");
  }

  return bundle_adjuster->Solve().termination_type != ceres::FAILURE;
}

void tcf::IncrementalMapperRerun::IterativeGlobalRefinement(int max_num_refinements,
                                                            double max_refinement_change,
                                                            const Options& options,
                                                            const colmap::BundleAdjustmentOptions& ba_options,
                                                            const colmap::IncrementalTriangulator::Options& tri_options,
                                                            bool normalize_reconstruction) {
  using namespace colmap;

  CompleteAndMergeTracks(tri_options);
  VLOG(1) << "=> Retriangulated observations: " << Retriangulate(tri_options);
  for (int i = 0; i < max_num_refinements; ++i) {
    const size_t num_observations = this->Reconstruction()->ComputeNumObservations();
    this->AdjustGlobalBundle(options, ba_options);
    if (normalize_reconstruction && !options.use_prior_position) {
      // Normalize scene for numerical stability and
      // to avoid large scale changes in the viewer.
      this->Reconstruction()->Normalize(/*fixed_scale=*/true);
    }
    size_t num_changed_observations = CompleteAndMergeTracks(tri_options);
    num_changed_observations += FilterPoints(options);
    const double changed = num_observations == 0 ? 0 : static_cast<double>(num_changed_observations) / num_observations;
    VLOG(1) << StringPrintf("=> Changed observations: %.6f", changed);
    if (changed < max_refinement_change) {
      break;
    }
  }
  ClearModifiedPoints3D();
}

////////////////////////////////////////////////////////////////////////////////
// Incremental Fusion Mapper
////////////////////////////////////////////////////////////////////////////////

tcf::IncrementalFusionMapper::IncrementalFusionMapper(std::shared_ptr<const colmap::DatabaseCache> database_cache,
                                                      FusionGraphBundleAdjustmentOptions& fusion_options,
                                                      const std::string& tum_file,
                                                      fuhe::rrfuse::RerunFusionVisOptions& rr_options)
    : IncrementalMapper(database_cache), fusion_options_{fusion_options}, tum_file_{tum_file}, rr_options_{rr_options} {
  // whether to allow fusion or not
  is_fusion_mapping_ = fusion_options.is_mapping_with_fusion;
  // get image ids images in time sorted order
  auto imgs_by_stamp = fuhe::col_utils::ImageIdsByStamp(database_cache->Images());
  // get absolute poses from external odom sensor sorted by stamps
  fuhe::types::MapOfPosesSec metric_poses;
  fuhe::io::TumToPosesEigen(tum_file, metric_poses, true);

  // -------------------- Create sequential image edges in sorted order (containing odom edges between images)
  fusion_graph_data_edges_ =
      std::make_shared<fuhe::edges::MapOfImageEdges>(fuhe::edges::CreateSequentialImageEdges(imgs_by_stamp, metric_poses));
}

void tcf::IncrementalFusionMapper::IterativeLocalRefinement(int max_num_refinements,
                                                            double max_refinement_change,
                                                            const Options& options,
                                                            const colmap::BundleAdjustmentOptions& ba_options,
                                                            const colmap::IncrementalTriangulator::Options& tri_options,
                                                            colmap::image_t image_id) {
  using namespace colmap;

  BundleAdjustmentOptions custom_ba_options = ba_options;
  for (int i = 0; i < max_num_refinements; ++i) {
    const auto report = this->AdjustLocalBundle(options, custom_ba_options, tri_options, image_id, GetModifiedPoints3D());
    VLOG(1) << "=> Merged observations: " << report.num_merged_observations;
    VLOG(1) << "=> Completed observations: " << report.num_completed_observations;
    VLOG(1) << "=> Filtered observations: " << report.num_filtered_observations;
    const double changed = report.num_adjusted_observations == 0
                               ? 0
                               : (report.num_merged_observations + report.num_completed_observations + report.num_filtered_observations) /
                                     static_cast<double>(report.num_adjusted_observations);
    VLOG(1) << StringPrintf("=> Changed observations: %.6f", changed);
    if (changed < max_refinement_change) {
      break;
    }
    // Only use robust cost function for first iteration.
    custom_ba_options.loss_function_type = BundleAdjustmentOptions::LossFunctionType::TRIVIAL;
  }
  ClearModifiedPoints3D();
}

colmap::IncrementalMapper::LocalBundleAdjustmentReport tcf::IncrementalFusionMapper::AdjustLocalBundle(
    const Options& options,
    const colmap::BundleAdjustmentOptions& ba_options,
    const colmap::IncrementalTriangulator::Options& tri_options,
    colmap::image_t image_id,
    const std::unordered_set<colmap::point3D_t>& point3D_ids) {
  using namespace colmap;
  THROW_CHECK_NOTNULL(this->Reconstruction());
  THROW_CHECK(options.Check());
  LocalBundleAdjustmentReport report;

  if (rr_recorder_) {
    fuhe::rrfuse::LogInfo(rr_recorder_->GetRerunRec(), "Local bundle adjustment!");
  }

  // Find images that have most 3D points with given image in common.
  const std::vector<image_t> local_bundle = FindLocalBundle(options, image_id);

  // Do the bundle adjustment only if there is any connected images.
  if (local_bundle.size() > 0) {
    BundleAdjustmentConfig ba_config;
    ba_config.AddImage(image_id);
    for (const image_t local_image_id : local_bundle) {
      ba_config.AddImage(local_image_id);
    }

    // Fix the existing images, if option specified.
    if (options.fix_existing_images) {
      for (const image_t local_image_id : local_bundle) {
        if (ExistingImageIds().count(local_image_id)) {
          ba_config.SetConstantCamPose(local_image_id);
        }
      }
    }

    // Determine which cameras to fix, when not all the registered images
    // are within the current local bundle.
    std::unordered_map<camera_t, size_t> num_images_per_camera;
    for (const image_t image_id : ba_config.Images()) {
      const Image& image = this->Reconstruction()->Image(image_id);
      num_images_per_camera[image.CameraId()] += 1;
    }

    for (const auto& [camera_id, num_images] : num_images_per_camera) {
      const size_t num_reg_images_for_camera = NumRegImagesPerCamera().at(camera_id);
      if (num_images < num_reg_images_for_camera) {
        ba_config.SetConstantCamIntrinsics(camera_id);
      }
    }

    // Fix 7 DOF to avoid scale/rotation/translation drift in bundle adjustment.
    if (fusion_options_.fix_first_cam_pose) {
      if (local_bundle.size() == 1) {
        ba_config.SetConstantCamPose(local_bundle[0]);
        ba_config.SetConstantCamPositions(image_id, {0});
      } else if (local_bundle.size() > 1) {
        // vanilla approach to fix poses of least connected images (in case choosing img by oldest time below does not work)
        image_t image_id1 = local_bundle[local_bundle.size() - 1];
        image_t image_id2 = local_bundle[local_bundle.size() - 2];
        // find 2 earliest images in local bundle in time to fix its pose for local BA (more consistent to sequential order of fusion
        // mapping)
        bool found_earliest = false;  // signal if oldest img was found to proceed with 2nd oldest
        for (const auto& img_edge : *this->FusionGraphDataEdges()) {
          // find earliest image of whole graph that is contained in local bundle
          if (std::find(local_bundle.begin(), local_bundle.end(), img_edge.second.CurrId()) != local_bundle.end()) {
            if (!found_earliest) {
              VLOG(3) << "Earliest image in local bundle identified!";
              image_id1 = img_edge.second.CurrId();
              found_earliest = true;
              continue;
            } else {
              image_id2 = img_edge.second.CurrId();
              break;
            }
          }
        }
        VLOG(2) << "Tagging pose of image " << image_id1 << " as fixed in ba_config of upcoming local bundle!";
        ba_config.SetConstantCamPose(image_id1);
        if (!options.fix_existing_images || !ExistingImageIds().count(image_id2)) {
          // do not fix position of 2nd image if scale should be ajusted during optim (desired in fusion)
          if (fusion_options_.fix_second_cam_position) {
            VLOG(2) << "Tagging position of image " << image_id2 << " as fixed in ba_config of upcoming local bundle!";
            ba_config.SetConstantCamPositions(image_id2, {0});  // 2nd image
          }
        }
      }
    }

    // Make sure, we refine all new and short-track 3D points, no matter if
    // they are fully contained in the local image set or not. Do not include
    // long track 3D points as they are usually already very stable and adding
    // to them to bundle adjustment and track merging/completion would slow
    // down the local bundle adjustment significantly.
    std::unordered_set<point3D_t> variable_point3D_ids;
    for (const point3D_t point3D_id : point3D_ids) {
      const Point3D& point3D = this->Reconstruction()->Point3D(point3D_id);
      const size_t kMaxTrackLength = 15;
      if (!point3D.HasError() || point3D.track.Length() <= kMaxTrackLength) {
        ba_config.AddVariablePoint(point3D_id);
        variable_point3D_ids.insert(point3D_id);
      }
    }

    // Adjust the local bundle.
    std::unique_ptr<BundleAdjuster> bundle_adjuster = nullptr;
    // if fusion is deactivated, use default bundle adjuster
    if (!fusion_options_.is_mapping_with_fusion || !fusion_options_.fusion_in_local_ba) {
      if (rr_recorder_) {
        // custom bundle adjuster with capability to log to rerun during optimization
        bundle_adjuster = tcf::CreateDefaultBundleAdjusterRerun(ba_options, std::move(ba_config), *this->Reconstruction(), rr_recorder_);
      } else {
        // default bundle adjuster without rerun logging
        bundle_adjuster = CreateDefaultBundleAdjuster(ba_options, std::move(ba_config), *this->Reconstruction());
      }
    } else {
      // custom bundle adjuster with fusion capabilities
      bundle_adjuster = tcf::CreateFusionGraphBundleAdjuster(
          ba_options, fusion_options_, rr_options_, rr_recorder_, ba_config, *this->Reconstruction(), *this->FusionGraphDataEdges());
    }

    const ceres::Solver::Summary summary = bundle_adjuster->Solve();

    report.num_adjusted_observations = summary.num_residuals / 2;

    // Merge refined tracks with other existing points.
    report.num_merged_observations = Triangulator().MergeTracks(tri_options, variable_point3D_ids);
    // Complete tracks that may have failed to triangulate before refinement
    // of camera pose and calibration in bundle-adjustment. This may avoid
    // that some points are filtered and it helps for subsequent image
    // registrations.
    report.num_completed_observations = Triangulator().CompleteTracks(tri_options, variable_point3D_ids);
    report.num_completed_observations += Triangulator().CompleteImage(tri_options, image_id);
  }

  // Filter both the modified images and all changed 3D points to make sure
  // there are no outlier points in the model. This results in duplicate work as
  // many of the provided 3D points may also be contained in the adjusted
  // images, but the filtering is not a bottleneck at this point.
  std::unordered_set<image_t> filter_image_ids;
  filter_image_ids.insert(image_id);
  filter_image_ids.insert(local_bundle.begin(), local_bundle.end());
  report.num_filtered_observations =
      this->ObservationManager().FilterPoints3DInImages(options.filter_max_reproj_error, options.filter_min_tri_angle, filter_image_ids);
  report.num_filtered_observations +=
      this->ObservationManager().FilterPoints3D(options.filter_max_reproj_error, options.filter_min_tri_angle, point3D_ids);

  return report;
}

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
    this->AdjustGlobalBundle(options, ba_options);
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

bool tcf::IncrementalFusionMapper::AdjustGlobalBundle(const Options& options, const colmap::BundleAdjustmentOptions& ba_options) {
  using namespace colmap;
  VLOG(1) << "Adjusting global bundle with fusion capapbilites!";
  if (rr_recorder_) {
    fuhe::rrfuse::LogInfo(rr_recorder_->GetRerunRec(), "Global bundle adjustment!");
  }
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

  // -------------------- Custom BA fusion code
  std::unique_ptr<BundleAdjuster> bundle_adjuster;

  // fix pose of 1st image and optionally 2nd image
  if (fusion_options_.fix_first_cam_pose) {
    // Fix 7-DOFs of the bundle adjustment problem.
    auto reg_image_ids_it = reg_image_ids.begin();
    VLOG(2) << "Tagging pose of image " << *reg_image_ids_it << " as fixed in ba_config for upcoming global bundle!";
    ba_config.SetConstantCamPose(*(reg_image_ids_it++));  // 1st image
    if (!options.fix_existing_images || !this->ExistingImageIds().count(*reg_image_ids_it)) {
      // do not fix position of 2nd image if scale should be ajusted during optim (desired in fusion)
      if (fusion_options_.fix_second_cam_position) {
        ba_config.SetConstantCamPositions(*reg_image_ids_it, {0});  // 2nd image
      }
    }
  }

  if (!fusion_options_.is_mapping_with_fusion || !fusion_options_.fusion_in_global_ba) {
    // default global BA with rerun visualization
    bundle_adjuster =
        CreateDefaultBundleAdjusterRerun(std::move(custom_ba_options), std::move(ba_config), *this->Reconstruction(), rr_recorder_);
  } else {
    // bundle adjuster with odometry fusion capabilities
    bundle_adjuster = tcf::CreateFusionGraphBundleAdjuster(
        custom_ba_options, fusion_options_, rr_options_, rr_recorder_, ba_config, *this->Reconstruction(), *this->FusionGraphDataEdges());
  }

  return bundle_adjuster->Solve().termination_type != ceres::FAILURE;
}
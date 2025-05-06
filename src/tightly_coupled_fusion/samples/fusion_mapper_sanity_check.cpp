/**
 * @file fusion_mapper_sanity_check.cpp
 * @author kraxel
 * @brief Sanity check for the FusionIncrementalMapper class (with rerun logging capabilities). Applies manual steps of Mapper (described
 * in: orig colmap repo src/colmap/sfm/incremental_mapper.h) to reconstruct a model from scratch with fusion (odometry) capabilities. Order
 * of images will be sorted by ascending time. Very first and 2nd image (by time) in database are forced as initial pair for mapping. Camera
 * intrinsics are fixed.
 * @version 0.1
 * @date 2025-04-23
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "tightly_coupled_fusion/sfm/incremental_mapper.h"
#include <colmap/controllers/incremental_pipeline.h>
#include <colmap/controllers/option_manager.h>
#include <colmap/scene/database_cache.h>
#include <fusion_helper/col_utils.h>
#include <fusion_helper/io.h>
#include <fusion_helper/rr_fusion_recorder.h>

/**
 * @brief Whether to run global refinement after current img registration. Taken from incremental_pipeline.cc in orig colmap repo.
 *
 * @param reconstruction
 * @param incr_pipieline_opts
 * @param ba_prev_num_reg_images
 * @param ba_prev_num_points
 * @return true
 * @return false
 */
bool CheckRunGlobalRefinement(const colmap::Reconstruction& reconstruction,
                              const colmap::IncrementalPipelineOptions& incr_pipieline_opts,
                              const size_t ba_prev_num_reg_images,
                              const size_t ba_prev_num_points) {
  return reconstruction.NumRegImages() >= incr_pipieline_opts.ba_global_images_ratio * ba_prev_num_reg_images ||
         reconstruction.NumRegImages() >= incr_pipieline_opts.ba_global_images_freq + ba_prev_num_reg_images ||
         reconstruction.NumPoints3D() >= incr_pipieline_opts.ba_global_points_ratio * ba_prev_num_points ||
         reconstruction.NumPoints3D() >= incr_pipieline_opts.ba_global_points_freq + ba_prev_num_points;
}

/**
 * @brief Taken from incremental_pipeline.cc in orig colmap repo
 *
 * @param image_path
 * @param image_id
 * @param reconstruction
 */
void ExtractColors(const std::string& image_path, const colmap::image_t image_id, colmap::Reconstruction& reconstruction) {
  if (!reconstruction.ExtractColorsForImage(image_id, image_path)) {
    LOG(WARNING) << colmap::StringPrintf(
        "Could not read image %s at path %s.", reconstruction.Image(image_id).Name().c_str(), image_path.c_str());
  }
}

int main(int argc, char** argv) {
  // -------------------- Parse COLMAP and Ceres inputs
  std::string db_path;  // database path
  std::string output_path;

  colmap::OptionManager col_options;                          // classic colmap options and cmd arg parser
  fuhe::rrfuse::RerunFusionVisOptions rr_options;             // rerun visualization options
  tcf::FusionGraphBundleAdjustmentOptions fusion_ba_options;  // options (e.g. tum path) for FusionGraphBundleAdjuster

  // classic colmap options
  col_options.AddRequiredOption("db_path", &db_path);
  col_options.AddRequiredOption("output_path", &output_path);
  // custom rerun option
  col_options.AddDefaultOption("rerun", &rr_options.is_log_to_rerun);
  col_options.AddDefaultOption("save_rrd", &rr_options.is_save_rerun_to_disk);
  col_options.AddDefaultOption("rerun_odom_as_pred", &rr_options.draw_rerun_odom_as_predicted_poses);
  // custom fusion options
  col_options.AddDefaultOption("fusion.is_mapping_with_fusion", &fusion_ba_options.is_mapping_with_fusion);
  col_options.AddDefaultOption("fusion.odom_cov", &fusion_ba_options.cov);
  col_options.AddDefaultOption("fusion.fusion_in_local_ba", &fusion_ba_options.fusion_in_local_ba);
  col_options.AddDefaultOption("fusion.fusion_in_global_ba", &fusion_ba_options.fusion_in_global_ba);
  col_options.AddDefaultOption("fusion.brute_force_scale_recovery", &fusion_ba_options.brute_force_scale_recovery);
  col_options.AddDefaultOption("fusion.use_robust_loss_on_scale_estimation", &fusion_ba_options.use_robust_loss_on_scale_estimation);
  col_options.AddDefaultOption("fusion.fix_first_cam_pose", &fusion_ba_options.fix_first_cam_pose);
  col_options.AddDefaultOption("fusion.fix_second_cam_position", &fusion_ba_options.fix_first_cam_pose);

  // classic colmap BA solver options
  col_options.AddBundleAdjustmentOptions();
  col_options.AddMapperOptions();
  // actually parse command line args into option members
  col_options.Parse(argc, argv);

  // -------------------- Logging stuff
  colmap::InitializeGlog(argv);
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;

  // Set log directory
  const std::string log_dir = fuhe::io::GetRepoRootDir() + "/logs";
  FLAGS_log_dir = log_dir;
  VLOG(2) << "Logging path is: " << log_dir;

  // -------------------- Read database cache and init fusion Mapper object
  colmap::Database db = colmap::Database(db_path);
  std::shared_ptr<colmap::DatabaseCache> db_cache = colmap::DatabaseCache::Create(db, 0, false, {});
  tcf::IncrementalFusionMapper fusion_mapper(db_cache, fusion_ba_options, fusion_ba_options.tum_file, rr_options);  // fusion mapper object
  // create empty reconstruction
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  VLOG(1) << "Begin reconstruction!";
  fusion_mapper.BeginReconstruction(reconstruction);

  // -------------------- Init rerun if visualization is toggled
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_rc = nullptr;
  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    VLOG(1) << "Rerun recording toggled. Attaching Recorder manager to mapper!";
    rr_rc = std::make_shared<fuhe::rrfuse::RerunFusionRecorder>(rr_options, *fusion_mapper.Reconstruction());
    fusion_mapper.AttachRerunRecorder(rr_rc);
  }

  // -------------------- Define order of images for active reconstructing
  // obtain image ids in database sorted by ascending time
  fuhe::types::MapOfImageIdsSec img_ids_sorted = fuhe::col_utils::ImageIdsByStamp(db_cache->Images());

  // -------------------- Options mingling
  auto incr_pipieline_opts = col_options.mapper;
  // force fix camera intrinsics
  incr_pipieline_opts->ba_refine_extra_params = false;
  incr_pipieline_opts->ba_refine_focal_length = false;
  incr_pipieline_opts->ba_refine_principal_point = false;

  ////////////////////////////////////////////////////////////////////////////////
  // Init Pair Registration
  ////////////////////////////////////////////////////////////////////////////////

  // force lower initial pair requirements
  colmap::IncrementalMapper::Options mapper_opts = incr_pipieline_opts->Mapper();
  mapper_opts.init_min_num_inliers = 50;
  mapper_opts.init_min_tri_angle = 0.013;     // degrees
  mapper_opts.init_max_forward_motion = 1.0;  // essential matrix z motion

  // -------------------- Force Select intial image pair
  colmap::TwoViewGeometry tvg;                                       // Essential matrix and (filtered matches) between initial image pair
  colmap::image_t id_1 = img_ids_sorted.begin()->second;             // very first image in tajectory sequence
  colmap::image_t id_2 = std::next(img_ids_sorted.begin())->second;  // second image in sequence

  VLOG(2) << "Trying to force 1st and 2nd image as initial pair of new reconstruction!";
  VLOG(2) << "Ids : " << id_1 << " and " << id_2;
  // try to force first and second image of whole sequence as initial pair
  bool init_succes = fusion_mapper.EstimateInitialTwoViewGeometry(mapper_opts, tvg, id_1, id_2);
  fuhe::col_utils::PrintTwoViewStatistics(tvg);
  if (!init_succes) {
    LOG(FATAL) << "Could not find initial image pair for reconstruction!";
    return 1;
  }

  VLOG(1) << "Initial Pair found with ids: " << id_1 << " and " << id_2;
  fusion_mapper.RegisterInitialImagePair(mapper_opts, tvg, id_1, id_2);  // lock in

  // -------------------- Initial Pair Rerun visualization
  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    rr_rc->UpdateRerunTimeStep();

    // log initial pair to rerun
    fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                    rr_rc->GetRerunPinhole(),
                                    fuhe::col_utils::RegisteredImages(fusion_mapper.Reconstruction()),
                                    fusion_mapper.Reconstruction()->Points3D());
  }
  // -------------------- One round of global bundle adjustment for the inital pair

  VLOG(1) << "Kick off a round of global bundle adjustment for initial par!";
  fusion_mapper.AdjustGlobalBundle(mapper_opts, incr_pipieline_opts->GlobalBundleAdjustment());
  // fusion_mapper.Reconstruction()->Normalize(/*fixed_scale=*/true);
  fusion_mapper.Reconstruction()->Normalize();
  // separate set of 3d point filtering due to low triangulation angle in init
  colmap::IncrementalMapper::Options init_filter_mapper_opts = mapper_opts;
  init_filter_mapper_opts.filter_min_tri_angle = init_filter_mapper_opts.init_min_tri_angle * 1.5;
  fusion_mapper.FilterPoints(init_filter_mapper_opts);

  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    rr_rc->UpdateRerunTimeStep();

    // log initial pair to rerun
    fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                    rr_rc->GetRerunPinhole(),
                                    fuhe::col_utils::RegisteredImages(fusion_mapper.Reconstruction()),
                                    fusion_mapper.Reconstruction()->Points3D());
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Continues image registration and mapping
  ////////////////////////////////////////////////////////////////////////////////
  size_t ba_prev_num_reg_images = reconstruction->NumRegImages();
  size_t ba_prev_num_points = reconstruction->NumPoints3D();

  // -------------------- Iterate over all time sorted images to register them
  VLOG(1) << "Begin reconstruction process!";
  double prev_stamp = 0;
  for (const auto& [stamp, next_image_id] : img_ids_sorted) {
    // skip inital pair
    if (next_image_id == id_1 || next_image_id == id_2) {
      prev_stamp = stamp;  // store stamps from initial pair for time compare
      continue;
    }

    VLOG(2) << "Registering image " << next_image_id;
    // register image in reconstruction
    if (!fusion_mapper.RegisterNextImage(mapper_opts, next_image_id)) {
      VLOG(1) << "Registration of current image failed! Stopping mapping process!";
      // break if registration fails TODO: make more robust and allow to try other images for reg before failing
      break;
    }

    if (rr_options.is_log_to_rerun) {
      fuhe::rrfuse::LogInfo(rr_rc->GetRerunRec(), "Registered image: " + std::to_string(next_image_id));
    }

    VLOG(2) << "Triangulating new points for image " << next_image_id;
    // triangulate new points and run a couple rounds of local BA
    fusion_mapper.TriangulateImage(incr_pipieline_opts->Triangulation(), next_image_id);

    // -------------------- Rerun visualization of newly registered image
    if (rr_options.is_log_to_rerun) {
      // initialize recorder objects when rerun logging is desired
      rr_rc->UpdateRerunTimeStep();

      // log initial pair to rerun
      fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                      rr_rc->GetRerunPinhole(),
                                      fuhe::col_utils::RegisteredImages(fusion_mapper.Reconstruction()),
                                      fusion_mapper.Reconstruction()->Points3D());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Local Bundle Adjustment
    ////////////////////////////////////////////////////////////////////////////////

    VLOG(2) << "Iterative local bundle adjustments!";
    fusion_mapper.IterativeLocalRefinement(incr_pipieline_opts->ba_local_max_refinements,
                                           incr_pipieline_opts->ba_local_max_refinement_change,
                                           mapper_opts,
                                           incr_pipieline_opts->LocalBundleAdjustment(),
                                           incr_pipieline_opts->Triangulation(),
                                           next_image_id);

    ////////////////////////////////////////////////////////////////////////////////
    // Global BA that is called only after x amount of registered images
    ////////////////////////////////////////////////////////////////////////////////
    if (CheckRunGlobalRefinement(*reconstruction, *incr_pipieline_opts, ba_prev_num_reg_images, ba_prev_num_points)) {
      VLOG(2) << "Enough imgs registered since last global BA. Global  bundle adjustments toggled!";
      fusion_mapper.IterativeGlobalRefinement(incr_pipieline_opts->ba_global_max_refinements,
                                              incr_pipieline_opts->ba_global_max_refinement_change,
                                              mapper_opts,
                                              incr_pipieline_opts->GlobalBundleAdjustment(),
                                              incr_pipieline_opts->Triangulation(),
                                              /*normalize */ false);
      ba_prev_num_points = reconstruction->NumPoints3D();
      ba_prev_num_reg_images = reconstruction->NumRegImages();
    }

    // TODO: bring in again once we have sorted out correct image path
    // if (incr_pipieline_opts->extract_colors) {
    //   ExtractColors(*col_options.image_path, next_image_id, *reconstruction);
    // }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Done with all images. Final round of global BA
  ////////////////////////////////////////////////////////////////////////////////
  // Only run final global BA, if last incremental BA was not global.
  if (reconstruction->NumRegImages() >= 2 && reconstruction->NumRegImages() != ba_prev_num_reg_images &&
      reconstruction->NumPoints3D() != ba_prev_num_points) {
    VLOG(1) << "Final round of global bundle adjustment!";
    fusion_mapper.IterativeGlobalRefinement(incr_pipieline_opts->ba_global_max_refinements,
                                            incr_pipieline_opts->ba_global_max_refinement_change,
                                            mapper_opts,
                                            incr_pipieline_opts->GlobalBundleAdjustment(),
                                            incr_pipieline_opts->Triangulation(),
                                            /*normalize */ false);
  }

  VLOG(1) << "Done registering all images in reconstruction!";
  VLOG(1) << "Writing model to: " << output_path;
  reconstruction->WriteText(output_path);

  fusion_mapper.EndReconstruction(/*false*/ false);  // finalize reconstruction

  return EXIT_SUCCESS;
}
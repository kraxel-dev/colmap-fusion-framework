/**
 * @file fusion_mapper_sanity_check.cpp
 * @author kraxel
 * @brief Sanity check for the FusionIncrementalMapper class (with rerun logging capabilities). Applies manual steps of Mapper
 * (described in: orig colmap repo src/colmap/sfm/incremental_mapper.h) to reconstruct a model from scratch with fusion
 * (odometry) capabilities. Order of images will be sorted by ascending time. Very first and 2nd image (by time) in database are
 * forced as initial pair for mapping. Camera intrinsics are fixed.
 * @version 0.1
 * @date 2025-04-23
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fusion_helper/frame_align_utils.h"
#include "tightly_coupled_fusion/sfm/incremental_mapper.h"
#include <colmap/controllers/incremental_pipeline.h>
#include <colmap/controllers/option_manager.h>
#include <colmap/scene/database_cache.h>
#include <fusion_helper/col_utils.h>
#include <fusion_helper/io.h>
#include <fusion_helper/rr_sfm_logger.h>

/**
 * @brief Whether to run global refinement after current img registration. Taken from incremental_pipeline.cc in orig colmap
 * repo.
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
  ////////////////////////////////////////////////////////////////////////////////
  // Parse COLMAP and ceres options and inputs
  ////////////////////////////////////////////////////////////////////////////////
  std::string db_path;  // database path
  std::string output_path;

  colmap::OptionManager col_options;                          // classic colmap options and cmd arg parser
  fuhe::rr::RerunVisualizationOptions rr_options;             // rerun visualization options
  tcf::FusionGraphBundleAdjustmentOptions fusion_ba_options;  // options (e.g. tum path) for FusionGraphBundleAdjuster
  fuhe::align::AlignmentOptions alignment_options;            // colmap reconstruction coordinate alingment options
  tcf::IncrementalFusionMapper::FusionMapperOptions fusion_mapper_options;  // options regarding the whole fusion mapping process

  // classic colmap options
  col_options.AddRequiredOption("db_path", &db_path);
  col_options.AddRequiredOption("output_path", &output_path);
  // custom rerun option
  col_options.AddDefaultOption("rerun", &rr_options.is_log_to_rerun);  // FIXME: change to flage to Rerun.log
  col_options.AddDefaultOption("save_rrd", &rr_options.is_save_rerun_to_disk);
  col_options.AddDefaultOption("rerun_odom_as_pred", &rr_options.draw_rerun_odom_as_predicted_poses);
  col_options.AddDefaultOption("Rerun.model_bbox_lower_bound", &rr_options.model_bbox_lb);
  col_options.AddDefaultOption("Rerun.model_bbox_upper_bound", &rr_options.model_bbox_ub);
  // custom fusion options
  col_options.AddDefaultOption("Fusion.is_mapping_with_fusion", &fusion_ba_options.is_mapping_with_fusion);
  col_options.AddDefaultOption("Fusion.tum_file", &fusion_ba_options.tum_file);
  col_options.AddDefaultOption("Fusion.time_diff_local_ba",
                               &fusion_ba_options.time_between_local_ba);  // seconds to pass to allow new round of local BA
  col_options.AddDefaultOption("Fusion.odom_cov", &fusion_ba_options.cov);
  col_options.AddDefaultOption("Fusion.fusion_in_local_ba", &fusion_ba_options.fusion_in_local_ba);
  col_options.AddDefaultOption("Fusion.fusion_in_global_ba", &fusion_ba_options.fusion_in_global_ba);
  col_options.AddDefaultOption("Fusion.brute_force_scale_recovery", &fusion_ba_options.brute_force_scale_recovery);
  col_options.AddDefaultOption("Fusion.use_robust_loss_on_scale_estimation",
                               &fusion_ba_options.use_robust_loss_on_scale_estimation);
  col_options.AddDefaultOption("Fusion.fix_first_cam_pose", &fusion_ba_options.fix_first_cam_pose);
  col_options.AddDefaultOption("Fusion.fix_second_cam_position", &fusion_ba_options.fix_first_cam_pose);
  // custom fusion mapping opts
  col_options.AddDefaultOption("FusionMapper.estimate_scale_on_init", &fusion_mapper_options.estimate_scale_on_init_ba);
  // custom frame alignment options
  col_options.AddDefaultOption("FrameAlign.n_reg_for_alignment", &alignment_options.n_reg_for_alignment);
  col_options.AddDefaultOption("FrameAlign.rotate_init_motion_onto_global_x_axis",
                               &alignment_options.rotate_init_motion_onto_global_x_axis);

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
  // fusion mapper object (the star of the show)
  tcf::IncrementalFusionMapper fusion_mapper(
      db_cache, fusion_ba_options, fusion_mapper_options, fusion_ba_options.tum_file, rr_options);
  // create empty reconstruction
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  VLOG(1) << "Begin reconstruction!";
  fusion_mapper.BeginReconstruction(reconstruction);

  // -------------------- Init rerun if visualization is toggled
  std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_logger = nullptr;
  if (rr_options.is_log_to_rerun) {
    // initialize rerun sfm logger objects when rerun logging is desired
    VLOG(1) << "Rerun recording toggled. Attaching sfm logger to mapper!";
    rr_logger = std::make_shared<fuhe::rr::RerunSfmLogger>(rr_options, fusion_mapper.Reconstruction());
    fusion_mapper.AttachRerunSfmLogger(rr_logger);
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
  colmap::TwoViewGeometry tvg;                            // Essential matrix and (filtered matches) between initial image pair
  colmap::image_t id_1 = img_ids_sorted.begin()->second;  // very first image in tajectory sequence
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
  // log initial pair to rerun
  if (rr_options.is_log_to_rerun) {
    // establish model bbox from 3d points of initial pair to filter out pts in rerun that would cause mayhem in the viewer
    if (rr_options.is_ignore_pts_beyond_model_bbox) {
      rr_logger->UpdateModelBBox();
    }

    rr_logger->LogFullReconstruction();
  }

  // -------------------- Global bundle adjustment for the inital pair
  VLOG(1) << "Kick off a round of global bundle adjustment for initial par!";
  fusion_mapper.AdjustGlobalBundle(mapper_opts, incr_pipieline_opts->GlobalBundleAdjustment());
  fusion_mapper.Reconstruction()->Normalize();
  // for first global ba use separate set of 3d point filtering options due to low triangulation angle in init
  colmap::IncrementalMapper::Options init_filter_mapper_opts = mapper_opts;
  init_filter_mapper_opts.filter_min_tri_angle = init_filter_mapper_opts.init_min_tri_angle * 1.5;
  fusion_mapper.FilterPoints(init_filter_mapper_opts);

  // log bundle adjusted initial pair to rerun
  if (rr_options.is_log_to_rerun) {
    rr_logger->LogFullReconstruction();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Continues image registration and mapping
  ////////////////////////////////////////////////////////////////////////////////
  size_t ba_prev_num_reg_images = reconstruction->NumRegImages();
  size_t ba_prev_num_points = reconstruction->NumPoints3D();

  // -------------------- Iterate over all time sorted images to register them
  VLOG(1) << "Begin reconstruction process!";
  double prev_local_ba_stamp = 0;  // img stamp of last successfully applied local ba process
  for (const auto& [stamp, next_image_id] : img_ids_sorted) {
    // skip inital pair
    if (next_image_id == id_1 || next_image_id == id_2) {
      prev_local_ba_stamp = stamp;  // store stamps from initial pair for time compare
      continue;
    }

    // register image in reconstruction
    VLOG(2) << "Registering image " << next_image_id;
    if (!fusion_mapper.RegisterNextImage(mapper_opts, next_image_id)) {
      LOG(ERROR) << "Registration of current image failed! Stopping mapping process!";
      // break if registration fails //FIXME: make more robust and allow to try other images for reg before failing
      break;
    }

    // text log to rerun
    if (rr_logger) {
      rr_logger->LogInfoMsg("Registered image: " + std::to_string(next_image_id));
    }

    // triangulate new points and run a couple rounds of local BA
    VLOG(2) << "Triangulating new points for image " << next_image_id;
    fusion_mapper.TriangulateImage(incr_pipieline_opts->Triangulation(), next_image_id);

    // perform colmap model coordinate frame alignment (once) if condition are met
    if (fuhe::align::CheckRunAlignment(reconstruction->NumRegImages(), alignment_options)) {
      // pca alignment, forcing 1st cam pose to a specfici intial value, etc. read frame_align_utils.h for more info
      fuhe::align::PerformAlignmentStrategies(reconstruction, alignment_options);
      // text log to rerun
      if (rr_options.is_log_to_rerun) {
        rr_logger->LogInfoMsg("Performed coordinate frame alignment strategies on colmap model!");
      }
    }

    // rerun visualization of newly registered image
    if (rr_logger) {
      rr_logger->LogFullReconstruction();
    }

    // TODO: bring in again once we have sorted out correct image path
    // if (incr_pipieline_opts->extract_colors) {
    //   ExtractColors(*col_options.image_path, next_image_id, *reconstruction);
    // }

    ////////////////////////////////////////////////////////////////////////////////
    // Local Bundle Adjustment
    ////////////////////////////////////////////////////////////////////////////////
    // perform local or global BA only if enough time has passed between current image and last employed BA image
    if (stamp - prev_local_ba_stamp < fusion_ba_options.time_between_local_ba) {
      VLOG(2) << "Time dff for new local BA not yet reached. Diff to last local BA only " << stamp - prev_local_ba_stamp
              << ". Skipping local and potential global BA!";
      continue;
    }

    VLOG(2) << "More than " << fusion_ba_options.time_between_local_ba
            << " seconds passed between images. Trigger iterative local refinmenent!";
    VLOG(2) << "Iterative local bundle adjustments!";
    fusion_mapper.IterativeLocalRefinement(incr_pipieline_opts->ba_local_max_refinements,
                                           incr_pipieline_opts->ba_local_max_refinement_change,
                                           mapper_opts,
                                           incr_pipieline_opts->LocalBundleAdjustment(),
                                           incr_pipieline_opts->Triangulation(),
                                           next_image_id);
    prev_local_ba_stamp = stamp;

    ////////////////////////////////////////////////////////////////////////////////
    // Global BA that is called only after x amount of registered images
    ////////////////////////////////////////////////////////////////////////////////
    if (CheckRunGlobalRefinement(*reconstruction, *incr_pipieline_opts, ba_prev_num_reg_images, ba_prev_num_points) &&
        reconstruction->NumRegImages() > mapper_opts.local_ba_num_images) {
      VLOG(2) << "Enough imgs registered since last global BA. Global bundle adjustments toggled!";
      fusion_mapper.IterativeGlobalRefinement(
          incr_pipieline_opts->ba_global_max_refinements,
          incr_pipieline_opts->ba_global_max_refinement_change,
          mapper_opts,
          incr_pipieline_opts->GlobalBundleAdjustment(),
          incr_pipieline_opts->Triangulation(),
          /*normalize (if fusion is toggled off)*/ !fusion_ba_options.is_mapping_with_fusion);
      ba_prev_num_points = reconstruction->NumPoints3D();
      ba_prev_num_reg_images = reconstruction->NumRegImages();
    }
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
                                            /*normalize (if fusion is toggled off)*/ !fusion_ba_options.is_mapping_with_fusion);
  }

  VLOG(1) << "Done registering all images in reconstruction!";
  fuhe::col_utils::ToTum(reconstruction.get(), output_path);
  VLOG(1) << "Writing model to: " << output_path;
  reconstruction->WriteText(output_path);

  fusion_mapper.EndReconstruction(/*false*/ false);  // finalize reconstruction

  return EXIT_SUCCESS;
}
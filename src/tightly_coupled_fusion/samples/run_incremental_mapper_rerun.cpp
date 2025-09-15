/**
 * @file run_incremental_mapper_rerun.cpp
 * @author kraxel
 * @brief Sanity check for the vanilla IncrementalMapper class (with rerun logging capabilities). Applies manual steps of Mapper
 * (described in: orig colmap repo src/colmap/sfm/incremental_mapper.h) to reconstruct a model from scratch without fusion
 * capabilities. Order of images will be sorted by ascending time. Very first and 2nd image (by time) in database are forced as
 * initial pair for mapping. Camera intrinsics are fixed. If model alignment is applied, normalization of the model in each
 * global BA will be turned off.
 * @version 0.1
 * @date 2025-03-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fusion_helper/frame_align_utils.h"
#include "tightly_coupled_fusion/estimators/bundle_adjustment.h"
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

int main(int argc, char** argv) {
  // -------------------- Parse COLMAP and Ceres inputs
  std::string db_path;  // database path
  std::string output_path;
  int n_init_pair_skip = 0;  // nr of imgs after first one to skip before selecting 2nd init img

  colmap::OptionManager col_options;                          // classic colmap options and cmd arg parser
  fuhe::rr::RerunVisualizationOptions rr_options;             // rerun visualization options
  tcf::FusionGraphBundleAdjustmentOptions fusion_ba_options;  // options (e.g. tum path) for FusionGraphBundleAdjuster
  fuhe::align::AlignmentOptions alignment_options;            // colmap reconstruction coordinate alingment options

  // classic colmap options
  col_options.AddRequiredOption("db_path", &db_path);
  col_options.AddRequiredOption("output_path", &output_path);
  // custom rerun option
  col_options.AddDefaultOption("Rerun.log", &rr_options.is_log_to_rerun);  
  col_options.AddDefaultOption("Rerun.save_rrd", &rr_options.is_save_rerun_to_disk);
  col_options.AddDefaultOption("Rerun.rrd_path", &rr_options.recording_path);
  col_options.AddDefaultOption("Rerun.odom_as_pred", &rr_options.draw_rerun_odom_as_predicted_poses);
  col_options.AddDefaultOption("Rerun.img_plane_dist", &rr_options.img_plane_dist);
  // custom init optiosn
  col_options.AddDefaultOption("Init.n_init_pair_skip", &n_init_pair_skip);
  // custom ba options
  col_options.AddDefaultOption("time_diff_local_ba",                       //! FIXME: change to Model. or Mapping.
                               &fusion_ba_options.time_between_local_ba);  // seconds to pass to allow new round of local BA
  // custom frame alignment options
  col_options.AddDefaultOption("FrameAlign.n_reg_for_alignment", &alignment_options.n_reg_for_alignment);
  col_options.AddDefaultOption("FrameAlign.pca_align", &alignment_options.pca_align);

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
  std::shared_ptr<colmap::DatabaseCache> db_cache = colmap::DatabaseCache::Create(db, 0, false, /*image_names=*/{});
  tcf::IncrementalMapperRerun mapper(db_cache);  // vanilla mapper object with rerun capabilities
  // create empty reconstruction
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  VLOG(1) << "Begin reconstruction!";
  mapper.BeginReconstruction(reconstruction);

  // -------------------- Init rerun if visualization is toggled
  std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_logger = nullptr;
  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    VLOG(1) << "Rerun recording toggled. Attaching Recorder manager to mapper!";
    rr_logger = std::make_shared<fuhe::rr::RerunSfmLogger>(rr_options, mapper.Reconstruction());
    mapper.AttachRerunSfmLogger(rr_logger);
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

  // force lower initial pair requirements
  colmap::IncrementalMapper::Options mapper_opts = incr_pipieline_opts->Mapper();
  mapper_opts.init_min_num_inliers = 50;
  mapper_opts.init_min_tri_angle = 0.013;     // degrees
  mapper_opts.init_max_forward_motion = 1.0;  // essential matrix z motion

  // -------------------- Force Select intial image pair
  colmap::TwoViewGeometry tvg;  // Essential matrix and (filtered matches) between initial image pair
  auto img_id_iter = img_ids_sorted.begin();
  colmap::image_t id_1 = img_id_iter->second;  // very first image in tajectory sequence
  // skip a couple of imgs as second init img if specified
  for (int i = 0; i < n_init_pair_skip; i++) {
    img_id_iter++;
  }
  colmap::image_t id_2 = img_id_iter->second;  // second image to take for init triangulation

  VLOG(2) << "Trying to force 1st and 2nd image as initial pair of new reconstruction!";
  VLOG(2) << "Ids : " << id_1 << " and " << id_2;

  int init_attempts = 0;
  // try to force first and second image (or n-th if skipped) of whole sequence as initial pair
  while (!mapper.EstimateInitialTwoViewGeometry(mapper_opts, tvg, id_1, id_2)) {
    fuhe::col_utils::PrintTwoViewStatistics(tvg);
    init_attempts++;
    img_id_iter++;
    id_2 = img_id_iter->second;
    VLOG(2) << "Attempt failed. Increasing id for 2nd image and trying again!";
    VLOG(2) << "Ids : " << id_1 << " and " << id_2;

    if (init_attempts > reconstruction->NumImages() / 2) {
      LOG(FATAL) << "Could not find initial image pair for reconstruction!";
      return 1;
    }
  }

  // -------------------- Force register selected intial image pair
  VLOG(1) << "Initial Pair found with ids: " << id_1 << " and " << id_2;
  fuhe::col_utils::PrintTwoViewStatistics(tvg);
  mapper.RegisterInitialImagePair(mapper_opts, tvg, id_1, id_2);  // lock in

  // -------------------- Initial Pair Rerun visualization
  if (rr_options.is_log_to_rerun) {
    // log initial pair to rerun
    rr_logger->LogFullReconstruction();
  }
  // -------------------- One round of global bundle adjustment for the inital pair
  VLOG(1) << "Kick off a round of global bundle adjustment for initial par!";
  mapper.AdjustGlobalBundle(mapper_opts, incr_pipieline_opts->GlobalBundleAdjustment());

  mapper.Reconstruction()->Normalize();
  // for first global ba use separate set of 3d point filtering options due to low triangulation angle in init
  colmap::IncrementalMapper::Options init_filter_mapper_opts = mapper_opts;
  init_filter_mapper_opts.filter_min_tri_angle = init_filter_mapper_opts.init_min_tri_angle * 1.5;
  mapper.FilterPoints(init_filter_mapper_opts);

  // log bundle adjusted initial pair to rerun
  if (rr_logger) {
    rr_logger->LogFullReconstruction();
  }

  // -------------------- Iterate over all time sorted images to register them
  VLOG(1) << "Begin reconstruction process!";
  double prev_stamp = 0;
  size_t ba_prev_num_reg_images = reconstruction->NumRegImages();
  size_t ba_prev_num_points = reconstruction->NumPoints3D();
  bool normalize = true;
  for (const auto& [stamp, img_id] : img_ids_sorted) {
    // -------------------- Continued registration
    // skip inital pair
    if (img_id == id_1 || img_id == id_2) {
      prev_stamp = stamp;  // store stamps from initial pair for time compare
      continue;
    }

    // register image in reconstruction
    VLOG(2) << "Registering image " << img_id;
    if (!mapper.RegisterNextImage(mapper_opts, img_id)) {
      VLOG(1) << "Registration of current image failed! Stopping mapping process!";
      break;
    }

    // triangulate image points from newly registered image
    VLOG(2) << "Triangulating new points for image " << img_id;
    mapper.TriangulateImage(incr_pipieline_opts->Triangulation(), img_id);

    // log newly registered img and triangulated points to rerun
    if (rr_logger) {
      rr_logger->LogFullReconstruction();
    }

    // -------------------- Frame alingment
    // perform colmap model coordinate frame alignment (once) if condition are met
    if (fuhe::align::CheckRunAlignment(reconstruction->NumRegImages(), alignment_options)) {
      // pca alignment, forcing 1st cam pose to a specfici intial value, etc. read frame_align_utils.h for more info
      fuhe::align::PerformAlignmentStrategies(reconstruction, alignment_options);
      // enough of model normalization after each global BA
      normalize = false;
      // text log to rerun
      if (rr_options.is_log_to_rerun) {
        rr_logger->LogInfoMsg("Performed coordinate frame alignment strategies on colmap model!");
      }
    }

    // -------------------- Local and global BAs
    // perform local or global BA only if enough time has passed between current image and last employed BA image
    if (stamp - prev_stamp < fusion_ba_options.time_between_local_ba) {
      VLOG(2) << "Time dff for new local BA not yet reached. Diff to last local BA only " << stamp - prev_stamp
              << ". Skipping local and potential global BA!";
      continue;
    }

    // -------------------- Iterative Local BAs
    VLOG(2) << "More than " << fusion_ba_options.time_between_local_ba
            << " seconds passed between images. Trigger iterative local refinmenent!";

    mapper.IterativeLocalRefinement(incr_pipieline_opts->ba_local_max_refinements,
                                    incr_pipieline_opts->ba_local_max_refinement_change,
                                    mapper_opts,
                                    incr_pipieline_opts->LocalBundleAdjustment(),
                                    incr_pipieline_opts->Triangulation(),
                                    img_id);
    prev_stamp = stamp;

    // -------------------- Iterative global BAs
    if (CheckRunGlobalRefinement(*reconstruction, *incr_pipieline_opts, ba_prev_num_reg_images, ba_prev_num_points) &&
        reconstruction->NumRegImages() > mapper_opts.local_ba_num_images) {
      VLOG(2) << "Enough imgs registered since last global BA. Global bundle adjustments toggled!";
      mapper.IterativeGlobalRefinement(incr_pipieline_opts->ba_global_max_refinements,
                                       incr_pipieline_opts->ba_global_max_refinement_change,
                                       mapper_opts,
                                       incr_pipieline_opts->GlobalBundleAdjustment(),
                                       incr_pipieline_opts->Triangulation(),
                                       /*normalize (if fusion is toggled off)*/ normalize);
      ba_prev_num_points = reconstruction->NumPoints3D();
      ba_prev_num_reg_images = reconstruction->NumRegImages();
    }
  }

  // -------------------- Final global bundle adjustment
  VLOG(1) << "Final round of global bundle adjustment!";
  mapper.IterativeGlobalRefinement(incr_pipieline_opts->ba_global_max_refinements,
                                   incr_pipieline_opts->ba_global_max_refinement_change,
                                   mapper_opts,
                                   incr_pipieline_opts->GlobalBundleAdjustment(),
                                   incr_pipieline_opts->Triangulation(),
                                   /*normalize */ normalize);
  // log final model to rerun
  if (rr_logger) {
    rr_logger->LogFullReconstruction();
  }

  // -------------------- Finalize
  VLOG(1) << "Done registering all images in reconstruction!";
  VLOG(1) << "Writing model to: " << output_path;
  reconstruction->WriteText(output_path);

  mapper.EndReconstruction(/*false*/ false);  // finalize reconstruction

  return EXIT_SUCCESS;
}
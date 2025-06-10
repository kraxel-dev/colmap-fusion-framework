/**
 * @file incremental_mapper_sanity_check.cpp
 * @author kraxel
 * @brief Sanity check for the vanilla IncrementalMapper class (with rerun logging capabilities). Applies manual steps of Mapper (described
 * in: orig colmap repo src/colmap/sfm/incremental_mapper.h) to reconstruct a model from scratch without fusion capabilities. Order of
 * images will be sorted by ascending time. Very first and 2nd image (by time) in database are forced as initial pair for mapping. Camera
 * intrinsics are fixed.
 * @version 0.1
 * @date 2025-03-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "tightly_coupled_fusion/estimators/bundle_adjustment.h"
#include "tightly_coupled_fusion/sfm/incremental_mapper.h"
#include <colmap/controllers/incremental_pipeline.h>
#include <colmap/controllers/option_manager.h>
#include <colmap/scene/database_cache.h>
#include <fusion_helper/col_utils.h>
#include <fusion_helper/io.h>
#include <fusion_helper/rr_fusion_recorder.h>

int main(int argc, char** argv) {
  // -------------------- Parse COLMAP and Ceres inputs
  std::string db_path;  // database path
  std::string output_path;

  colmap::OptionManager col_options;                          // classic colmap options and cmd arg parser
  fuhe::rrfuse::RerunVisualizationOptions rr_options;             // rerun visualization options
  tcf::FusionGraphBundleAdjustmentOptions fusion_ba_options;  // options (e.g. tum path) for FusionGraphBundleAdjuster

  // classic colmap options
  col_options.AddRequiredOption("db_path", &db_path);
  col_options.AddRequiredOption("output_path", &output_path);
  // custom rerun option
  col_options.AddDefaultOption("rerun", &rr_options.is_log_to_rerun);
  col_options.AddDefaultOption("save_rrd", &rr_options.is_save_rerun_to_disk);
  col_options.AddDefaultOption("rerun_odom_as_pred", &rr_options.draw_rerun_odom_as_predicted_poses);
  col_options.AddDefaultOption("rerun_img_plane_dist", &rr_options.img_plane_dist);
  // custom fusion options
  col_options.AddDefaultOption("time_diff_local_ba",
                               &fusion_ba_options.time_between_local_ba);  // seconds to pass to allow new round of local BA

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
  // colmap::IncrementalMapper mapper(db_cache);  // vanilla mapper object
  tcf::IncrementalMapperRerun mapper(db_cache);  //
  // create empty reconstruction
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  VLOG(1) << "Begin reconstruction!";
  mapper.BeginReconstruction(reconstruction);

  // -------------------- Init rerun if visualization is toggled
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_rc = nullptr;
  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    VLOG(1) << "Rerun recording toggled. Attaching Recorder manager to mapper!";
    rr_rc = std::make_shared<fuhe::rrfuse::RerunFusionRecorder>(rr_options, *mapper.Reconstruction());
    mapper.AttachRerunRecorder(rr_rc);
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
  colmap::TwoViewGeometry tvg;                                       // Essential matrix and (filtered matches) between initial image pair
  colmap::image_t id_1 = img_ids_sorted.begin()->second;             // very first image in tajectory sequence
  colmap::image_t id_2 = std::next(img_ids_sorted.begin())->second;  // second image in sequence

  // -------------------- Force register selected intial image pair
  VLOG(2) << "Trying to force 1st and 2nd image as initial pair of new reconstruction!";
  VLOG(2) << "Ids : " << id_1 << " and " << id_2;
  // try to force first and second image of whole sequence as initial pair
  bool init_succes = mapper.EstimateInitialTwoViewGeometry(mapper_opts, tvg, id_1, id_2);
  fuhe::col_utils::PrintTwoViewStatistics(tvg);
  if (!init_succes) {
    LOG(FATAL) << "Could not find initial image pair for reconstruction!";
    return 1;
  }

  VLOG(1) << "Initial Pair found with ids: " << id_1 << " and " << id_2;
  mapper.RegisterInitialImagePair(mapper_opts, tvg, id_1, id_2);  // lock in

  // -------------------- Initial Pair Rerun visualization
  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    rr_rc->UpdateRerunTimeStep();

    // log initial pair to rerun
    fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                    rr_rc->GetRerunPinhole(),
                                    fuhe::col_utils::RegisteredImages(mapper.Reconstruction()),
                                    mapper.Reconstruction()->Points3D());
  }
  // -------------------- One round of global bundle adjustment for the inital pair
  VLOG(1) << "Kick off a round of global bundle adjustment for initial par!";
  mapper.AdjustGlobalBundle(mapper_opts, incr_pipieline_opts->GlobalBundleAdjustment());
  // mapper.FilterPoints(mapper_opts);
  mapper.Reconstruction()->Normalize(/*fixed_scale=*/true);

  if (rr_rc) {
    // log bundle adjusted initial pair to rerun
    rr_rc->UpdateRerunTimeStep();
    fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                    rr_rc->GetRerunPinhole(),
                                    fuhe::col_utils::RegisteredImages(mapper.Reconstruction()),
                                    mapper.Reconstruction()->Points3D());
  }

  // -------------------- Iterate over all time sorted images to register them
  VLOG(1) << "Begin reconstruction process!";
  double prev_stamp = 0;
  for (const auto& [stamp, img_id] : img_ids_sorted) {
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

    // log newly registered img to rerun
    if (rr_rc) {
      rr_rc->UpdateRerunTimeStep();
      fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                      rr_rc->GetRerunPinhole(),
                                      fuhe::col_utils::RegisteredImages(mapper.Reconstruction()),
                                      mapper.Reconstruction()->Points3D());
    }

    if (stamp - prev_stamp > fusion_ba_options.time_between_local_ba) {
      VLOG(2) << "More than " << fusion_ba_options.time_between_local_ba
              << " seconds passed between images. Trigger iterative local refinmenent!";

      mapper.IterativeLocalRefinement(incr_pipieline_opts->ba_local_max_refinements,
                                      incr_pipieline_opts->ba_local_max_refinement_change,
                                      mapper_opts,
                                      incr_pipieline_opts->LocalBundleAdjustment(),
                                      incr_pipieline_opts->Triangulation(),
                                      img_id);
      prev_stamp = stamp;
    }
  }

  // -------------------- Final global bundle adjustment
  VLOG(1) << "Final round of global bundle adjustment!";
  mapper.IterativeGlobalRefinement(incr_pipieline_opts->ba_global_max_refinements,
                                   incr_pipieline_opts->ba_global_max_refinement_change,
                                   mapper_opts,
                                   incr_pipieline_opts->GlobalBundleAdjustment(),
                                   incr_pipieline_opts->Triangulation(),
                                   /*normalize */ false);
  // log final model to rerun
  if (rr_rc) {
    rr_rc->UpdateRerunTimeStep();
    fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                    rr_rc->GetRerunPinhole(),
                                    fuhe::col_utils::RegisteredImages(mapper.Reconstruction()),
                                    mapper.Reconstruction()->Points3D());
  }

  // -------------------- Finalize
  VLOG(1) << "Done registering all images in reconstruction!";
  VLOG(1) << "Writing model to: " << output_path;
  reconstruction->WriteText(output_path);

  mapper.EndReconstruction(/*false*/ false);  // finalize reconstruction

  return EXIT_SUCCESS;
}
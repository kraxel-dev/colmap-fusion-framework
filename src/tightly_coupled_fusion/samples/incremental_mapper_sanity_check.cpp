/**
 * @file incremental_mapper_sanity_check.cpp
 * @author kraxel
 * @brief Sanity check for the IncrementalFusionMapper class. Applies manual steps of Mapper (described in: orig colmap repo
 * src/colmap/sfm/incremental_mapper.h) to reconstruct a model from scratch with custom odometry fusion capabilities from this repo. Order
 * of images will be ascending time sorted.
 * @version 0.1
 * @date 2025-03-24
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
  VLOG(3) << "Logging path is: " << log_dir;

  //   google::InitGoogleLogging(argv[0]);

  // -------------------- Read database cache and init fusion Mapper object
  colmap::Database db = colmap::Database(db_path);
  std::shared_ptr<colmap::DatabaseCache> db_cache = colmap::DatabaseCache::Create(db, 0, false, {});
  //   tcf::IncrementalFusionMapper mapper(db_cache);  // fusion mapper object
  colmap::IncrementalMapper mapper(db_cache);  // vanilla mapper object

  // -------------------- Define order of images for active reconstructing
  // obtain image ids in database sorted by ascending time
  fuhe::types::MapOfImageIdsSec img_ids_sorted = fuhe::col_utils::ImageIdsByStamp(db_cache->Images());

  // -------------------- Find intial image pair
  // create empty reconstruction
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  VLOG(1) << "Begin reconstruction!";
  mapper.BeginReconstruction(reconstruction);

  colmap::TwoViewGeometry tvg;                            // Essential matrix and (filtered matches) between initial image pair
  colmap::image_t id_1 = img_ids_sorted.begin()->second;  // very first image in tajectory sequence
  //   colmap::image_t id_2 = std::next(img_ids_sorted.begin())->second;  // second image in sequence
  colmap::image_t id_2 = -1;  // second image in sequence
  // find second image of sequence stored in db
  for (auto [_, id] : img_ids_sorted) {
    if (id == id_1) {
      continue;
    }

    if (db_cache->ExistsImage(id)) {
      VLOG(1) << "Found 2nd image id in sequence with id: " << id;
      id_2 = id;
      break;
    }
  }

  // force down initial pair requirements
  auto incr_pipieline_opts = col_options.mapper;
  colmap::IncrementalMapper::Options mapper_opts = incr_pipieline_opts->Mapper();
  mapper_opts.init_min_num_inliers = 50;
  mapper_opts.init_min_tri_angle = 0.013;     // degrees
  mapper_opts.init_max_forward_motion = 1.0;  // essential matrix z motion

  VLOG(2) << "Trying to force 1st and 2nd image as initial pair of new reconstruction!";
  VLOG(2) << "Ids : " << id_1 << " and " << id_2;
  // try to force first and second image of whole sequence as initial pair
  bool init_succes = mapper.EstimateInitialTwoViewGeometry(mapper_opts, tvg, id_1, id_2);
  VLOG(1) << "Inlier matches: " << tvg.inlier_matches.size();
  VLOG(1) << "z forward motion: " << tvg.cam2_from_cam1.translation.z();
  VLOG(1) << "Tri angle: " << tvg.tri_angle;

  if (!init_succes) {
    LOG(FATAL) << "Could not find initial image pair for reconstruction!";
    return 1;
  }
  VLOG(1) << "Initial Pair found with ids: " << id_1 << " and " << id_2;

  mapper.RegisterInitialImagePair(mapper_opts, tvg, id_1, id_2);  // lock in

  // -------------------- Iterate over all time sorted images to register them
  VLOG(2) << "Begin iterating over all images to register them in reconstruction!";
  double prev_stamp = 0;
  for (const auto& [stamp, img_id] : img_ids_sorted) {
    if (img_id == id_1 || img_id == id_2) {
      prev_stamp = stamp;  // store stamps from initial pair
      continue;            // skip intial pair
    }

    // register image in reconstruction
    VLOG(3) << "Registering image " << img_id;
    if (!mapper.RegisterNextImage(mapper_opts, img_id)) {
      VLOG(1) << "Registration of current image failed! Stopping mapping process!";
      break;
    }

    double secs_to_wait = 0.75;
    if (stamp - prev_stamp > secs_to_wait) {
      VLOG(2) << "More than " << secs_to_wait << " seconds passed between images. Trigger iterative local refinmenent!";
      prev_stamp = stamp;

      int max_num_refinements = 5;
      double max_refinement_change = incr_pipieline_opts->ba_local_max_refinement_change;
      mapper.IterativeLocalRefinement(max_num_refinements,
                                      max_refinement_change,
                                      mapper_opts,
                                      incr_pipieline_opts->LocalBundleAdjustment(),
                                      incr_pipieline_opts->Triangulation(),
                                      img_id);
    }
  }

  // -------------------- Final global bundle adjustment
  mapper.IterativeGlobalRefinement(incr_pipieline_opts->ba_global_max_refinements,
                                   incr_pipieline_opts->ba_global_max_refinement_change,
                                   mapper_opts,
                                   incr_pipieline_opts->GlobalBundleAdjustment(),
                                   incr_pipieline_opts->Triangulation());

  VLOG(1) << "Done registering all images in reconstruction!";
  VLOG(1) << "Writing model to: " << output_path;
  reconstruction->WriteText(output_path);

  // -------------------- Clean up model
  mapper.EndReconstruction(/*false*/ false);  // finalize reconstruction
  mapper.Reconstruction()->TearDown();

  return EXIT_SUCCESS;
}
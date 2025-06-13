/**
 * @file fusion_graph_ba_sanity_check.cpp
 * @author kraxel
 * @brief Saniy check for the FusionGraphBundleAdjuster class. Does the same as high_level_fusions "metric_odom_bundle_adjust"
 * (reproj + odom error optimization for the whole reconstruction).
 * @version 0.1
 * @date 2025-03-12
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "fusion_helper/col_utils.h"
#include "tightly_coupled_fusion/estimators/bundle_adjustment.h"
#include <colmap/controllers/option_manager.h>
#include <fusion_helper/io.h>
#include <fusion_helper/rr_sfm_logger.h>

int main(int argc, char** argv) {
  // -------------------- Parse COLMAP and Ceres inputs
  std::string input_path;
  std::string output_path;

  colmap::OptionManager col_options;                          // classic colmap options and cmd arg parser
  fuhe::rr::RerunVisualizationOptions rr_options;             // rerun visualization options
  tcf::FusionGraphBundleAdjustmentOptions fusion_ba_options;  // options (e.g. tum path) for FusionGraphBundleAdjuster

  // classic colmap options
  col_options.AddRequiredOption("input_path", &input_path);
  col_options.AddRequiredOption("output_path", &output_path);
  // custom rerun option
  col_options.AddDefaultOption("rerun", &rr_options.is_log_to_rerun);  // FIXME: change to flage to Rerun.log
  col_options.AddDefaultOption("save_rrd", &rr_options.is_save_rerun_to_disk);
  col_options.AddDefaultOption("rerun_odom_as_pred", &rr_options.draw_rerun_odom_as_predicted_poses);
  col_options.AddDefaultOption("rerun_img_plane_dist", &rr_options.img_plane_dist);
  // deactivate bb around active camposes in rerun since all images are active in this sanity check
  rr_options.is_highlight_active_cams = false;
  // classic colmap BA solver options
  col_options.AddBundleAdjustmentOptions();
  // actually parse command line args into option members
  col_options.Parse(argc, argv);

  // -------------------- Logging stuff
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;

  // Set log directory
  const std::string log_dir = fuhe::io::GetRepoRootDir() + "/logs";
  FLAGS_log_dir = log_dir;
  VLOG(3) << "Logging path is: " << log_dir;

  google::InitGoogleLogging(argv[0]);

  // -------------------- Read colmap model
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(input_path);
  // TODO: decide for better generic strategy on kicking bad points
  fuhe::col_utils::CropBBoxOutlierPoints(reconstruction);

  auto imgs_by_stamp = fuhe::col_utils::ImageIdsByStamp(reconstruction->Images());  // image-ids in time sorted order

  // -------------------- Prepare fusion graph data edges between images
  fuhe::types::MapOfPosesSec metric_poses;  // absolute poses from external odom sensor sorted by stamps
  fuhe::io::TumToPosesEigen(fusion_ba_options.tum_file, metric_poses, true);

  // data structure holding image sequence with odom edges to iterate over
  std::shared_ptr<fuhe::edges::MapOfImageEdges> fusion_graph_data_edges =
      fuhe::edges::CreateSequentialImageEdgesPtr(imgs_by_stamp, metric_poses);

  // -------------------- Tune BA config to decide which img to consider and/or  are constant in problem
  VLOG(1) << "Selecting colmap images for ceres optimization!";
  colmap::BundleAdjustmentConfig ba_cfg;  // cfg deciding which images to considere for ceres optim

  // FIXME: here, all reg images are considered for optim while internally the graph drops images that preceed odometry edges
  int i = 0;
  for (const auto& [stamp, img_id] : imgs_by_stamp) {
    ba_cfg.AddImage(img_id);                  // notify ceres to inlcude this img for optimization
    ba_cfg.SetConstantCamIntrinsics(img_id);  // no intrinsics optimization for now

    // NOTE: no 3d point adding to ba_config required

    if (i == 0) {
      VLOG(2) << "Fixing position of image: " << img_id;
      // fix gauge freedom on first image-pose in model
      ba_cfg.SetConstantCamPose(img_id);
    }

    i++;
  }

  // -------------------- Init rerun if visualization is toggled
  std::shared_ptr<fuhe::rr::RerunSfmLogger> rr_logger = nullptr;
  if (rr_options.is_log_to_rerun) {
    // initialize rerun sfm logger object when rerun logging is desired
    VLOG(1) << "Rerun logging toggled. Attaching Sfm Logger to mapper!";
    rr_logger = std::make_shared<fuhe::rr::RerunSfmLogger>(rr_options, reconstruction);
  }

  // -------------------- Create FusionGraphBundleAdjuster object
  VLOG(1) << "Creating fusion bundle adjuster object!";
  // bundle adjuster with odometry fusion capabilities and automatic rerun loggung during iters
  std::unique_ptr<colmap::BundleAdjuster> fusion_bundle_adjuster =
      tcf::CreateFusionGraphBundleAdjuster(*col_options.bundle_adjustment,
                                           fusion_ba_options,
                                           ba_cfg,
                                           *reconstruction.get(),
                                           *fusion_graph_data_edges.get(),
                                           rr_logger);

  // -------------------- Solve fusion problem
  VLOG(1) << "Starting to solve bundle adjustment problem!";
  const ceres::Solver::Summary summary = fusion_bundle_adjuster->Solve();

  VLOG(1) << summary.FullReport();
  VLOG(1) << "Done solving odometry fusion BA!";

  reconstruction->WriteText(output_path);

  return 0;
}
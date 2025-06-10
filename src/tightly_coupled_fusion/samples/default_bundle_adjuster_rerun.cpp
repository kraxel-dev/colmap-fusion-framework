/**
 * @file default_bundle_adjuster_rerun.cpp
 * @author kraxel
 * @brief Small sample of how to use the default colmap Bundle Adjuster with extra rerun visualization of the full BA on a fully
 * reconstructed model. Also acts as showcase how to populate the ba_config, which will dictate which images are considered in
 * the BA process by ceres.
 * @version 0.1
 * @date 2025-05-26
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "tightly_coupled_fusion/estimators/bundle_adjustment.h"
#include <colmap/controllers/option_manager.h>
#include <colmap/estimators/coordinate_frame.h>
#include <fusion_helper/col_utils.h>
#include <fusion_helper/io.h>
#include <fusion_helper/rr_fusion_recorder.h>

int main(int argc, char** argv) {
  // -------------------- Parse COLMAP and Ceres inputs
  std::string input_path;
  std::string output_path;
  // whether to crop model (from bogus points) before optimization (e.g. remove bogus points that mess up rerun viz)
  bool pre_crop_points = true;
  // whether to align model with PCA before optimization (e.g. for better visualization)
  bool pca_align = true;

  colmap::OptionManager col_options;               // classic colmap options and cmd arg parser
  fuhe::rrfuse::RerunVisualizationOptions rr_options;  // rerun visualization options

  // classic colmap options
  col_options.AddRequiredOption("input_path", &input_path);
  col_options.AddRequiredOption("output_path", &output_path);
  col_options.AddDefaultOption("Model.pre_crop_points", &pre_crop_points);
  col_options.AddDefaultOption("Model.pca_align", &pca_align);
  // custom rerun options
  col_options.AddDefaultOption("rerun", &rr_options.is_log_to_rerun);
  col_options.AddDefaultOption("save_rrd", &rr_options.is_save_rerun_to_disk);

  // classic colmap BA solver options
  col_options.AddBundleAdjustmentOptions();
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

  // -------------------- Read reconstruction
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(input_path);
  // obtain image ids in database sorted by ascending time
  fuhe::types::MapOfImageIdsSec img_ids_sorted = fuhe::col_utils::ImageIdsByStamp(reconstruction->Images());

  // -------------------- Init rerun if visualization is toggled
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_rc = nullptr;
  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    VLOG(1) << "Rerun recording toggled. Initializing recorder object!";
    rr_rc = std::make_shared<fuhe::rrfuse::RerunFusionRecorder>(rr_options, *reconstruction.get());

    // write initial reconstruction to rerun
    rr_rc->UpdateRerunTimeStep();
    fuhe::rrfuse::LogReconstruction(
        rr_rc->GetRerunRec(), rr_rc->GetRerunPinhole(), reconstruction->Images(), reconstruction->Points3D());
  }

  // -------------------- Model preprocessing

  // align model with PCA if toggled
  if (pca_align) {
    VLOG(1) << "Aligning model with PCA before optimization!";
    colmap::Sim3d align;
    colmap::AlignToPrincipalPlane(reconstruction.get(), &align);
  } else {
    VLOG(1) << "Skipping model PCA alignment before optimization!";
  }
  
  // crop model to remove bad points that mess up rerun viz
  if (pre_crop_points) {
    VLOG(1) << "Cropping model pts before optimization!";
    fuhe::col_utils::CropBBoxOutlierPoints(reconstruction);
  } else {
    VLOG(1) << "Skipping model pts cropping before optimization!";
  }

  // write updated reconstruction to rerun
  if (rr_options.is_log_to_rerun && (pre_crop_points || pca_align)) {
    rr_rc->UpdateRerunTimeStep();
    fuhe::rrfuse::LogReconstruction(
        rr_rc->GetRerunRec(), rr_rc->GetRerunPinhole(), reconstruction->Images(), reconstruction->Points3D());
  }

  // -------------------- Tune BA config to decide which img to consider and/or are constant in problem
  VLOG(1) << "Selecting colmap images for ceres optimization!";
  colmap::BundleAdjustmentConfig ba_cfg;  // cfg deciding which images to considere for ceres optim

  int i = 0;
  for (const auto& [stamp, img_id] : img_ids_sorted) {
    ba_cfg.AddImage(img_id);                  // notify ceres to inlcude this img for optimization
    ba_cfg.SetConstantCamIntrinsics(img_id);  // no intrinsics optimization for now

    // NOTE: no 3d point adding to ba_config required. Points are selected automatically by colmap based on the images selected
    // in the default bundle adjuster.

    // FIXME: analyze weird behavior with fixed pose in more detail
    if (i == 0) {
      VLOG(2) << "Fixing position of image: " << img_id;
      // fix gauge freedom on first image-pose in model
      ba_cfg.SetConstantCamPose(img_id);
    }

    i++;
  }

  // -------------------- Create FusionGraphBundleAdjuster object
  VLOG(1) << "Creating Rerun bundle adjuster object!";

  // vanilla bundle adjuster with rerun visualization
  std::unique_ptr<colmap::BundleAdjuster> default_ba =
      tcf::CreateDefaultBundleAdjusterRerun(*col_options.bundle_adjustment, ba_cfg, *reconstruction.get(), rr_rc);

  // -------------------- Solve fusion problem
  VLOG(1) << "Starting to solve bundle adjustment problem!";
  const ceres::Solver::Summary summary = default_ba->Solve();

  VLOG(1) << summary.FullReport();
  VLOG(1) << "Done solving default Bunde Adjustment!";

  reconstruction->WriteText(output_path);

  return 0;
}
#include "tightly_coupled_fusion/estimators/bundle_adjustment.h"
#include <colmap/controllers/option_manager.h>
#include <fusion_helper/col_utils.h>
#include <fusion_helper/io.h>
#include <fusion_helper/rr_fusion_recorder.h>

int main(int argc, char** argv) {
  // -------------------- Parse COLMAP and Ceres inputs
  std::string input_path;
  std::string output_path;

  colmap::OptionManager col_options;                          // classic colmap options and cmd arg parser
  fuhe::rrfuse::RerunFusionVisOptions rr_options;             // rerun visualization options
  tcf::FusionGraphBundleAdjustmentOptions fusion_ba_options;  // options (e.g. tum path) for FusionGraphBundleAdjuster

  // classic colmap options
  col_options.AddRequiredOption("input_path", &input_path);
  col_options.AddRequiredOption("output_path", &output_path);
  // custom rerun option
  col_options.AddDefaultOption("rerun", &rr_options.is_log_to_rerun);
  col_options.AddDefaultOption("save_rrd", &rr_options.is_save_rerun_to_disk);
  col_options.AddDefaultOption("rerun_odom_as_pred", &rr_options.draw_rerun_odom_as_predicted_poses);
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
  VLOG(3) << "Logging path is: " << log_dir;

  // -------------------- Read reconstruction
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(input_path);
  // obtain image ids in database sorted by ascending time
  fuhe::types::MapOfImageIdsSec img_ids_sorted = fuhe::col_utils::ImageIdsByStamp(reconstruction->Images());
  // TODO: decide for better generic strategy on kicking bad points
//   fuhe::col_utils::CropFarAwayPoints(reconstruction);

  // -------------------- Init rerun if visualization is toggled
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_rc = nullptr;
  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    VLOG(1) << "Rerun recording toggled. Attaching Recorder manager to mapper!";
    rr_rc = std::make_shared<fuhe::rrfuse::RerunFusionRecorder>(rr_options, *reconstruction.get());
  }

  // -------------------- Tune BA config to decide which img to consider and/or  are constant in problem
  VLOG(1) << "Selecting colmap images for ceres optimization!";
  colmap::BundleAdjustmentConfig ba_cfg;  // cfg deciding which images to considere for ceres optim

  // FIXME: here, all reg images are considered for optim while internally the graph drops images that preceed odometry edges
  int i = 0;
  for (const auto& [stamp, img_id] : img_ids_sorted) {
    ba_cfg.AddImage(img_id);                  // notify ceres to inlcude this img for optimization
    ba_cfg.SetConstantCamIntrinsics(img_id);  // no intrinsics optimization for now

    // NOTE: no 3d point adding to ba_config required

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

  // bundle adjuster with odometry fusion capabilities
  std::unique_ptr<colmap::BundleAdjuster> default_ba =
      tcf::CreateDefaultBundleAdjusterRerun(*col_options.bundle_adjustment, ba_cfg, *reconstruction.get(), rr_rc);

  // -------------------- Solve fusion problem
  VLOG(1) << "Starting to solve bundle adjustment problem!";
  const ceres::Solver::Summary summary = default_ba->Solve();

  VLOG(1) << summary.FullReport();
  VLOG(1) << "Done solving odometry fusion BA!";

  reconstruction->WriteText(output_path);

  return 0;
}
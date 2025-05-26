/**
 * @file model_align.cpp
 * @author kraxel
 * @brief sample/helper to apply the colmap reconstruction alignment strategies to a fully reconstructed model, implemented in
 * the fusion helper module. Mainly use this to pca align a reconstruciton or force the first pose of model to assume a provided
 * pose from config (e.g. extrinsic camera position). Additional rerun visualization can help you eyball your extrinsics in case
 * they are not 100% error free.
 * @version 0.1
 * @date 2025-05-26
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <colmap/controllers/option_manager.h>
#include <colmap/util/file.h>
#include <fusion_helper/col_utils.h>
#include <fusion_helper/frame_align_utils.h>
#include <fusion_helper/io.h>
#include <fusion_helper/rr_fusion_logging.h>
#include <fusion_helper/rr_fusion_recorder.h>

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////////////////////
  // Parse COLMAP and ceres options and inputs
  ////////////////////////////////////////////////////////////////////////////////
  std::string output_path;
  std::string input_path;

  colmap::OptionManager col_options;                // classic colmap options and cmd arg parser
  fuhe::rrfuse::RerunFusionVisOptions rr_options;   // rerun visualization options
  fuhe::align::AlignmentOptions alignment_options;  // colmap reconstruction coordinate alingment options

  // classic colmap options
  col_options.AddRequiredOption("input_path", &input_path);
  col_options.AddRequiredOption("output_path", &output_path);
  // custom rerun option
  col_options.AddDefaultOption("rerun", &rr_options.is_log_to_rerun);
  col_options.AddDefaultOption("save_rrd", &rr_options.is_save_rerun_to_disk);
  // custom fusion options

  // custom frame alignment options
  col_options.AddDefaultOption("FrameAlign.pca_align", &alignment_options.pca_align);
  col_options.AddDefaultOption("FrameAlign.align_first_cam_to_specific_pose",
                               &alignment_options.align_first_cam_to_specific_pose);
  col_options.AddDefaultOption("FrameAlign.roll", &alignment_options.specified_roll);
  col_options.AddDefaultOption("FrameAlign.pitch", &alignment_options.specified_pitch);
  col_options.AddDefaultOption("FrameAlign.yaw", &alignment_options.specified_yaw);
  col_options.AddDefaultOption("FrameAlign.rotate_init_motion_onto_global_x_axis",
                               &alignment_options.rotate_init_motion_onto_global_x_axis);

  // actually parse command line args into option members
  col_options.Parse(argc, argv);

  // -------------------- Logging stuff
  colmap::InitializeGlog(argv);
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;

  // set log directory
  const std::string log_dir = fuhe::io::GetRepoRootDir() + "/logs";
  FLAGS_log_dir = log_dir;
  VLOG(2) << "Logging path is: " << log_dir;

  // -------------------- check directoreis
  if (!colmap::ExistsDir(input_path)) {
    LOG(ERROR) << "`input_path` is not a directory";
    return EXIT_FAILURE;
  }

  if (!colmap::ExistsDir(output_path)) {
    LOG(ERROR) << "`output_path` is not a directory";
    return EXIT_FAILURE;
  }

  // -------------------- Read reconstruction
  std::shared_ptr<colmap::Reconstruction> reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(input_path);

  // -------------------- Init rerun if visualization is toggled
  std::shared_ptr<fuhe::rrfuse::RerunFusionRecorder> rr_rc = nullptr;
  if (rr_options.is_log_to_rerun) {
    // initialize recorder objects when rerun logging is desired
    VLOG(2) << "Rerun recording toggled.";
    rr_rc = std::make_shared<fuhe::rrfuse::RerunFusionRecorder>(rr_options, *reconstruction.get());

    // log colmap model to rerun
    rr_rc->UpdateRerunTimeStep();
    fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                    rr_rc->GetRerunPinhole(),
                                    fuhe::col_utils::RegisteredImages(reconstruction),
                                    reconstruction->Points3D());
  }

  // -------------------- Model alignment
  // perform colmap model coordinate frame alignment (once) if condition are met
  fuhe::align::PerformAlignmentStrategies(reconstruction, alignment_options);

  // log aligned model to rerun
  if (rr_options.is_log_to_rerun) {
    // log colmap model to rerun
    rr_rc->UpdateRerunTimeStep();
    fuhe::rrfuse::LogReconstruction(rr_rc->GetRerunRec(),
                                    rr_rc->GetRerunPinhole(),
                                    fuhe::col_utils::RegisteredImages(reconstruction),
                                    reconstruction->Points3D());
  }

  // -------------------- Write newly aligned model and tum file
  fuhe::col_utils::ToTum(reconstruction.get(), output_path);
  VLOG(1) << "Writing model to: " << output_path;
  reconstruction->WriteText(output_path);

  return EXIT_SUCCESS;
}
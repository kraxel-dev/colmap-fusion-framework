/**
 * @file sparsify_model.cpp
 * @author kraxel
 * @brief For a given fully reconstructed colmap model, keep only every n-th image and kick the rest.
 * @version 0.1
 * @date 2025-02-19
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <ostream>

#include "fusion_helper/io.h"
#include <colmap/controllers/option_manager.h>
#include <colmap/scene/reconstruction.h>
#include <colmap/util/file.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  std::string input_path;
  std::string output_path;
  int n_th = 1;  // every n-th image to keep in model

  colmap::OptionManager col_options;

  col_options.AddRequiredOption("input_path", &input_path);
  col_options.AddRequiredOption("output_path", &output_path);
  col_options.AddRequiredOption("keep_n_th", &n_th);

  col_options.Parse(argc, argv);

  // -------------------- Logging stuff
  FLAGS_logtostderr = 0;
  FLAGS_alsologtostderr = 1;

  // Set log directory
  const std::string log_dir = fuhe::io::GetRepoRootDir() + "/logs";
  FLAGS_log_dir = log_dir;
  VLOG(3) << "Logging path is: " << log_dir;

  google::InitGoogleLogging(argv[0]);

  // -------------------- check directoreis
  if (!colmap::ExistsDir(input_path)) {
    LOG(ERROR) << "`input_path` is not a directory";
    return EXIT_FAILURE;
  }

  if (!colmap::ExistsDir(output_path)) {
    LOG(ERROR) << "`output_path` is not a directory";
    return EXIT_FAILURE;
  }

  // -------------------- Read COLMAP model
  auto reconstruction = std::make_shared<colmap::Reconstruction>();
  reconstruction->Read(input_path);

  // -------------------- Kick out unwanted images from model
  std::vector<colmap::image_t> ids_to_kick;
  int i = 0;

  // NOTE: do not delete while iterating
  // iterate over all model images
  for (const auto [id, _] : reconstruction->Images()) {
    // skip over every non n_th image
    if (i % n_th != 0) {
      LOG(INFO) << "Storing image with id: " << id << " for kicking!";
      ids_to_kick.push_back(id);
    }
    i++;
  }

  // iterate over all ids to kick
  for (const auto id : ids_to_kick) {
    // kick image from model
    reconstruction->DeRegisterImage(id);
    LOG(INFO) << "Unregistered image with id: " << id << " from model!";
  }

  LOG(INFO) << "Saving model to: " << output_path;
  reconstruction->WriteText(output_path);

  reconstruction->TearDown();
}
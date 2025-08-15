/**
 * @file create_and_validate_odom_edges.cpp
 * @author kraxel
 * @brief Quick sanity check whether valid 6DoF odometry edges betwenn imgs are created from your own .tum files and colmap
 * DBs. Created sequential img edges should match the number of imgs in your DB. Odometry edges should be atleast above 1
 * (if not your timestamps are most prob messed up). Can be referenced as brief usage example on odom edges creation (including
 * Covariance values). Roughly same apporach is used internally by tightly-coupled fusion mapping.
 * @version 0.1
 * @date 2025-08-15
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <ostream>

#include "fusion_helper/col_utils.h"
#include "fusion_helper/io.h"
#include "fusion_helper/odom_edges.h"
#include "fusion_helper/stream_utils.h"
#include <colmap/controllers/option_manager.h>
#include <colmap/scene/database_cache.h>
#include <colmap/util/file.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////////////////////
  // Parse COLMAP and ceres options and inputs
  ////////////////////////////////////////////////////////////////////////////////
  std::string db_path;        // database path
  std::string odom_tum_file;  // path to tum file containing absolute poses from external odometry source

  colmap::OptionManager col_options;            // classic colmap options and cmd arg parser
  fuhe::cov_utils::OdomCovOptions cov_options;  // covariance options for relative odometry measurements

  // classic colmap options
  col_options.AddRequiredOption("db_path", &db_path);
  col_options.AddRequiredOption("odom_tum_file", &odom_tum_file);

  // Odom cov options
  col_options.AddDefaultOption("OdomCov.tx_std", &cov_options.std_tx_per_s);
  col_options.AddDefaultOption("OdomCov.ty_std", &cov_options.std_ty_per_s);
  col_options.AddDefaultOption("OdomCov.tz_std", &cov_options.std_tz_per_s);
  col_options.AddDefaultOption("OdomCov.rx_std", &cov_options.std_rx_per_s);
  col_options.AddDefaultOption("OdomCov.ry_std", &cov_options.std_ry_per_s);
  col_options.AddDefaultOption("OdomCov.rz_std", &cov_options.std_rz_per_s);

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

  // -------------------- Read database cache
  colmap::Database db = colmap::Database(db_path);
  std::shared_ptr<colmap::DatabaseCache> db_cache = colmap::DatabaseCache::Create(db, 0, false, {});

  // obtain all image ids from COLMAP database, sorted by ascending-time
  fuhe::types::MapOfImageIdsSec imgs_by_stamp = fuhe::col_utils::ImageIdsByStamp(db_cache->Images());

  // -------------------- Read TUM file
  fuhe::types::MapOfPosesSec odom_poses;  // absolute poses from external odom sensor sorted by stamps
  fuhe::io::TumToPosesEigen(odom_tum_file, odom_poses, /*cut_precision=*/true);

  // -------------------- Create directed odom edges between images in sorted order
  // covariance manager for relative odometry measurements
  std::shared_ptr<fuhe::cov_utils::TimeScaledOdomCovManager> cov_manager = std::make_shared<fuhe::cov_utils::TimeScaledOdomCovManager>(cov_options);

  // Create main data structure that will be iterated over to build a fusion BA problem with ceres. SeqImgEdges are (sorted)
  // edges from img i to consecutive img (in time) j (without holes) for the whole COLMAP model. Most importantly, this
  // associates the absolute odom poses from the tum file to the constraining image pairs as relative edge (which can be used by
  // the interface as relative pose factor).

  fuhe::edges::MapOfImageEdges consec_img_edges =
      fuhe::edges::CreateSequentialImageEdges(imgs_by_stamp, odom_poses, *cov_manager);

  // -------------------- Showcase on how to iterate over the img created edges
  // example usage of sequential image edges
  int n_valid_odom_edges = 0;
  for (std::pair<const double, fuhe::edges::SequentialImageEdge>& pair : consec_img_edges) {
    const double img_stamp = pair.first;
    fuhe::edges::SequentialImageEdge image_edge = pair.second;

    // check if external odom is available for current image
    if (!image_edge.OdomEdge()) {
      continue;
    }

    n_valid_odom_edges++;

    // Do stuff with attached odometry...
  }

  // -------------------- Check if edges were created correctly
  // check sequential image edges
  std::cout << "N created sequential image edges: " << consec_img_edges.size() << std::endl;
  std::cout << "N total imgs in COLMAP Database: " << db_cache->Images().size() << std::endl;
  std::cout << "SeqImgEdges should have the same size as n_imgs_in_db (orgin node is also an edge)!" << std::endl;

  if (consec_img_edges.size() != db_cache->Images().size()) {
    std::cerr << "Error: Sequential image edges do not match number of images in database!" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "N valid odometry edges: " << n_valid_odom_edges << std::endl;
  std::cout << "Ideally, you have as many odom edges as consec image edges (unless you external odom sensor is slower that your "
               "image rate)."
            << std::endl;

  return EXIT_SUCCESS;
}
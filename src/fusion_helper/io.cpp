#include "fusion_helper/io.h"

#include <filesystem>

#include "fusion_helper/tum_benchmark/eigen_support.hpp"
#include <Eigen/src/Geometry/Transform.h>
#include <glog/logging.h>

typedef tum_benchmark::Trajectory EntryFormat;

double fuhe::io::TruncateDouble(const double stamp, int precision) {
  std::stringstream stream;
  stream << std::fixed << std::setprecision(precision) << stamp;
  double truncated_value;
  stream >> truncated_value;
  return truncated_value;
}

void fuhe::io::TumToStamps(const std::string& tum_file, std::vector<double>& timestamps) {
  // -------------------- Parse tum file with reader object
  tum_benchmark::FileReader<EntryFormat> reader(tum_file);
  tum_benchmark::FileReader<EntryFormat> r_dist_copy(tum_file);  // duplicate of reader for getting total number of poses since
                                                                 // reader does not properly work with std::distance

  // -------------------- Allocate output vectors with number of total poses
  // Get total number of poses
  int n_poses = std::distance(r_dist_copy.begin(),
                              r_dist_copy.end());  // NOTE: reader iterator useless after distance operation.
  timestamps.resize(n_poses);

  // -------------------- Iterate over all tum poses
  int i = 0;
  for (TumPoseReader::iterator it = reader.begin(); it != reader.end(); ++it) {
    // convert tum pose to Eigen isometry and place in output vector
    timestamps.at(i) = it->timestamp;
    i++;
  }
}

void fuhe::io::TumToPosesEigen(const std::string& tum_file, std::vector<Eigen::Isometry3d>& out_poses) {
  // -------------------- Parse tum file with reader object
  tum_benchmark::FileReader<EntryFormat> reader(tum_file);
  tum_benchmark::FileReader<EntryFormat> r_dist_copy(
      tum_file);  // duplicate of reader for getting total number of poses since reader does not properly work with std::distance

  // -------------------- Allocate output vectors with number of total poses
  // Get total number of poses
  int nPoses = std::distance(r_dist_copy.begin(), r_dist_copy.end());  // NOTE: reader iterator useless after distance operation.
  out_poses.resize(nPoses);

  // -------------------- Iterate over all tum poses
  int i = 0;
  for (TumPoseReader::iterator it = reader.begin(); it != reader.end(); ++it) {
    // convert tum pose to Eigen isometry and place in output vector
    tum_benchmark::toEigen(*it, out_poses.at(i));
    i++;
  }
}

void fuhe::io::TumToPosesEigen(const std::string& tum_file, types::MapOfPosesSec& out_poses_map, const bool cut_precision) {
  // -------------------- Allocate container for poses and timestamps
  std::vector<Eigen::Isometry3d> poses;
  std::vector<double> stamps;

  // -------------------- Parse from tumfile
  VLOG(2) << "Extracting poses from tum file: " << tum_file;
  io::TumToPosesEigen(tum_file, poses);
  fuhe::io::TumToStamps(tum_file, stamps);

  for (size_t i = 0; i < poses.size(); i++) {
    double stamp = stamps.at(i);
    if (cut_precision) {
      // roundoff digits after decimal to easen time stamp matching for stamps with too many digits to parse
      stamp = TruncateDouble(stamp, fuhe::io::DIGIT_PRECISION);
    }
    out_poses_map[stamp] = poses.at(i);
  }
  VLOG(2) << out_poses_map.size() << " poses from tum extracted!";
}

void fuhe::io::Rigid3dToTum(const std::vector<double>& stamps,
                            std::vector<colmap::Rigid3d>& X,
                            const std::string& tum_file,
                            const bool do_inv) {
  std::fstream out_file(tum_file, std::istream::out);

  // write each optimized pose into tumfile
  VLOG(2) << "Exporting trajectory to : " << tum_file;
  for (int n = 0; n < X.size(); n++) {
    // invert pose if desired (remember that colmap poses are world poses expressed in camera frame)
    const colmap::Rigid3d pose = (do_inv) ? colmap::Inverse(X.at(n)) : X.at(n);
    // write write write
    out_file << std::fixed << std::setprecision(6) << stamps.at(n) << " " << pose.translation.x() << " " << pose.translation.y()
             << " " << pose.translation.z() << " " << pose.rotation.x() << " " << pose.rotation.y() << " " << pose.rotation.z()
             << " " << pose.rotation.w() << '\n';
  }
  out_file.close();
}

std::string fuhe::io::GetRepoRootDir() {
  const std::filesystem::path exe_path = std::filesystem::current_path();  // Get the executable's working directory
  std::filesystem::path repo_root = exe_path;                              // use exe path as startpoint

  // Traverse up until we find a known root marker (e.g., .git directory)
  while (repo_root.has_parent_path()) {
    if (std::filesystem::exists(repo_root / ".git") && std::filesystem::exists(repo_root / "src") &&
        std::filesystem::exists(repo_root / "include")) {  // check for root dir pattern of this repo
      return repo_root;
    }
    repo_root = repo_root.parent_path();
  }

  LOG(WARNING) << "Path to colmap_fusion_framework repo root dir not found. Returning path to currently run exe instead.";
  return exe_path;
}

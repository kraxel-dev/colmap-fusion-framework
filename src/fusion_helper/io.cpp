#include "fusion_helper/io.h"

#include "fusion_helper/tum_benchmark/eigen_support.hpp"
#include <Eigen/src/Geometry/Transform.h>

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
  tum_benchmark::FileReader<EntryFormat> rDistCopy(
      tum_file);  // duplicate of reader for getting total number of poses since reader does not properly work with std::distance

  // -------------------- Allocate output vectors with number of total poses
  // Get total number of poses
  int nPoses = std::distance(rDistCopy.begin(), rDistCopy.end());  // NOTE: reader iterator useless after distance operation.
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
}

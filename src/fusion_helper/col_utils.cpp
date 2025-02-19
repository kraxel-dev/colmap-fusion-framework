#include "fusion_helper/col_utils.h"

#include "fusion_helper/io.h"
#include <colmap/util/file.h>

fuhe::types::MapOfImageIdsSec fuhe::col_utils::ImageIdsByStamp(const std::set<colmap::image_t>& image_ids,
                                                               std::shared_ptr<colmap::Reconstruction> reconstruction) {
  fuhe::types::MapOfImageIdsSec ordered_image_stamps;  // output map -> image ids by timestamps [secs]

  // iterate over all ids to register them into a hashmap that takes care of the sorting automatically
  for (const colmap::image_t image_id : image_ids) {
    const colmap::Image img = reconstruction->Image(image_id);

    // get filename (represents [nsec] time stamp)
    std::string stamp_string, end;
    colmap::SplitFileExtension(img.Name(), &stamp_string, &end);

    // convert string to integer timestamp [nsec]
    uint64_t stamp = std::stoull(stamp_string);

    // convert [nsec] stamp to [secs] and accept loss of precision
    double trunc_stamp_sec = static_cast<double>(stamp / 1e9);
    // deliberately cutt off precision after n digits to make it matchable
    trunc_stamp_sec = fuhe::io::TruncateDouble(trunc_stamp_sec, fuhe::io::DIGIT_PRECISION);

    // use timestamp [secs] as hashmap key
    if (ordered_image_stamps.find(trunc_stamp_sec) != ordered_image_stamps.end()) {
      LOG(WARNING) << "Bad news, duplicate timestamps in colmap model found for image stamps: " << trunc_stamp_sec;
    }
    ordered_image_stamps[trunc_stamp_sec] = image_id;
  }
  return ordered_image_stamps;
}

const std::vector<colmap::Point3D> fuhe::col_utils::GetPoints3DForImage(const colmap::image_t& image_id,
                                                                        const int min_track_len,
                                                                        const std::shared_ptr<colmap::Reconstruction> reconstruction) {
  const auto& img = reconstruction->Image(image_id);
  std::vector<colmap::Point3D> pts3D;

  // iterate over all 2d points associated to image
  for (const colmap::Point2D& point2D : img.Points2D()) {
    if (!point2D.HasPoint3D()) {
      continue;
    }

    // skip points with less than 2 views.
    const auto& point3D = reconstruction->Point3D(point2D.point3D_id);
    if (point3D.track.Length() < min_track_len) {
      continue;
    }

    // recover associated 3d point and store in output vector
    pts3D.push_back(point3D);
  }

  return pts3D;
}

void fuhe::col_utils::CropFarAwayPoints(const std::shared_ptr<colmap::Reconstruction> reconstruction) {
  VLOG(2) << "Cropping out far away 3d points from colmap model!";
  auto bbox = reconstruction->ComputeBoundingBox(0.0, 0.8);

  VLOG(3) << "Bounding Box Corenrs 1 are: " << bbox.first;
  VLOG(3) << "Bounding Box Corenrs 2 are: " << bbox.second;

  std::vector<colmap::point3D_t> for_del;  // collect point ids for deletion
  for (const auto& point3D : reconstruction->Points3D()) {
    if (!((point3D.second.xyz.array() >= bbox.first.array()).all() && (point3D.second.xyz.array() <= bbox.second.array()).all())) {
      VLOG(4) << "Bogus 3d point detected! Prepare deletion of id: " << point3D.first;
      VLOG(4) << "Bogus 3d Position: \n" << point3D.second.xyz;
      // NOTE: Do not delete point while iterating
      for_del.push_back(point3D.first);
    }
  }
  VLOG(2) << "Number of 3d Points to delete: " << for_del.size();

  for (auto& id : for_del) {
    reconstruction->DeletePoint3D(id);
    VLOG(4) << "Commence deletion of id: " << id;
  }
}

void fuhe::col_utils::GetPointersToPose(colmap::Image& img, double*& q_c_from_w, double*& t_c_from_w) {
  // -------------------- Recover pointers to image poses from colmap model
  // recover image pose from colmap model

  img.CamFromWorld().rotation.normalize();

  // cam from world -> pose of world expressed in camera frame
  q_c_from_w = img.CamFromWorld().rotation.coeffs().data();  // pointer to quaternion part of image pose.
                                                             // represents ceres parameter pointer to position part of image pose.
  t_c_from_w = img.CamFromWorld().translation.data();        // pointer to translation part of image pose.
                                                             // represents ceres parameter pointer to position part of image pose.
}

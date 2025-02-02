#include "fusion_helper/col_utils.h"
#include "fusion_helper/io.h"

#include <colmap/util/file.h>

std::map<const double, colmap::image_t> fuhe::col_utils::ImageIdsByStamp(const std::set<colmap::image_t>& image_ids,
                                                                         std::shared_ptr<colmap::Reconstruction> reconstruction) {
  std::map<const double, colmap::image_t> ordered_image_stamps;  // output map -> image ids by timestamps [secs]

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

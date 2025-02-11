#include "high_level_fusion/rerun_interface.h"

#include "high_level_fusion/collection_adapters.h"
#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>

void rrfuse::LogCamPose(std::shared_ptr<rerun::RecordingStream>& rec,
                        std::shared_ptr<rerun::Pinhole>& rrpinhole,
                        const colmap::Image& img,
                        const colmap::image_t& id) {
  
  std::string cam_name = "world/cam" + std::to_string(id);
  
  // downcast to float to match rerun adapter type. Invert to obtain pose of cam with respect to world.
  const Eigen::Vector3f t = colmap::Inverse(img.CamFromWorld()).translation.cast<float>();
  Eigen::Matrix<float, 3, 3> R = colmap::Inverse(img.CamFromWorld()).rotation.toRotationMatrix().cast<float>();

  // establish camera for logged pose under same name as pose
  rec->log(cam_name, *rrpinhole);
  // log pose.  data of t and R may go out of scope, but as rerun object they shall live on
  rec->log(cam_name, rerun::Transform3D(rerun::Vec3D(t.data()), rerun::Mat3x3(R.data())));
}
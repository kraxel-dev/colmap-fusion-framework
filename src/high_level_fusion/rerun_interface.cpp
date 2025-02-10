
#include "high_level_fusion/collection_adapters.h"
#include "high_level_fusion/rerun_interface.h"

#include <Eigen/Core>
#include <colmap/geometry/rigid3.h>

void rrfuse::LogCamPose(const colmap::Image& img) {
  auto rec = rerun::RecordingStream("bundle", "shared");
  rec.spawn().exit_on_failure();
  // auto result = rec.connect_tcp();  // Connect to local host with default port.

  // if (result.is_err()) {
  //   LOG(WARNING) << "Failed to connect to rerun server! Skipping sending campose to rerun!";
  //   return;
  // }

  // downcast to float to match rerunn adapter type. Invert to obtain pose of cam with respect to world.
  const Eigen::Vector3f t = colmap::Inverse(img.CamFromWorld()).translation.cast<float>();
  Eigen::Matrix<float, 3, 3> R = colmap::Inverse(img.CamFromWorld()).rotation.toRotationMatrix().cast<float>();

  // data of t and R may go out of scope, but as rerun object they shall live on
  rec.log("cam_pose", rerun::Transform3D(rerun::Vec3D(t.data()), rerun::Mat3x3(R.data())));
}
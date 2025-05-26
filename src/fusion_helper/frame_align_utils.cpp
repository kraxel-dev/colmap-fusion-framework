#include "fusion_helper/frame_align_utils.h"

#include "fusion_helper/col_utils.h"
#include "fusion_helper/stream_utils.h"

namespace fuhe {
namespace align {

bool CheckRunAlignment(const int n_registered_imgs, const AlignmentOptions& align_opts) {
  if (align_opts.n_reg_for_alignment == n_registered_imgs && align_opts.n_reg_for_alignment != 0) {
    return true;
  }
  return false;
}

void AlignInitMotionWithGlobalXAxis(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                                    const colmap::image_t id1,
                                    const colmap::image_t id2) {
  const colmap::Rigid3d T1 = colmap::Inverse(reconstruction->Image(id1).CamFromWorld());  // pose of cam 1 w.r.t to world
  const colmap::Rigid3d T2 = colmap::Inverse(reconstruction->Image(id2).CamFromWorld());  // pose of cam 2 w.r.t to world
  // to translate 1st pose to origin without changing orientation
  const colmap::Rigid3d T_to_origin(Eigen::Quaterniond::Identity(), (-1 * T1.translation));

  // direction of the initial motion
  const Eigen::Vector3d t_init_motion((T2.translation - T1.translation).normalized());
  VLOG(3) << "Init motion: " << t_init_motion;
  // direction of x axis in global frame that we want to rotate the init motion into
  const Eigen::Vector3d t_x_axis({1.0, 0.0, 0.0});

  // obtain rotation axis between global x-axis dir and init-motion-dir
  const auto rot_axis = t_x_axis.cross(t_init_motion);             // rotation axis between the 2 directions as cross product
  const double theta = acos(t_x_axis.dot(t_init_motion));          // theta between the 2 directions
  const Eigen::Quaterniond q(Eigen::AngleAxisd(theta, rot_axis));  // rotation to bring x axis onto init motion dir

  // target rotation that would rotate the initial motion onto the x axis
  const colmap::Rigid3d T_target_rot(q.inverse(), Eigen::Vector3d::Zero());
  VLOG(3) << "Target rotation tf: " << T_target_rot;

  // the final tf to apply. Bring 1st pose to origin, then apply target rotation (for some reason inverted)
  const colmap::Rigid3d apply = T_target_rot * T_to_origin;
  reconstruction->Transform(colmap::Sim3d(1, apply.rotation, apply.translation));
  VLOG(2) << "Applying rotation to colmap model to align init motion with global x axis!";
}

void AlignFirstPoseToSpecified(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                               const colmap::image_t& id1,
                               const AlignmentOptions& align_opts) {
  // translate 1st cam pose onto specified translation (e.g. camera extrinsic w.r.t. to vehicle)
  if (align_opts.align_first_cam_to_specific_pose) {
    // pose of cam 1 w.r.t to world
    const colmap::Rigid3d T1 = colmap::Inverse(reconstruction->Image(id1).CamFromWorld());

    // obtain pose of specified extrinsic
    // specified translation
    const Eigen::Vector3d target_t(align_opts.specified_x, align_opts.specified_y, align_opts.specified_z);
    // zyx (intrinsic) euler angles of extrinsics to rotation matrix
    const Eigen::Matrix3d target_R =
        colmap::EulerAnglesToRotationMatrix(align_opts.specified_roll, align_opts.specified_pitch, align_opts.specified_yaw);
    const Eigen::Quaterniond target_q(target_R);
    colmap::Rigid3d T_target(target_q, target_t);
    VLOG(2) << "Specified pose target for 1st colmap cam pose is: " << T_target;

    // extra rotation apply to tilt cam center into optical frame
    if (align_opts.auto_rot_into_optical) {
      // w, x, y, z notation (ROS would be x, y, z, w)
      const Eigen::Quaterniond tilt(-0.500, 0.500, -0.500, 0.500);  // from x forward to z forward y down
      T_target.rotation = T_target.rotation * tilt;
      VLOG(2) << "Applying tilt onto target pose to obtain corrected extrinsics of: " << T_target;
    }

    // To bring 1st colmap pose onto extrinsic position, use following equation:
    // T_extr = T? * T1 <=> T? = T_extr * inv(T1)
    const colmap::Rigid3d apply = T_target * colmap::Inverse(T1);

    reconstruction->Transform(colmap::Sim3d(1, apply.rotation, apply.translation));
    VLOG(2) << "Aligning 1st colmap cam pose to specified pose through: " << apply;
  }
}

void PerformAlignmentStrategies(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                                const AlignmentOptions& align_opts) {
  if (reconstruction->NumRegImages() < 2) {
    LOG(WARNING) << "Trying to align colmap model to inital pose but not enough imgs registered yet.";
    return;
  }

  // align reconstruction along motion axes by PCA
  if (align_opts.pca_align) {
    VLOG(2) << "Aligning reconstruction via PCA to dominant motion axes!";

    colmap::Sim3d tf;
    colmap::AlignToPrincipalPlane(reconstruction.get(), &tf);

    VLOG(2) << "PCA Transform:\n" << tf.ToMatrix();
  }

  // obtain very first 2 poses in model (seen from time stamps)
  fuhe::types::MapOfImageIdsSec ids_by_stamp = col_utils::ImageIdsByStamp(col_utils::RegisteredImages(reconstruction));
  auto id1 = ids_by_stamp.begin()->second;             // very 1st pose
  auto id2 = std::next(ids_by_stamp.begin())->second;  // 2nd pose

  // orient the colmap trajectory such that init motion aligns with global x axis
  if (align_opts.rotate_init_motion_onto_global_x_axis) {
    AlignInitMotionWithGlobalXAxis(reconstruction, id1, id2);
  }

  // bring 1st colmap image pose to specified target pose
  if (align_opts.align_first_cam_to_specific_pose) {
    AlignFirstPoseToSpecified(reconstruction, id1, align_opts);
  }
}

}  // namespace align
}  // namespace fuhe

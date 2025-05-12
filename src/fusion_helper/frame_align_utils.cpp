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

void AlignFirstPoseToSpecified(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                               const AlignmentOptions& align_opts) {
  if (reconstruction->NumRegImages() < 2) {
    LOG(WARNING) << "Trying to align colmap model to inital pose but not enough imgs registered yet.";
    return;
  }
  // obtain very first 2 poses in model (seen from time stamps)
  fuhe::types::MapOfImageIdsSec ids_by_stamp = col_utils::ImageIdsByStamp(col_utils::RegisteredImages(reconstruction));
  auto id1 = ids_by_stamp.begin()->second;             // very first pose
  auto id2 = std::next(ids_by_stamp.begin())->second;  // 2nd pose

  // orient the colmap trajectory such that init motion aligns with global x axis
  if (align_opts.rotate_init_motion_onto_global_x_axis) {
    const colmap::Rigid3d T1 = colmap::Inverse(reconstruction->Image(id1).CamFromWorld());  // pose of cam 1 w.r.t to world
    const colmap::Rigid3d T2 = colmap::Inverse(reconstruction->Image(id2).CamFromWorld());  // pose of cam 2 w.r.t to world
    // to translate 1st pose to origin without changing orientation
    const colmap::Rigid3d T_to_origin(Eigen::Quaterniond::Identity(), (-1 * T1.translation));

    // direction of the initial motion
    const Eigen::Vector3d t_init_motion((T2.translation - T1.translation).normalized());
    VLOG(3) << "Init motion: " << t_init_motion;
    // direction of x axis in global frame that we want to rotate the init motion into
    const Eigen::Vector3d t_x_axis({1.0, 0.0, 0.0});

    // rotation axis between the 2 directions as cross product
    const auto rot_axis = t_x_axis.cross(t_init_motion);
    // theta between the 2 directions
    const double theta = acos(t_x_axis.dot(t_init_motion));
    // rotation to bring x axis onto init motion dir
    const Eigen::Quaterniond q(Eigen::AngleAxisd(theta, rot_axis));

    // target rotation that would rotate the initial motion onto the x axis
    const colmap::Rigid3d T_target_rot(q.inverse(), Eigen::Vector3d::Zero());
    VLOG(3) << "Target rotation tf: " << T_target_rot;

    // the final tf to apply. Bring 1st pose to origin, then apply target rotation (for some reason inverted)
    const colmap::Rigid3d apply = T_target_rot * T_to_origin;
    reconstruction->Transform(colmap::Sim3d(1, apply.rotation, apply.translation));
    VLOG(2) << "Applying rotation to colmap model to align init motion with global x axis!";
  }

  // translate 1st cam pose onto specified translation (e.g. camera extrinsic w.r.t. to vehicle)
  if (align_opts.align_first_cam_to_specific_pos) {
    const colmap::Rigid3d T1 = colmap::Inverse(reconstruction->Image(id1).CamFromWorld());  // pose of cam 1 w.r.t to world
    // to translate 1st pose to origin without changing orientation
    const colmap::Rigid3d T_to_origin(Eigen::Quaterniond::Identity(), (-1 * T1.translation));
    // to bring 1st campose into target translation without changing orientation
    const Eigen::Vector3d target_position(align_opts.align_cam_to_x, align_opts.align_cam_to_y, align_opts.align_cam_to_z);
    const colmap::Rigid3d T_target_translate(Eigen::Quaterniond::Identity(), target_position);

    // the final tf to apply. Bring 1st position to origin, then apply target translation
    const colmap::Rigid3d apply = T_target_translate * T_to_origin;
    reconstruction->Transform(colmap::Sim3d(1, apply.rotation, apply.translation));
    VLOG(2) << "Aligning 1st colmap cam pose to specified translation: " << target_position;
  }
}

void PerformAlignmentStrategies(const std::shared_ptr<colmap::Reconstruction> reconstruction,
                                const AlignmentOptions& align_opts) {
  // align reconstruction along motion axes by PCA
  if (align_opts.pca_align) {
    VLOG(2) << "Aligning reconstruction via PCA to dominant motion axes!";

    colmap::Sim3d tf;
    colmap::AlignToPrincipalPlane(reconstruction.get(), &tf);

    VLOG(2) << "PCA Transform:\n" << tf.ToMatrix();
  }

  // rotate colmap trajectory according to a stragey and to bring 1st colmap image pose to specified target position
  AlignFirstPoseToSpecified(reconstruction, align_opts);
}

}  // namespace align
}  // namespace fuhe

#include "high_level_fusion/fusion_graph_interface.h"

#include "fusion_helper/ceres_eval_utils.h"
#include "fusion_helper/col_utils.h"
#include "high_level_fusion/rerun_interface.h"
#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/manifold.h>

hifuse::FusionGraphInterface::FusionGraphInterface(std::shared_ptr<colmap::Reconstruction> reconstruction,
                                                   std::shared_ptr<ceres::Problem> ceres_graph,
                                                   const bool log_to_rerun,
                                                   const bool save_rerun_recording,
                                                   const std::string recording_path)
    : is_log_to_rerun{log_to_rerun},
      ceres_graph{ceres_graph},
      reconstruction{reconstruction},
      is_save_rerun_to_disk{save_rerun_recording},
      recording_path{recording_path} {
  if (!log_to_rerun) {
    VLOG(2) << "Rerun logging and visualization turned off!";
    return;
  }
  InitRerunViewer();
}

void hifuse::FusionGraphInterface::AddReprojectionFactor(const colmap::image_t img_id,
                                                         const bool const_t,
                                                         const bool const_q,
                                                         const bool const_3d_pts) {
  // -------------------- Recover pointers to image pose from colmap model
  // recover image pose from colmap model
  colmap::Image& img = this->reconstruction->Image(img_id);
  colmap::Camera& cam = this->reconstruction->Camera(img.CameraId());
  VLOG(3) << "Prepare reprojection factors for image id: " << img_id;

  double *q_cw = nullptr, *t_cw = nullptr;  // pointers to pose params of image
  fuhe::col_utils::GetPointersToPose(img, q_cw, t_cw);
  double* camera_params = cam.params.data();

  std::vector<colmap::Point3D> points3D_curr_img;  // store 3D points for rerun
  std::vector<ceres::ResidualBlockId>
      reproj_residual_ids_curr_img;  // ceres residual block ids of all reprojection factors for current image

  size_t num_observations = 0;
  std::unordered_map<colmap::point3D_t, size_t> point3D_num_observations;
  const int min_track_len = 2;

  VLOG(3) << "Starting to iterate over all 2d points of target image!";
  // -------------------- Iterate over all 2d points associated to image // NOTE: keep loop fixed to 2d instead of 3d points since its hard
  // to recover 2d ids from 3d pts
  for (const colmap::Point2D& point2D : img.Points2D()) {
    if (!point2D.HasPoint3D()) {
      continue;
    }

    // recover associated 3d point
    colmap::Point3D& point3D = reconstruction->Point3D(point2D.point3D_id);
    if (point3D.track.Length() < min_track_len) {
      continue;
    }
    double* pt3Dxyz = point3D.xyz.data();

    num_observations += 1;
    point3D_num_observations[point2D.point3D_id] += 1;
    points3D_curr_img.push_back(point3D);  // append for Rerun visualization

    // create reprojection error cost function for 3d point
    ceres::CostFunction* cost_function = colmap::CreateCameraCostFunction<colmap::ReprojErrorCostFunctor>(cam.model_id, point2D.xy);

    // add cost function to ceres problem
    reproj_residual_ids_curr_img.push_back(
        ceres_graph->AddResidualBlock(cost_function, new ceres::HuberLoss(1.5), q_cw, t_cw, point3D.xyz.data(), camera_params));

    // log reisudal error of current reprojecion factor
    if (VLOG_IS_ON(5)) {
      fuhe::ceres_eval_utils::LogReprojFactorCost(cost_function, q_cw, t_cw, pt3Dxyz, camera_params);
    }

    // if user doenst want poisiton of 3d points to be optimized in ceres problem
    if (const_3d_pts) {
      //  force 3d point to consant position
      ceres_graph->SetParameterBlockConstant(point3D.xyz.data());
      VLOG(2) << "Set 3d point of id " << point2D.point3D_id << " to constant!";
    }
  }

  // if user doenst want camera poisiton to be optimized in ceres problem
  if (const_t) {
    //  force 3d point to consant position
    ceres_graph->SetParameterBlockConstant(t_cw);
    VLOG(2) << "Set cam position of id " << img_id << " to constant!";
  }

  // if user doenst want camera poisiton to be optimized in ceres problem
  if (const_q) {
    //  force 3d point to consant position
    ceres_graph->SetParameterBlockConstant(q_cw);
    VLOG(2) << "Set cam orientation of id " << img_id << " to constant!";
  }

  // log camera pose and 3d pts to rerun
  if (this->is_log_to_rerun) {
    rrfuse::LogCamPose(this->rr_rec, this->rr_pinhole, img);
    rrfuse::LogCamPoints3D(this->rr_rec, img, points3D_curr_img);
  }
  this->reproj_residual_ids.push_back(reproj_residual_ids_curr_img);
}

void hifuse::FusionGraphInterface::AddBetweenFactor(const colmap::image_t img_id_i,
                                                    const colmap::image_t img_id_j,
                                                    const Eigen::Isometry3d& i_from_j,
                                                    const Eigen::Matrix<double, 6, 6> cov_i_from_j) {
  // -------------------- Recover pointers to image poses from colmap model
  colmap::Image& img_i = this->reconstruction->Image(img_id_i);  // prev image
  colmap::Image& img_j = this->reconstruction->Image(img_id_j);  // curr image

  // recover image pose from colmap model
  double *q_i = nullptr, *t_i = nullptr, *q_j = nullptr, *t_j = nullptr;  // pointers to pose params of both images
  fuhe::col_utils::GetPointersToPose(img_i, q_i, t_i);
  fuhe::col_utils::GetPointersToPose(img_j, q_j, t_j);

  // -------------------- Create between factor and add to problem
  // convert raltive eigen pose to colmap format
  const colmap::Rigid3d T_ij_rigid = colmap::Rigid3d(Eigen::Quaterniond(i_from_j.rotation()), i_from_j.translation());

  // if (this->is_log_to_rerun) {
  //   rrfuse::LogRelPoseFactor(this->rr_rec, T_ij_rigid, img_i, img_j);
  // }

  VLOG(3) << "Creating metric relative odom cost function from img id: " << img_id_i << " to id: " << img_id_j;
  // create ceres relaitve pose factor weighted by its covariance
  ceres::CostFunction* weighted_cost_function =
      colmap::CovarianceWeightedCostFunctor<colmap::RelativePosePriorCostFunctor>::Create(cov_i_from_j, T_ij_rigid);

  if (VLOG_IS_ON(5)) {
    fuhe::ceres_eval_utils::LogBetweenFactorCost(weighted_cost_function, q_i, t_i, q_j, t_j);
  }

  VLOG(3) << "Adding residual block to ceres graph!";
  // register odom between factor in ceres graph and directly retrieve id of residual block. order of params: q_i, t_i, q_j, t_j
  this->odom_residual_ids.push_back(ceres_graph->AddResidualBlock(
      weighted_cost_function, nullptr, q_i, t_i, q_j, t_j));  // TODO: investiage loss function usage in this residual

  // -------------------- Double check if pose parameters are registered as manifold in optimizaion
  // Set Lie algebra for pose on manifold optimization in case it was not set already
  if (!ceres_graph->GetParameterization(q_i)) {
    colmap::SetQuaternionManifold(ceres_graph.get(), q_i);
  }
  if (!ceres_graph->GetParameterization(q_j)) {
    colmap::SetQuaternionManifold(ceres_graph.get(), q_j);
  }
}

void hifuse::FusionGraphInterface::UpdateRegisterdFactorsRerun(const fuhe::types::MapOfPosesSec& metric_poses) {
  auto imgs_by_stamp = fuhe::col_utils::ImageIdsByStamp(this->reconstruction->RegImageIds(), this->reconstruction);

  int i = 0;
  colmap::image_t curr_img_id, prev_img_id = -1;
  double curr_img_stamp = -1, prev_stamp = -1;
  const int min_track_length = 2;

  // iterate over all images in model
  for (const auto pair : imgs_by_stamp) {
    curr_img_stamp = pair.first;
    curr_img_id = pair.second;
    // -------------------- First iteration init condition
    if (i == 0) {
      // start only if synchronized meas from both sources are availalbe
      if (metric_poses.find(curr_img_stamp) != metric_poses.end()) {
        rrfuse::LogCamPose(this->rr_rec, this->rr_pinhole, this->reconstruction->Image(curr_img_id));
        rrfuse::LogCamPoints3D(this->rr_rec,
                               this->reconstruction->Image(curr_img_id),
                               fuhe::col_utils::GetPoints3DForImage(curr_img_id, min_track_length, this->reconstruction));

        // preparing next iteration
        prev_stamp = curr_img_stamp;
        i++;  // break init loop
      }
      continue;
    }

    VLOG(2) << "Searching matching tumpose:" << curr_img_id;

    // check if external odom is available for current image
    if (metric_poses.find(curr_img_stamp) == metric_poses.end()) {
      i++;
      continue;
    }

    rrfuse::LogCamPose(this->rr_rec, this->rr_pinhole, this->reconstruction->Image(curr_img_id));
    rrfuse::LogCamPoints3D(this->rr_rec,
                           this->reconstruction->Image(curr_img_id),
                           fuhe::col_utils::GetPoints3DForImage(curr_img_id, min_track_length, this->reconstruction));

    // -------------------- Fetch relative pose
    // Get metric relative  pose of j (curr) expressed in i (prev) := i_from_j = world_from_i.inverse() * world_from_j
    const Eigen::Isometry3d T_i_from_j = metric_poses.at(prev_stamp).inverse() * metric_poses.at(curr_img_stamp);

    // -------------------- Update rel pose factor in rerun
    colmap::image_t prev_img_id = imgs_by_stamp.at(prev_stamp);
    rrfuse::LogRelPoseFactor(this->rr_rec,
                             colmap::Rigid3d(Eigen::Quaterniond(T_i_from_j.rotation()), T_i_from_j.translation()),
                             this->reconstruction->Image(prev_img_id),
                             this->reconstruction->Image(curr_img_id));
    // preparing next iteration
    prev_stamp = curr_img_stamp;
    i++;
  }
}

void hifuse::FusionGraphInterface::UpdateWholeReconstroctionRerun() {
  rrfuse::LogReconstruction(this->rr_rec, this->rr_pinhole, this->reconstruction->Images(), this->reconstruction->Points3D());
}

void hifuse::FusionGraphInterface::InitRerunViewer() {
  // --------------------
  VLOG(2) << "Initializing rerun viewer for fusion graph!";
  this->rr_rec = std::make_shared<rerun::RecordingStream>("bundle", "shared/nested");
  this->rr_rec->spawn().exit_on_failure();

  this->rr_rec->log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);  // Set an up-axis

  // -------------------- Save rerun recording to disk if specified
  if (this->is_save_rerun_to_disk) {
    VLOG(2) << "Saving rerun recording to rr file!";
    this->rr_rec->save(std::string_view(this->recording_path + "/recording.rrd")).exit_on_failure();
    VLOG(2) << "Path: " << std::string_view(this->recording_path + "/recording.rrd");
  }

  // --------------------
  // TODO: make generic for all registered cameras (currently we assume that the same pinhole model was used)
  // obtain camera params from first image in colmap model
  colmap::Camera cam = this->reconstruction->Camera(this->reconstruction->Images().begin()->second.CameraId());

  const float focal_length_x = cam.FocalLengthX(), focal_length_y = cam.FocalLengthY();
  const float width = cam.width, height = cam.height;
  VLOG(2) << "Focal length of first camera in model: " << focal_length_x << " and " << focal_length_y;
  VLOG(2) << "Resolution of first camera in model [pxl]: " << width << " and " << height;

  // create rerun pinhole object needed to visualize camera poses in rerun
  this->rr_pinhole = std::make_shared<rerun::Pinhole>(
      rerun::Pinhole::from_focal_length_and_resolution({focal_length_x, focal_length_y}, {width, height}).with_image_plane_distance(0.1));
  // create different looking pinhole object for pose predicted from wheel odom
  this->rr_pinhole_pred = std::make_shared<rerun::Pinhole>(
      rerun::Pinhole::from_focal_length_and_resolution({focal_length_y / 2, focal_length_x / 2}, {height / 2, width / 2}));
}

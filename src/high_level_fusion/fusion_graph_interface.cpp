#include "high_level_fusion/fusion_graph_interface.h"

#include "fusion_helper/ceres_eval_utils.h"
#include "fusion_helper/col_utils.h"
#include "high_level_fusion/rerun_interface.h"
#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/manifold.h>

hifuse::FusionGraphInterface::FusionGraphInterface(std::shared_ptr<colmap::Reconstruction>& reconstruction,
                                                   std::shared_ptr<ceres::Problem>& ceres_graph,
                                                   bool log_to_rerun)
    : log_to_rerun(log_to_rerun), ceres_graph(ceres_graph), reconstruction(reconstruction) {
  if (log_to_rerun) {
    InitRerunViewer();
  }
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

  // log camera pose to rerun
  if (this->log_to_rerun) {
    rrfuse::LogCamPose(this->rec, this->rr_pinhole, img, img_id);
  }

  // -------------------- Iterate over all 2d points associated to image
  std::vector<ceres::ResidualBlockId>
      reproj_residual_ids_curr_img;  // ceres residual block ids of all reprojection factors for current image
  size_t num_observations = 0;
  std::unordered_map<colmap::point3D_t, size_t> point3D_num_observations;
  VLOG(3) << "Starting to iterate over all 2d points of target image!";

  // iterate over all 2d points associated to image
  for (const colmap::Point2D& point2D : img.Points2D()) {
    if (!point2D.HasPoint3D()) {
      continue;
    }

    num_observations += 1;
    point3D_num_observations[point2D.point3D_id] += 1;

    // recover associated 3d point
    colmap::Point3D& point3D = reconstruction->Point3D(point2D.point3D_id);
    double* pt3Dxyz = point3D.xyz.data();

    // create reprojection error cost function for 3d point
    ceres::CostFunction* cost_function = colmap::CreateCameraCostFunction<colmap::ReprojErrorCostFunctor>(cam.model_id, point2D.xy);

    // add cost function to ceres problem
    reproj_residual_ids_curr_img.push_back(
        ceres_graph->AddResidualBlock(cost_function, new ceres::HuberLoss(1.5), q_cw, t_cw, point3D.xyz.data(), camera_params));

    // log reisudal error of current reprojecion factor
    if (VLOG_IS_ON(5)) {
      fuhe::ceres_eval_utils::LogReprojFactorCost(cost_function, q_cw, t_cw, pt3Dxyz, camera_params);
    }

    // if user doenst want 3d poisiton to be optimized in ceres problem
    if (const_3d_pts) {
      //  force 3d point to consant position
      ceres_graph->SetParameterBlockConstant(point3D.xyz.data());
      VLOG(2) << "Set 3d point of id " << point2D.point3D_id << " to constant!";
    }
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

void hifuse::FusionGraphInterface::InitRerunViewer() {
  VLOG(2) << "Initializing rerun viewer for fusion graph!";
  this->rec = std::make_shared<rerun::RecordingStream>("bundle", "shared");
  this->rec->spawn().exit_on_failure();

  this->rec->log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);  // Set an up-axis

  // TODO: make generic for all registered cameras (currently we assume that the same pinhole model was used)
  // obtain camera params from first image in colmap model
  colmap::Camera cam = this->reconstruction->Camera(this->reconstruction->Images().begin()->second.CameraId());

  const float focal_length_x = cam.FocalLengthX(), focal_length_y = cam.FocalLengthY();
  const float width = cam.width, height = cam.height;
  VLOG(2) << "Focal length of first camera in model: " << focal_length_x << " and " << focal_length_y;
  VLOG(2) << "Resolution of first camera in model: " << width << " and " << height;

  this->rr_pinhole =
      std::make_shared<rerun::Pinhole>(rerun::Pinhole::from_focal_length_and_resolution({focal_length_x, focal_length_y}, {width, height}));
}

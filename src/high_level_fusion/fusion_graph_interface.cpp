#include "high_level_fusion/fusion_graph_interface.h"

#include <colmap/estimators/cost_functions.h>
#include <colmap/estimators/manifold.h>

void hifuse::GetPointersToPose(colmap::Image& img, double*& q_c_from_w, double*& t_c_from_w) {
  // -------------------- Recover pointers to image poses from colmap model
  // recover image pose from colmap model

  img.CamFromWorld().rotation.normalize();

  // cam from world -> pose of world expressed in camera frame
  q_c_from_w = img.CamFromWorld().rotation.coeffs().data();  // pointer to quaternion part of image pose.
                                                             // represents ceres parameter pointer to position part of image pose.
  t_c_from_w = img.CamFromWorld().translation.data();        // pointer to translation part of image pose.
                                                             // represents ceres parameter pointer to position part of image pose.
}

std::vector<ceres::ResidualBlockId> hifuse::AddReprojectionFactor(const colmap::image_t img_id,
                                                                  std::shared_ptr<ceres::Problem> ceres_graph,
                                                                  std::shared_ptr<colmap::Reconstruction> reconstruction,
                                                                  const bool const_t,
                                                                  const bool const_q,
                                                                  const bool const_3d_pts) {
  // -------------------- Recover pointers to image pose from colmap model
  // recover image pose from colmap model
  colmap::Image& img = reconstruction->Image(img_id);
  colmap::Camera& cam = reconstruction->Camera(img.CameraId());
  VLOG(3) << "Prepare reprojection factors for image id: " << img_id;

  double *q_cw = nullptr, *t_cw = nullptr;  // pointers to pose params of image
  hifuse::GetPointersToPose(img, q_cw, t_cw);
  double* camera_params = cam.params.data();

  // -------------------- Iterate over all 2d points associated to image
  std::vector<ceres::ResidualBlockId> reproj_residual_ids;  // ceres residual block ids of all reprojection factors
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
    reproj_residual_ids.push_back(
        ceres_graph->AddResidualBlock(cost_function, new ceres::HuberLoss(1.5), q_cw, t_cw, point3D.xyz.data(), camera_params));

    // log reisudal error of current reprojecion factor
    if (VLOG_IS_ON(5)) {
      const double* params[4] = {q_cw, t_cw, pt3Dxyz, camera_params};
      double residuals[2];

      cost_function->Evaluate(params, residuals, nullptr);
      VLOG(5) << "Reprojection error -> x y residuals in image coords [px]: ";
      VLOG(5) << "x [px]: " << residuals[0] << " y [px]: " << residuals[1];
    }

    // if user doenst want 3d poisiton to be optimized in ceres problem
    if (const_3d_pts) {
      //  force 3d point to consant position
      ceres_graph->SetParameterBlockConstant(point3D.xyz.data());
      VLOG(2) << "Set 3d point of id " << point2D.point3D_id << " to constant!";
    }
  }
  return reproj_residual_ids;
}

ceres::ResidualBlockId hifuse::AddBetweenFactor(const colmap::image_t img_id_i,
                                                const colmap::image_t img_id_j,
                                                const Eigen::Isometry3d& i_from_j,
                                                const Eigen::Matrix<double, 6, 6> cov_i_from_j,
                                                std::shared_ptr<ceres::Problem> ceres_graph,
                                                std::shared_ptr<colmap::Reconstruction> model) {
  // -------------------- Recover pointers to image poses from colmap model
  colmap::Image& img_i = model->Image(img_id_i);  // prev image
  colmap::Image& img_j = model->Image(img_id_j);  // curr image

  // recover image pose from colmap model
  double *q_i = nullptr, *t_i = nullptr, *q_j = nullptr, *t_j = nullptr;  // pointers to pose params of both images
  hifuse::GetPointersToPose(img_i, q_i, t_i);
  hifuse::GetPointersToPose(img_j, q_j, t_j);

  // -------------------- Create between factor and add to problem
  ceres::ResidualBlockId
      odom_residual_id;  // id for current odom factor such that we can refer to it in upstream code sections (e.g for residual evaluation)

  // convert raltive eigen pose to colmap format
  const colmap::Rigid3d T_ij_rigid = colmap::Rigid3d(Eigen::Quaterniond(i_from_j.rotation()), i_from_j.translation());

  VLOG(3) << "Creating metric relative odom cost function from img id: " << img_id_i << " to id: " << img_id_j;
  // create ceres relaitve pose factor weighted by its covariance
  ceres::CostFunction* weighted_cost_function =
      colmap::CovarianceWeightedCostFunctor<colmap::RelativePosePriorCostFunctor>::Create(cov_i_from_j, T_ij_rigid);

  // log reisudal error of current between factor
  if (VLOG_IS_ON(3)) {
    const double* params[4] = {q_i, t_i, q_j, t_j};
    double residuals[6];

    weighted_cost_function->Evaluate(params, residuals, nullptr);
    VLOG(3) << "X Y Z residuals in [m]: ";
    VLOG(3) << "x: " << residuals[3] << " y: " << residuals[4] << " z: " << residuals[5];
  }

  VLOG(3) << "Adding residual block to ceres graph!";
  // order of params: q_i, t_i, q_j, t_j
  odom_residual_id = ceres_graph->AddResidualBlock(
      weighted_cost_function, nullptr, q_i, t_i, q_j, t_j);  // TODO: investiage loss function usage in this residual

  // -------------------- Double check if pose parameters are registered as manifold in optimizaion
  // Set Lie algebra for pose on manifold optimization in case it was not set already
  if (!ceres_graph->GetParameterization(q_i)) {
    colmap::SetQuaternionManifold(ceres_graph.get(), q_i);
  }
  if (!ceres_graph->GetParameterization(q_j)) {
    colmap::SetQuaternionManifold(ceres_graph.get(), q_j);
  }

  return odom_residual_id;
}
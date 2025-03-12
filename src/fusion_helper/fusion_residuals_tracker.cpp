#include "fusion_helper/fusion_residuals_tracker.h"

namespace fuhe {

const std::shared_ptr<FusionResidualsTracker> FusionResidualsTracker::Create() { return std::make_shared<FusionResidualsTracker>(); }

void FusionResidualsTracker::RegisterStalkedOdomResidual(const std::shared_ptr<ResidualStalker<6>> odom_residual_stalker,
                                                         const std::string& edge_id) {
  if (!odom_residual_stalker) {
    LOG(WARNING) << "Tried to add nullptr instead of odom residual stalker to fusion residual tracker!";
    return;
  }

  std::string id = edge_id;
  if (this->stalked_odom_residuals.find(edge_id) != this->stalked_odom_residuals.end()) {
    // in case edge id is duplicate due to mutliple sensor sources
    id += "v2";
  }
  odom_residual_stalker->SetId(id);
  this->stalked_odom_residuals[odom_residual_stalker->GetId()] = odom_residual_stalker;
};

void FusionResidualsTracker::RegisterStalkedReprojectionResidual(const std::shared_ptr<ResidualStalker<2>> reproj_residual_stalker,
                                                                 const std::string& img_id,
                                                                 const std::string& pt3D_id) {
  if (!reproj_residual_stalker) {
    LOG(WARNING) << "Tried to add nullptr instead of reprojection residual stalker to fusion residual tracker!";
    return;
  }

  std::string id = img_id + "_" + pt3D_id;  // 3d point id per image id
  if (this->stalked_reprojection_residuals.find(id) != this->stalked_reprojection_residuals.end()) {
    // the same 3d point id might be matched to multiple 2d features per image
    id += "v2";
  }

  reproj_residual_stalker->SetId(id);
  this->stalked_reprojection_residuals[reproj_residual_stalker->GetId()] = reproj_residual_stalker;
};

const std::map<std::string, std::shared_ptr<ResidualStalker<6>>>& FusionResidualsTracker::StalkedOdomResiduals() const {
  return this->stalked_odom_residuals;
}

const std::map<std::string, std::shared_ptr<ResidualStalker<2>>>& FusionResidualsTracker::StalkedReprojectionResiduals() const {
  return this->stalked_reprojection_residuals;
}

const double FusionResidualsTracker::GetTotalOdomCost() const {
  Eigen::Matrix<double, 6, 1> summed_residuals = Eigen::Matrix<double, 6, 1>::Zero();  // summed total residuals
  double total_cost = 0;                                                               // 1/2 * squared total cost

  for (auto& [id, stalker] : this->StalkedOdomResiduals()) {
    summed_residuals += stalker->GetTrackedResidual();
    total_cost += 0.5 * stalker->GetTrackedResidual().dot(stalker->GetTrackedResidual());  // add square product
  };

  return total_cost;
}

const double FusionResidualsTracker::GetTotalReprojCost() const {
  Eigen::Matrix<double, 2, 1> summed_residuals = Eigen::Matrix<double, 2, 1>::Zero();  // summed total residuals
  double total_cost = 0;                                                               // 1/2 * squared total cost

  for (auto& [id, stalker] : this->StalkedReprojectionResiduals()) {
    summed_residuals += stalker->GetTrackedResidual();
    total_cost += 0.5 * stalker->GetTrackedResidual().dot(stalker->GetTrackedResidual());  // add square product
  };

  return total_cost;
}
}  // namespace fuhe

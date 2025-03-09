#include "fusion_helper/fusion_residuals_tracker.h"

namespace fuhe {

const std::shared_ptr<FusionResidualsTracker> FusionResidualsTracker::Create() { return std::make_shared<FusionResidualsTracker>(); }

void FusionResidualsTracker::RegisterStalkedOdomResidual(const std::shared_ptr<ResidualStalker<6>> odom_residual_stalker,
                                                         const std::string& edge_id) {
  if (!odom_residual_stalker) {
    LOG(WARNING) << "Tried to add nullptr instead of odom residual stalker to fusion residual tracker!";
    return;
  }
  this->stalked_odom_residuals[edge_id] = odom_residual_stalker;
};

void FusionResidualsTracker::RegisterStalkedReprojectionResidual(const std::shared_ptr<ResidualStalker<2>> reproj_residual_stalker,
                                                                 const std::string& img_id) {
  if (!reproj_residual_stalker) {
    LOG(WARNING) << "Tried to add nullptr instead of Reprojection residual stalker to fusion residual tracker!";
    return;
  }
  this->stalked_reprojection_residuals[img_id] = reproj_residual_stalker;
};

const std::map<std::string, std::shared_ptr<ResidualStalker<6>>>& FusionResidualsTracker::StalkedOdomResiduals() const {
  return stalked_odom_residuals;
}

const std::map<std::string, std::shared_ptr<ResidualStalker<2>>>& FusionResidualsTracker::StalkedReprojectionResiduals() const {
  return stalked_reprojection_residuals;
}
}  // namespace fuhe

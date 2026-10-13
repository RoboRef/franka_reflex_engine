#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct FrankaJointImpedanceControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};
using PandaJointImpedanceControllerPose = FrankaJointImpedanceControllerPose;

class ComlessFrankaJointImpedanceController :
    public virtual PandaControllerBase<double,
                                       FrankaJointImpedanceControllerPose> {
 public:
  virtual ~ComlessFrankaJointImpedanceController() = default;

 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                      const FrankaJointImpedanceControllerPose& desired,
                      const double& p) override final;
  double defaultParameters() override final;
  FrankaJointImpedanceControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
};

using ComlessPandaJointImpedanceController = ComlessFrankaJointImpedanceController;

}  // namespace panda_controllers

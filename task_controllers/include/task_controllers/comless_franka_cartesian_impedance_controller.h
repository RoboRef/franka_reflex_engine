#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct FrankaCartesianImpedanceControllerPose {
  Eigen::Matrix<double, 7, 1> q_n;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};
using PandaCartesianImpedanceControllerPose = FrankaCartesianImpedanceControllerPose;

struct FrankaCartesianImpedanceControllerParams {
  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 1> damping_ratio;
  double nullspace_stiffness;
};
using PandaCartesianImpedanceControllerParams = FrankaCartesianImpedanceControllerParams;

class ComlessFrankaCartesianImpedanceController :
    public virtual PandaControllerBase<FrankaCartesianImpedanceControllerParams,
                                       FrankaCartesianImpedanceControllerPose> {
 public:
  virtual ~ComlessFrankaCartesianImpedanceController() = default;

 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                      const FrankaCartesianImpedanceControllerPose& desired,
                      const FrankaCartesianImpedanceControllerParams& p) override final;
  FrankaCartesianImpedanceControllerParams defaultParameters() override final;
  FrankaCartesianImpedanceControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
};

using ComlessPandaCartesianImpedanceController = ComlessFrankaCartesianImpedanceController;

}  // namespace panda_controllers

#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct PandaJointImpedanceExampleControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

class PandaJointImpedanceExampleController :
    public virtual PandaControllerBase<double,
                                       PandaJointImpedanceExampleControllerPose> {
 public:
  virtual ~PandaJointImpedanceExampleController() = default;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const PandaJointImpedanceExampleControllerPose& desired,
      const double& p) override final;
  double defaultParameters() override final;
  PandaJointImpedanceExampleControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;

  // Track the start time for the sine wave
  double start_time_ = -1.0;
};

}

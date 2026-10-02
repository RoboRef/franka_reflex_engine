#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct PandaStopControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

class PandaStopController :
    public virtual PandaControllerBase<double,
                                       PandaStopControllerPose> {
 public:
  virtual ~PandaStopController() = default;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const PandaStopControllerPose& desired,
      const double& p) override final;
  double defaultParameters() override final;
  PandaStopControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
  void startImpl() override final;
  // Velocity target ramping to prevent power_limit_violation on activation
  int ramp_counter_ = 0;
  int ramp_duration_ = 100; // cycles (0.1s at 1kHz)
  Eigen::Matrix<double, 7, 1> qd_initial_;
};

}

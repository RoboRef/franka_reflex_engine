#pragma once

#include <Eigen/Dense>
// #include <franka_hw/franka_model_interface.h>
// #include <franka_hw/franka_state_interface.h>

#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

class PandaContactEvasionController :
    public virtual PandaControllerBase<Eigen::Matrix<double, 7, 7>,
                                       Eigen::Matrix<double, 7, 1>> {
 public:
  virtual ~PandaContactEvasionController() = default;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const Eigen::Matrix<double, 7, 1>& desired,
      const Eigen::Matrix<double, 7, 7>& p) override final;
  Eigen::Matrix<double, 7, 7> defaultParameters() override final;
  Eigen::Matrix<double, 7, 1> getCurrentPoseImpl() override final;
};

}

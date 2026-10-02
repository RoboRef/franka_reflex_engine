#pragma once

#include <Eigen/Dense>
// #include <franka_hw/franka_state_interface.h>

#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct PandaSuperZeroGControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

struct PandaSuperZeroGControllerParams {
  Eigen::Matrix<double, 7, 7> K_r;
};

class PandaSuperZeroGController :
    public PandaControllerBase<PandaSuperZeroGControllerParams,
                               PandaSuperZeroGControllerPose> {
 public:
  virtual ~PandaSuperZeroGController() = default;
 protected:
  virtual bool initImpl(const std::vector<RobotData*>& robot_data,
      rclcpp_lifecycle::LifecycleNode::SharedPtr& nh, std::string name, std::string resource) override;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const PandaSuperZeroGControllerPose& desired,
      const PandaSuperZeroGControllerParams& p) override final;
  PandaSuperZeroGControllerParams defaultParameters() override final;
  PandaSuperZeroGControllerPose getCurrentPoseImpl() override final;
  void startImpl() override final;
  std::array<bool, 7> moving_wall_;
  std::array<double, 7> wall_scale_;
  std::array<double, 7> lower_soft_limit_;
  std::array<double, 7> upper_soft_limit_;
};

}

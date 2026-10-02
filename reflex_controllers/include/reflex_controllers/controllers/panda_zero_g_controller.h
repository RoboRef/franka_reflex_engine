#pragma once

#include <Eigen/Dense>
// #include <franka_hw/franka_state_interface.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct PandaZeroGControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

struct PandaZeroGControllerParams {};

class PandaZeroGController :
    public PandaControllerBase<PandaZeroGControllerParams,
                               PandaZeroGControllerPose> {
 public:
  virtual ~PandaZeroGController() = default;
 protected:
  virtual bool initImpl(const std::vector<RobotData*>& robot_data,
      rclcpp_lifecycle::LifecycleNode::SharedPtr& nh, std::string name, std::string resource) override;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const PandaZeroGControllerPose& desired,
      const PandaZeroGControllerParams& p) override final;
  PandaZeroGControllerParams defaultParameters() override final;
  PandaZeroGControllerPose getCurrentPoseImpl() override final;
  void startImpl() override final;
  std::array<bool, 7> moving_wall_;
  std::array<double, 7> wall_scale_;
  std::array<double, 7> lower_soft_limit_;
  std::array<double, 7> upper_soft_limit_;
};

}

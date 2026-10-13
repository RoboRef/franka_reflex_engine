#pragma once

#include <Eigen/Dense>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct FrankaSuperZeroGControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

struct FrankaSuperZeroGControllerParams {
  Eigen::Matrix<double, 7, 7> K_r;
};

class FrankaSuperZeroGController :
    public PandaControllerBase<FrankaSuperZeroGControllerParams,
                               FrankaSuperZeroGControllerPose> {
 public:
  virtual ~FrankaSuperZeroGController() = default;
 protected:
  virtual bool initImpl(const std::vector<RobotData*>& robot_data,
      rclcpp_lifecycle::LifecycleNode::SharedPtr& nh, std::string name, std::string resource) override;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const FrankaSuperZeroGControllerPose& desired,
      const FrankaSuperZeroGControllerParams& p) override final;
  FrankaSuperZeroGControllerParams defaultParameters() override final;
  FrankaSuperZeroGControllerPose getCurrentPoseImpl() override final;
  void startImpl() override final;
  std::array<bool, 7> moving_wall_;
  std::array<double, 7> wall_scale_;
  std::array<double, 7> lower_soft_limit_;
  std::array<double, 7> upper_soft_limit_;
};

}

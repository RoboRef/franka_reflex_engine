#pragma once

#include <Eigen/Dense>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct FrankaZeroGControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

struct FrankaZeroGControllerParams {};

class FrankaZeroGController :
    public PandaControllerBase<FrankaZeroGControllerParams,
                               FrankaZeroGControllerPose> {
 public:
  virtual ~FrankaZeroGController() = default;
 protected:
  virtual bool initImpl(const std::vector<RobotData*>& robot_data,
      rclcpp_lifecycle::LifecycleNode::SharedPtr& nh, std::string name, std::string resource) override;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const FrankaZeroGControllerPose& desired,
      const FrankaZeroGControllerParams& p) override final;
  FrankaZeroGControllerParams defaultParameters() override final;
  FrankaZeroGControllerPose getCurrentPoseImpl() override final;
  void startImpl() override final;
  std::array<bool, 7> moving_wall_;
  std::array<double, 7> wall_scale_;
  std::array<double, 7> lower_soft_limit_;
  std::array<double, 7> upper_soft_limit_;
};

}

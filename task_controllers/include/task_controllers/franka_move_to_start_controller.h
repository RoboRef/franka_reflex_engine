#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>
#include <reflex_controllers/utils/motion_generator.h>
#include <memory>

namespace panda_controllers {

struct FrankaMoveToStartControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

class FrankaMoveToStartController :
    public virtual PandaControllerBase<double, FrankaMoveToStartControllerPose> {
 public:
  virtual ~FrankaMoveToStartController() = default;

 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                      const FrankaMoveToStartControllerPose& desired,
                      const double& p) override final;
  double defaultParameters() override final;
  FrankaMoveToStartControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
  
  bool initImpl(const std::vector<RobotData*>& robot_data,
                rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) override final;
  void startImpl() override final;
  void loadGoalPosition();

  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Time start_time_;
  Eigen::Matrix<double, 7, 1> q_goal_;
  Eigen::Matrix<double, 7, 1> dq_filtered_;
  Eigen::Matrix<double, 7, 1> k_gains_;
  Eigen::Matrix<double, 7, 1> d_gains_;
};

}  // namespace panda_controllers

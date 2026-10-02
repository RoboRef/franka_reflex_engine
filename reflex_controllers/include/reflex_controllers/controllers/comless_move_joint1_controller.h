#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_base.h>

namespace panda_controllers {

struct ComlessMoveJoint1ControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

struct ComlessMoveJoint1ControllerParams {
  Eigen::Matrix<double, 7, 1> stiffness;
  Eigen::Matrix<double, 7, 1> damping_ratio;
  
  // Custom parameters for the move joint1 trajectory
  double max_velocity;
  double acceleration;
  double distance;
};

class ComlessMoveJoint1Controller :
    public virtual PandaControllerBase<ComlessMoveJoint1ControllerParams,
                                       ComlessMoveJoint1ControllerPose> {
 public:
  virtual ~ComlessMoveJoint1Controller() = default;
 private:
  bool initImpl(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) override final;
  void startImpl() override final;
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const ComlessMoveJoint1ControllerPose& desired,
      const ComlessMoveJoint1ControllerParams& p) override final;
  ComlessMoveJoint1ControllerParams defaultParameters() override final;
  ComlessMoveJoint1ControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;

  // Trajectory state
  bool initialized_start_pose_ = false;
  double time_traj_ = 0.0;
  Eigen::Matrix<double, 7, 1> initial_q_;
};

}

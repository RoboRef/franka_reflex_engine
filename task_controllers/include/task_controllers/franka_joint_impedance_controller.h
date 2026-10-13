#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_ros_interface.h>
#include <task_controllers/comless_franka_joint_impedance_controller.h>
#include <multi_mode_control_msgs/srv/set_joint_impedance.hpp>
#include <multi_mode_control_msgs/msg/joint_goal.hpp>

namespace panda_controllers {
  using GoalMsg = multi_mode_control_msgs::msg::JointGoal;
  using ServiceParameter = multi_mode_control_msgs::srv::SetJointImpedance;
  using Pose = FrankaJointImpedanceControllerPose;
    
class FrankaJointImpedanceController :
    public virtual ComlessFrankaJointImpedanceController,
    public virtual ControllerRosInterface<ServiceParameter,
                                          GoalMsg,
                                          double, 
                                          Pose> {
 public:
  virtual ~FrankaJointImpedanceController() = default;

 private:
  bool desiredPoseCallbackImpl(Pose& p_d,
                               const Pose& p,
                               const GoalMsg& msg) override final;
  
  bool setParametersCallbackImpl(double& p_d, const double& p,
                                 const ServiceParameter::Request::SharedPtr& req, 
                                 const ServiceParameter::Response::SharedPtr& res) override final;
};
}  // namespace panda_controllers

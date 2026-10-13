#pragma once

#include <Eigen/Dense>
#include <reflex_controllers/base/panda_controller_ros_interface.h>
#include <task_controllers/comless_franka_cartesian_impedance_controller.h>
#include <multi_mode_control_msgs/srv/set_cartesian_impedance.hpp>
#include <multi_mode_control_msgs/msg/cartesian_impedance_goal.hpp>

namespace panda_controllers {
  using GoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
  using ServiceParameter = multi_mode_control_msgs::srv::SetCartesianImpedance;
  using Pose = FrankaCartesianImpedanceControllerPose;
  using Parameters = FrankaCartesianImpedanceControllerParams;

class FrankaCartesianImpedanceController :
    public virtual ComlessFrankaCartesianImpedanceController,
    public virtual ControllerRosInterface<ServiceParameter, 
                                          GoalMsg, 
                                          Parameters, 
                                          Pose> {
 public:
  virtual ~FrankaCartesianImpedanceController() = default;

 private:
  bool desiredPoseCallbackImpl(Pose& p_d,
                               const Pose& p,
                               const GoalMsg& msg) override final;

  bool setParametersCallbackImpl(Parameters& p_d, 
                                 const Parameters& p,
                                 const ServiceParameter::Request::SharedPtr& req, 
                                 const ServiceParameter::Response::SharedPtr& res) override final;
};
}  // namespace panda_controllers

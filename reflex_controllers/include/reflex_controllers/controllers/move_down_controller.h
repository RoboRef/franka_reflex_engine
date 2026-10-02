#pragma once

#include <Eigen/Dense>

#include <reflex_controllers/base/panda_controller_ros_interface.h>
#include <reflex_controllers/controllers/comless_move_down_controller.h>
#include <multi_mode_control_msgs/srv/set_cartesian_impedance.hpp>
#include <multi_mode_control_msgs/msg/cartesian_impedance_goal.hpp>

namespace panda_controllers {
    // We reuse the CartesianImpedance goal and parameter services just to have an interface,
    // though the controller actually ignores the goal messages since it computes trajectory internally.
    using GoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
    using ServiceParameter = multi_mode_control_msgs::srv::SetCartesianImpedance;
    using Pose = ComlessMoveDownControllerPose;
    using Parameters = ComlessMoveDownControllerParams;

class MoveDownController :
    public virtual ComlessMoveDownController,
    public virtual ControllerRosInterface<ServiceParameter, 
                                          GoalMsg, 
                                          Parameters, 
                                          Pose> {
public:
  virtual ~MoveDownController() = default;

private:
  bool desiredPoseCallbackImpl(Pose& p_d,
                               const Pose& p,
                               const GoalMsg& msg)
                               override final;

  bool setParametersCallbackImpl(Parameters& p_d, 
        const Parameters& p,
        const ServiceParameter::Request::SharedPtr& req, 
        const ServiceParameter::Response::SharedPtr& res) 
        override final;
};
}

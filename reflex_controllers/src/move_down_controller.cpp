#include <reflex_controllers/controllers/move_down_controller.h>
#include <reflex_controllers/utils/controller_factory.h>

using namespace panda_controllers;
using Pose = ComlessMoveDownControllerPose;
using Parameters = ComlessMoveDownControllerParams;
using Controller = MoveDownController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "move_down_controller");

bool Controller::desiredPoseCallbackImpl(Pose& p_d,
                                          const Pose& p,
                                          const GoalMsg& msg) {
  // Goal message is ignored because trajectory is computed internally
  return true;
}

bool Controller::setParametersCallbackImpl(Parameters& p_d,
    const Parameters& p,
    const ServiceParameter::Request::SharedPtr& request,
    const ServiceParameter::Response::SharedPtr& response) {
  
  if (!node_->has_parameter("move_down_controller.max_velocity")) {
    node_->declare_parameter("move_down_controller.max_velocity", 0.15);
  }
  if (!node_->has_parameter("move_down_controller.acceleration")) {
    node_->declare_parameter("move_down_controller.acceleration", 0.1);
  }
  if (!node_->has_parameter("move_down_controller.distance")) {
    node_->declare_parameter("move_down_controller.distance", 0.3);
  }
  
  // Set stiffness from the service
  p_d.stiffness.setZero();
  for (int i = 0; i < 6; ++i) {
    p_d.stiffness(i, i) = request->stiffness.data()[i];
  }
  p_d.nullspace_stiffness = request->nullspace_stiffness;

  // We read custom parameters in startImpl() instead

  RCLCPP_INFO_STREAM(node_->get_logger(), "MoveDownController Stiffness Parameters set");

  return true;
}

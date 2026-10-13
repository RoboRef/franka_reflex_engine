#include <task_controllers/wiping_controller.h>
#include <reflex_controllers/utils/controller_factory.h>

using namespace panda_controllers;
using Pose = PandaCartesianImpedanceControllerPose;
using Parameters = PandaCartesianImpedanceControllerParams;
using Controller = WipingController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "wiping_controller");

bool Controller::desiredPoseCallbackImpl(Pose& p_d,
                                         const Pose& /*p*/,
                                         const GoalMsg& msg) {
  p_d.position[0] = msg.pose.position.x;
  p_d.position[1] = msg.pose.position.y;
  p_d.position[2] = msg.pose.position.z;
  p_d.orientation.w() = msg.pose.orientation.w;
  p_d.orientation.x() = msg.pose.orientation.x;
  p_d.orientation.y() = msg.pose.orientation.y;
  p_d.orientation.z() = msg.pose.orientation.z;
  return true;
}

bool Controller::setParametersCallbackImpl(Parameters& p_d,
    const Parameters& /*p*/,
    const ServiceParameter::Request::SharedPtr& request,
    const ServiceParameter::Response::SharedPtr& /*response*/) {
  p_d.stiffness.setZero();
  for (int i = 0; i < 6; ++i) {
    p_d.stiffness(i, i) = request->stiffness.data()[i];
  }
  p_d.nullspace_stiffness = request->nullspace_stiffness;
  return true;
}

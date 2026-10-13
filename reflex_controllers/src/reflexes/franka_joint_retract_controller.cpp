#include <reflex_controllers/reflexes/franka_joint_retract_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <algorithm>
#include <cmath>

namespace panda_controllers {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = FrankaJointRetractControllerPose;
using Params = FrankaJointRetractControllerParams;
using Controller = FrankaJointRetractController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "franka_joint_retract_controller");

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& /*desired*/, const Params& /*p*/) {
  Pose current = getCurrentPose();

  static Vector7d stiffness((std::vector<double>(
      {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0})).data());
  static Vector7d damping((std::vector<double>(
      {30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0})).data());

  Vector7d coriolis(robot_data_[0]->coriolis().data());
  Vector7d tau_cmd = coriolis + stiffness.cwiseProduct(q_goal_ - current.q) - damping.cwiseProduct(current.qD);

  walls_.applyWalls(current.q, current.qD, tau_cmd, *tau[0]);
}

Params Controller::defaultParameters() {
  Params p;
  p.max_retraction = 0.05;
  return p;
}

Pose Controller::getCurrentPoseImpl() {
  Pose current;
  current.q = Vector7d(robot_data_[0]->state().q.data());
  current.qD = Vector7d(robot_data_[0]->state().dq.data());
  return current;
}

bool Controller::hasOffsetImpl() {
  return false;
}

void Controller::resetOffset() {}

void Controller::startImpl() {
  walls_.reset();
  q_start_ = Vector7d(robot_data_[0]->state().q.data());
  Vector7d tau_ext(robot_data_[0]->state().tau_ext_hat_filtered.data());

  double tau_norm = tau_ext.norm();
  Vector7d delta = Vector7d::Zero();
  if (tau_norm > 1e-3) {
    // Retract away from the direction of external impact torque
    delta = -(tau_ext / tau_norm) * defaultParameters().max_retraction;
  } else {
    // If external torque magnitude is very small, retract in direction opposite to current velocity
    Vector7d dq(robot_data_[0]->state().dq.data());
    if (dq.norm() > 1e-3) {
      delta = -(dq / dq.norm()) * defaultParameters().max_retraction;
    }
  }

  // Clip to joint limits with buffer
  static const Vector7d q_min = (Vector7d() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished();
  static const Vector7d q_max = (Vector7d() <<  2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973).finished();
  const double buffer = 0.15;

  q_goal_ = q_start_ + delta;
  for (int i = 0; i < 7; ++i) {
    q_goal_[i] = std::clamp(q_goal_[i], q_min[i] + buffer, q_max[i] - buffer);
  }
}

}  // namespace panda_controllers

#include <reflex_controllers/reflexes/franka_admittance_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <algorithm>

namespace panda_controllers {

using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = FrankaAdmittanceControllerPose;
using Params = FrankaAdmittanceControllerParams;
using Controller = FrankaAdmittanceController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "franka_admittance_controller");

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& /*desired*/, const Params& p) {
  Pose current = getCurrentPose();
  Vector7d tau_ext(robot_data_[0]->state().tau_ext_hat_filtered.data());

  // Deadband on external torque to avoid drifting due to sensor bias/noise
  Vector7d tau_ext_active = Vector7d::Zero();
  for (int i = 0; i < 7; ++i) {
    if (std::abs(tau_ext[i]) > p.deadband) {
      double sign = (tau_ext[i] > 0.0) ? 1.0 : -1.0;
      tau_ext_active[i] = tau_ext[i] - sign * p.deadband;
    }
  }

  // Control law: q_dot_d(t) = alpha * tau_ext(t)
  // Integrating with dt = 0.001 (1kHz loop rate):
  // Note on libfranka conventions: tau_ext_hat_filtered is external torque applied on the robot.
  // In libfranka / multipanda compliance, subtracting yields in direction of the force:
  // q_d += (-p.alpha * tau_ext_active) * dt
  const double dt = 0.001;
  q_d_ -= p.alpha * tau_ext_active * dt;

  // Stiff joint impedance tracking of q_d_ with damping
  static Vector7d stiffness((std::vector<double>(
      {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0})).data());
  static Vector7d damping((std::vector<double>(
      {30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0})).data());

  Vector7d coriolis(robot_data_[0]->coriolis().data());
  Vector7d tau_cmd = coriolis + stiffness.cwiseProduct(q_d_ - current.q) - damping.cwiseProduct(current.qD);

  // Apply joint limit walls
  walls_.applyWalls(current.q, current.qD, tau_cmd, *tau[0]);
}

Params Controller::defaultParameters() {
  Params p;
  p.alpha = 0.05;
  p.deadband = 1.0;
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
  q_d_ = Vector7d(robot_data_[0]->state().q.data());
}

}  // namespace panda_controllers

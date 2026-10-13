#include <reflex_controllers/reflexes/franka_stop_controller.h>

#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>
#include <algorithm>

using namespace panda_controllers;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = FrankaStopControllerPose;
using Controller = FrankaStopController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "franka_stop_controller");

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const double& p) {
  Pose current = getCurrentPose();
  current.q -= getOffset().q;
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());

  // Spring-damper stop: stiffness holds at activation position, damping decelerates
  // We use very high stiffness for a "stiff catch"
  static Vector7d stiffness(std::vector<double>(
      {200.0, 200.0, 200.0, 200.0, 100.0, 100.0, 50.0}).data());
  static Matrix7d damping_ratio = Matrix7d::Identity() * 0.8; // standard damping
  Matrix7d designed_damping = sqrtDesign<7>(inertia,
      Matrix7d(stiffness.asDiagonal()), damping_ratio.diagonal());

  // Ramp the target velocity from initial velocity to zero
  // This completely eliminates the torque spike at activation (since initial error is 0)
  // while keeping feedback gains constant (avoids phase lag/noise).
  double t = std::min(1.0, static_cast<double>(ramp_counter_) / ramp_duration_);
  ++ramp_counter_;
  Vector7d qd_target = (1.0 - t) * qd_initial_;

  // Stiff impedance torque
  auto tau_d = coriolis + stiffness.asDiagonal() * (desired.q - current.q)
               + designed_damping * (qd_target - current.qD);
               
  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}

double Controller::defaultParameters() {
  return 1.0;
}

Pose Controller::getCurrentPoseImpl() {
  Pose current;
  current.q = Vector7d(robot_data_[0]->state().q.data());
  current.qD = Vector7d(robot_data_[0]->state().dq.data());
  return current;
}

bool Controller::hasOffsetImpl() {
  return getOffset().q.norm() != 0;
}

void Controller::resetOffset() {
  Pose p;
  p.q = Vector7d::Zero();
  p.qD = Vector7d::Zero();
  setOffset(p);
}

void Controller::startImpl() {
  ramp_counter_ = 0;
  // Capture initial velocity to ramp it down smoothly
  qd_initial_ = Vector7d(robot_data_[0]->state().dq.data());
}

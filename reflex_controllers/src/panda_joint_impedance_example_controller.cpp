#include <reflex_controllers/controllers/panda_joint_impedance_example_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>

using namespace panda_controllers;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = PandaJointImpedanceExampleControllerPose;
using Controller = PandaJointImpedanceExampleController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "panda_joint_impedance_example_controller");

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const double& p) {
  
  if (start_time_ < 0) {
      start_time_ = node_->now().seconds();
  }
  
  Pose current = getCurrentPose();
  current.q -= getOffset().q;
  
  Pose dynamic_desired = desired;
  
  // Sine wave trajectory parameters (matching franka_example_controllers)
  double time = node_->now().seconds() - start_time_;
  double radius = M_PI / 8.0;
  double period = 5.0; // 2 * PI / 5.0 = PI / 2.5
  double delta_q = radius * (1.0 - std::cos(2.0 * M_PI * time / period));
  double delta_qD = radius * (2.0 * M_PI / period) * std::sin(2.0 * M_PI * time / period);

  // Apply to joint 4 and 5
  dynamic_desired.q[3] += delta_q;
  dynamic_desired.q[4] += delta_q;
  dynamic_desired.qD[3] = delta_qD;
  dynamic_desired.qD[4] = delta_qD;
  
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Matrix7d stiffness(Vector7d(std::vector<double>(
      {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}).data()).asDiagonal());
  Matrix7d damping(Vector7d(std::vector<double>(
      {30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0}).data()).asDiagonal());
  Matrix7d designed_damping = sqrtDesign<7>(
      Eigen::Map<const Matrix7d>(robot_data_[0]->mass().data()), stiffness,
      damping.diagonal());
  auto tau_d = coriolis + p*stiffness*(dynamic_desired.q - current.q) +
               designed_damping*(dynamic_desired.qD - current.qD);
  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}

double Controller::defaultParameters() {
  return 1.0;
}

Pose Controller::getCurrentPoseImpl() {
  Pose p;
  Eigen::Map<const Vector7d> q(robot_data_[0]->state().q.data());
  Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());
  p.q = q;
  p.qD = qD;
  return p;
}

bool Controller::hasOffsetImpl() {
  return getOffset().q.norm() != 0;
}

void Controller::resetOffset() {
  Pose p;
  p.q = Vector7d::Zero();
  p.qD = Vector7d::Zero();
  setOffset(p);
  start_time_ = -1.0; // Reset timer when offset is reset (e.g. on controller start)
}

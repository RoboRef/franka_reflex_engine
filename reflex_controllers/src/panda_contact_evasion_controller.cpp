#include <reflex_controllers/controllers/panda_contact_evasion_controller.h>

#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>

using namespace panda_controllers;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = Vector7d;
using Controller = PandaContactEvasionController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "panda_contact_evasion_controller");

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const Matrix7d& p) {
  Pose current = getCurrentPose();
  setOffset(getOffset() - p*Eigen::Map<const Vector7d>(
      robot_data_[0]->state().tau_ext_hat_filtered.data())*0.001);
  Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());
  static Matrix7d damping(Vector7d(std::vector<double>(
      {30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}).data()).asDiagonal());
  static Matrix7d stiffness(Vector7d(std::vector<double>(
      {1200.0, 1200.0, 1200.0, 1200.0, 500.0, 300.0, 100.0}
      ).data()).asDiagonal());
  auto tau_d = stiffness*(desired + getOffset() - current) - damping*qD;
  for (uint64_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}

Matrix7d Controller::defaultParameters() {
  return Matrix7d::Identity()*0.05;
}

Pose Controller::getCurrentPoseImpl() {
  return Vector7d(robot_data_[0]->state().q.data());
}

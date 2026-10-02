#include <reflex_controllers/controllers/panda_cartesian_impedance_example_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>
#include <reflex_controllers/utils/nullspace_projection.h>

using namespace panda_controllers;
using Eigen::Vector3d;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Eigen::Quaterniond;
using Pose = PandaCartesianImpedanceExampleControllerPose;
using Params = PandaCartesianImpedanceExampleControllerParams;
using Controller = PandaCartesianImpedanceExampleController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "panda_cartesian_impedance_example_controller");

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const Params& p) {
    
  if (start_time_ < 0) {
      start_time_ = node_->now().seconds();
  }
  
  Pose current = getCurrentPose();
  current.position -= getOffset().position;
  
  Pose dynamic_desired = desired;
  
  // Sine wave trajectory parameters in Cartesian space
  double time = node_->now().seconds() - start_time_;
  double radius = 0.15; // 15 cm radius
  double period = 5.0; // seconds
  
  double angle = (2.0 * M_PI * time) / period;
  // Create circular motion in X and Y, centered at initial desired.position
  dynamic_desired.position[0] += radius * std::sin(angle);
  dynamic_desired.position[1] += radius * (1.0 - std::cos(angle)); 

  Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
      robot_data_[0]->eeZeroJacobian().data());
  Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());
  Vector6d error;
  error.head(3) << current.position - dynamic_desired.position;
  if (dynamic_desired.orientation.coeffs().dot(current.orientation.coeffs()) < 0.0) {
    current.orientation.coeffs() << -current.orientation.coeffs();
  }
  Eigen::Quaterniond rot_error(
      current.orientation * dynamic_desired.orientation.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
  Vector7d tau_task, tau_nullspace, tau_d;
  Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia),
      p.stiffness, p.damping_ratio);
  tau_task << jacobian.transpose() * (-p.stiffness*error - D*(jacobian*qD));
  tau_nullspace <<
      getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
      (p.nullspace_stiffness * (dynamic_desired.q_n - current.q_n) -
       (2.0 * std::sqrt(p.nullspace_stiffness)) * qD);
  tau_d << tau_task + tau_nullspace + coriolis;
  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}

Params Controller::defaultParameters() {
  Params p;
  p.stiffness.setIdentity();
  p.stiffness.topLeftCorner(3, 3) << 400 * Eigen::Matrix3d::Identity();
  p.stiffness.bottomRightCorner(3, 3) << 20 * Eigen::Matrix3d::Identity();
  p.damping_ratio = Vector6d::Constant(0.8);
  p.nullspace_stiffness = 10;
  return p;
}

Pose Controller::getCurrentPoseImpl() {
  Pose p;
  Eigen::Map<const Vector7d> q(robot_data_[0]->state().q.data());
  Eigen::Affine3d transform(
      Eigen::Matrix4d::Map(robot_data_[0]->state().O_T_EE.data()));
  p.position = transform.translation();
  p.orientation = Eigen::Quaterniond(transform.linear());
  p.q_n = q;
  return p;
}

bool Controller::hasOffsetImpl() {
  return getOffset().position.norm() != 0;
}

void Controller::resetOffset() {
  Pose p;
  p.position = Vector3d::Zero();
  p.orientation = Quaterniond(1, 0, 0, 0);
  p.q_n = Vector7d::Zero();
  setOffset(p);
  start_time_ = -1.0;
}

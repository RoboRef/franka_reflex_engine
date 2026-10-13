#include <reflex_controllers/reflexes/franka_cartesian_retract_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>
#include <reflex_controllers/utils/nullspace_projection.h>
#include <algorithm>
#include <cmath>

namespace panda_controllers {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Pose = FrankaCartesianRetractControllerPose;
using Params = FrankaCartesianRetractControllerParams;
using Controller = FrankaCartesianRetractController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "franka_cartesian_retract_controller");

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& /*desired*/, const Params& /*p*/) {
  Pose current = getCurrentPose();
  Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(robot_data_[0]->eeZeroJacobian().data());
  Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());

  Vector6d error;
  error.head(3) = current.position - p_goal_;

  // Maintain initial orientation
  if (q_orientation_.coeffs().dot(current.orientation.coeffs()) < 0.0) {
    current.orientation.coeffs() << -current.orientation.coeffs();
  }
  Eigen::Quaterniond rot_error(current.orientation * q_orientation_.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) = rot_error_aa.axis() * rot_error_aa.angle();

  Vector6d x_dot;
  x_dot.head(3) = jacobian.topRows(3) * qD;
  x_dot.tail(3) = jacobian.bottomRows(3) * qD;

  Matrix6d stiffness = Matrix6d::Identity();
  stiffness.topLeftCorner(3, 3) = 600.0 * Eigen::Matrix3d::Identity();
  stiffness.bottomRightCorner(3, 3) = 40.0 * Eigen::Matrix3d::Identity();
  Vector6d damping_ratio = Vector6d::Constant(0.8);

  Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia), stiffness, damping_ratio);

  Vector7d tau_task = jacobian.transpose() * (-stiffness * error - D * x_dot);
  double nullspace_stiffness = 10.0;
  Vector7d tau_nullspace = getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
      (nullspace_stiffness * (q_start_ - current.q_n) - (2.0 * std::sqrt(nullspace_stiffness)) * qD);

  Vector7d tau_cmd = tau_task + tau_nullspace + coriolis;
  walls_.applyWalls(current.q_n, qD, tau_cmd, *tau[0]);
}

Params Controller::defaultParameters() {
  Params p;
  p.retraction_distance = 0.09;
  return p;
}

Pose Controller::getCurrentPoseImpl() {
  Pose p;
  Eigen::Map<const Vector7d> q(robot_data_[0]->state().q.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_data_[0]->state().O_T_EE.data()));
  p.position = transform.translation();
  p.orientation = Eigen::Quaterniond(transform.linear());
  p.q_n = q;
  return p;
}

bool Controller::hasOffsetImpl() {
  return false;
}

void Controller::resetOffset() {}

void Controller::startImpl() {
  walls_.reset();
  Pose current = getCurrentPose();
  p_start_ = current.position;
  q_orientation_ = current.orientation;
  q_start_ = current.q_n;

  // Contact force at EE: O_F_ext_hat_K (estimated external wrench in base frame)
  Eigen::Matrix<double, 6, 1> F_ext(robot_data_[0]->state().O_F_ext_hat_K.data());
  Eigen::Vector3d force = F_ext.head(3);

  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  if (force.norm() > 2.0) {
    // Retract away from the direction of external impact force
    delta = -(force / force.norm()) * defaultParameters().retraction_distance;
  } 
  // Potential alternative retraction strategy based on velocity
  // else {
    // If EE force is not distinct, check end-effector translational velocity
    // Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(robot_data_[0]->eeZeroJacobian().data());
    // Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());
    // Eigen::Vector3d v_ee = jacobian.topRows(3) * qD;
    // if (v_ee.norm() > 0.01) {
    //   delta = -(v_ee / v_ee.norm()) * defaultParameters().retraction_distance;
    // } else {
    //   // Default upward retraction along +Z
    //   delta = Eigen::Vector3d(0.0, 0.0, defaultParameters().retraction_distance);
    // }
  //}

  p_goal_ = p_start_ + delta;
}

}  // namespace panda_controllers

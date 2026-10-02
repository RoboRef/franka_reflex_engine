#include <reflex_controllers/controllers/comless_move_down_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>
#include <reflex_controllers/utils/nullspace_projection.h>
#include <cmath>

using namespace panda_controllers;
using Eigen::Vector3d;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Eigen::Quaterniond;
using Pose = ComlessMoveDownControllerPose;
using Params = ComlessMoveDownControllerParams;
using Controller = ComlessMoveDownController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "comless_move_down_controller");

void Controller::startImpl() {
  initialized_start_pose_ = false;
  time_traj_ = 0.0;
  Params p_curr = getParametersBuffered();
  if (node_->has_parameter("move_down_controller.max_velocity")) {
    node_->get_parameter("move_down_controller.max_velocity", p_curr.max_velocity);
  }
  if (node_->has_parameter("move_down_controller.acceleration")) {
    node_->get_parameter("move_down_controller.acceleration", p_curr.acceleration);
  }
  if (node_->has_parameter("move_down_controller.distance")) {
    node_->get_parameter("move_down_controller.distance", p_curr.distance);
  }
  
  double rot_stiff = 0.0;
  if (node_->has_parameter("move_down_controller.orientation_stiffness")) {
    node_->get_parameter("move_down_controller.orientation_stiffness", rot_stiff);
  }
  p_curr.stiffness.bottomRightCorner(3, 3) << rot_stiff * Eigen::Matrix3d::Identity();
  
  setParametersBuffered(p_curr);
}

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const Params& p) {
  Pose current = getCurrentPose();
  current.position -= getOffset().position;

  if (!initialized_start_pose_) {
    initial_position_ = current.position;
    initial_orientation_ = current.orientation;
    time_traj_ = 0.0;
    initialized_start_pose_ = true;
  }

  double v_max = p.max_velocity;
  double accel = p.acceleration;
  double distance = p.distance;

  double t_r = v_max / accel;
  double d_r = 0.5 * accel * (t_r * t_r);
  
  double t_c = 0.0;
  double d_c = 0.0;
  if (1.5 * d_r > distance) {
    d_r = distance / 2.0;
    t_r = std::sqrt(distance / accel);
    v_max = accel * t_r;
  } else {
    d_c = distance - 2 * d_r;
    t_c = d_c / v_max;
  }
  double T = 2 * t_r + t_c;

  time_traj_ += 0.001; // 1kHz loop

  double z_offset = 0.0;
  if (time_traj_ <= t_r) {
    z_offset = 0.5 * accel * (time_traj_ * time_traj_);
  } else if (time_traj_ <= t_r + t_c) {
    z_offset = d_r + v_max * (time_traj_ - t_r);
  } else if (time_traj_ <= T) {
    double t_dec = time_traj_ - (t_r + t_c);
    z_offset = d_r + d_c + (v_max * t_dec) - (0.5 * accel * (t_dec * t_dec));
  } else {
    z_offset = distance;
  }

  Pose current_desired;
  current_desired.position = initial_position_;
  current_desired.position.z() -= z_offset;
  current_desired.orientation = initial_orientation_;

  // Use the desired.q_n if provided, otherwise hold current nullspace
  current_desired.q_n = desired.q_n;

  Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
      robot_data_[0]->eeZeroJacobian().data());
  Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());
  
  Vector6d error;
  error.head(3) << current.position - current_desired.position;
  if (current_desired.orientation.coeffs().dot(current.orientation.coeffs()) < 0.0) {
    current.orientation.coeffs() << -current.orientation.coeffs();
  }
  Eigen::Quaterniond rot_error(
      current.orientation * current_desired.orientation.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
  
  Vector7d tau_task, tau_nullspace, tau_d;
  Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia),
      p.stiffness, p.damping_ratio);
  tau_task << jacobian.transpose() * (-p.stiffness*error - D*(jacobian*qD));
  tau_nullspace <<
      getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
      (p.nullspace_stiffness * (current_desired.q_n - current.q_n) -
       (2.0 * std::sqrt(p.nullspace_stiffness)) * qD);
  tau_d << tau_task + tau_nullspace + coriolis;
  
  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}

Params Controller::defaultParameters() {
  Params p;
  p.stiffness.setIdentity();
  p.stiffness.topLeftCorner(3, 3) << 800 * Eigen::Matrix3d::Identity();
  p.stiffness.bottomRightCorner(3, 3) << 600.0 * Eigen::Matrix3d::Identity();
  p.damping_ratio = Vector6d::Constant(0.8);
  p.nullspace_stiffness = 10;
  p.max_velocity = 0.15;
  p.acceleration = 0.1;
  p.distance = 0.3;
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
}

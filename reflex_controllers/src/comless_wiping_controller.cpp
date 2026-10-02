#include <reflex_controllers/controllers/comless_wiping_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>
#include <reflex_controllers/utils/nullspace_projection.h>
#include <cmath>
#include <algorithm>

using namespace panda_controllers;
using Eigen::Vector3d;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Eigen::Quaterniond;
using Pose = PandaCartesianImpedanceControllerPose;
using Params = PandaCartesianImpedanceControllerParams;
using Controller = ComlessWipingController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "comless_wiping_controller");

void Controller::startImpl() {
  initialized_start_pose_ = false;
  time_traj_ = 0.0;
  f_error_integral_z_ = 0.0;
  current_z_force_ = 0.0;
  initial_f_ext_hat_.setZero();
  initial_tau_ext_.setZero();
}

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const Params& p) {
  Pose current = getCurrentPose();
  current.position -= getOffset().position;

  if (!initialized_start_pose_) {
    initial_position_ = current.position;
    initial_orientation_ = current.orientation;
    initial_f_ext_hat_ = Eigen::Map<const Vector6d>(robot_data_[0]->state().O_F_ext_hat_K.data());
    initial_tau_ext_ = Eigen::Map<const Vector7d>(robot_data_[0]->state().tau_ext_hat_filtered.data());
    time_traj_ = 0.0;
    f_error_integral_z_ = 0.0;
    current_z_force_ = 0.0;
    initialized_start_pose_ = true;
  }

  time_traj_ += 0.001; // 1 kHz control loop step

  // 2D Raster Scan Surface Wiping Trajectory:
  // - Y-axis: Multiple back-and-forth wiping passes with velocity shaping
  // - X-axis: One-way progression along -X (0 -> pi over T_total, does not return to start)
  double Lx = 0.20;       // 20 cm progression along -X direction
  double Ly = 0.40;       // 35 cm primary wiping stroke along Y direction
  double T_total = 15.0;  // 15 seconds total duration

  double num_passes = 6.0;
  double omega_y = num_passes * (2.0 * M_PI / T_total);

  // X-axis one-way progression: phase theta_x spans 0 -> pi over T_total (monotonically moves from 0 to -Lx)
  double omega_x_base = M_PI / T_total;
  double theta_x = std::min(M_PI, omega_x_base * time_traj_);
  double omega_x = (time_traj_ <= T_total) ? omega_x_base : 0.0;

  // Harmonic trajectory shaping for Y-motion (gamma = 0.35)
  // Slows down turnaround velocity near Y direction changes by ~70% and speeds up middle stroke by +35%
  double gamma = 0.35;
  double cy = std::cos(omega_y * time_traj_);
  double sy = std::sin(omega_y * time_traj_);

  double u_shaped_y = (1.0 + gamma) * cy - gamma * cy * cy * cy;
  double du_dt_y = -omega_y * sy * ((1.0 + gamma) - 3.0 * gamma * cy * cy);
  double d2u_dt2_y = -omega_y * omega_y * cy * ((1.0 + gamma) - 3.0 * gamma * cy * cy + 6.0 * gamma * sy * sy);

  Vector3d p_des = initial_position_;
  p_des[0] -= 0.5 * Lx * (1.0 - std::cos(theta_x)); // One-way progression in -X direction
  p_des[1] -= 0.5 * Ly * (1.0 - u_shaped_y); // Primary wiping passes along Y direction
  p_des[2] -= 0.005; // 5 mm virtual penetration bias to maintain continuous contact

  Vector6d dx_des = Vector6d::Zero();
  dx_des[0] = -0.5 * Lx * omega_x * std::sin(theta_x);
  dx_des[1] = 0.5 * Ly * du_dt_y;

  Vector6d ddx_des = Vector6d::Zero();
  ddx_des[0] = -0.5 * Lx * omega_x * omega_x * std::cos(theta_x);
  ddx_des[1] = 0.5 * Ly * d2u_dt2_y;

  Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
      robot_data_[0]->eeZeroJacobian().data());
  Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());

  // Compute Cartesian pose error
  Vector6d error;
  error.head(3) << current.position - p_des;

  Quaterniond des_orient = initial_orientation_;
  if (des_orient.coeffs().dot(current.orientation.coeffs()) < 0.0) {
    current.orientation.coeffs() << -current.orientation.coeffs();
  }
  Quaterniond rot_error(current.orientation * des_orient.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();

  // Keep a small Z stiffness (100 N/m) and virtual penetration bias to guarantee continuous surface contact
  Params p_wiping = p;
  p_wiping.stiffness(2, 2) = 100.0;
  p_wiping.damping_ratio = Vector6d::Constant(1.0);

  Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia),
      p_wiping.stiffness, p_wiping.damping_ratio);

  // Cartesian impedance force output
  Vector6d f_imp = -p_wiping.stiffness * error - D * (jacobian * qD - dx_des);

  // --- Z Force Control Loop (Max 5.0 N downward force) ---
  double target_force_z = -8.0; // -5.0 N downward force into table
  double filter_gain = 0.001;
  current_z_force_ = filter_gain * target_force_z + (1.0 - filter_gain) * current_z_force_;

  // Read estimated external wrench from Franka robot state & zero out recorded start offset to eliminate sensor drift
  Eigen::Map<const Vector6d> f_ext_hat_raw(robot_data_[0]->state().O_F_ext_hat_K.data());
  Vector6d f_ext_hat = f_ext_hat_raw - initial_f_ext_hat_;
  double raw_z_force = f_ext_hat[2];
  double alpha_f = 0.05; // Low-pass filter smoothing coefficient
  double measured_z_force = alpha_f * raw_z_force + (1.0 - alpha_f) * current_z_force_;

  double f_err_z = current_z_force_ - measured_z_force;
  f_error_integral_z_ += 0.001 * f_err_z;
  f_error_integral_z_ = std::max(-5.0, std::min(5.0, f_error_integral_z_));

  // Reduced k_p to 0.15 for smoother force feedback without overshoot
  double k_p = 0.2;
  double k_i = 0.05;
  double f_cmd_z = current_z_force_ + k_p * f_err_z + k_i * f_error_integral_z_;

  // Clamp Z force command magnitude strictly
  f_cmd_z = std::max(-10.0, std::min(0.0, f_cmd_z));

  // Override Z component in f_imp with the force controller output
  f_imp[2] = f_cmd_z;

  Vector7d tau_task, tau_nullspace, tau_d;
  tau_task << jacobian.transpose() * f_imp;
  tau_nullspace <<
      getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
      (p_wiping.nullspace_stiffness * (desired.q_n - current.q_n) -
       (2.0 * std::sqrt(p_wiping.nullspace_stiffness)) * qD);

  tau_d << tau_task + tau_nullspace + coriolis;

  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}

Params Controller::defaultParameters() {
  Params p;
  p.stiffness.setIdentity();
  p.stiffness.topLeftCorner(3, 3) << 200 * Eigen::Matrix3d::Identity();
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
}

#include <reflex_mc_controller_impl/standard_wiping_controller.h>

#include <cassert>
#include <cmath>
#include <algorithm>
#include <exception>
#include <string>
#include <franka/model.h>
#include "pluginlib/class_list_macros.hpp"

namespace reflex_mc_controller {

controller_interface::InterfaceConfiguration
StandardWipingController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
StandardWipingController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

CallbackReturn StandardWipingController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during StandardWipingController init stage: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardWipingController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model", arm_id_));
  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardWipingController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  Matrix4d current_pose(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  initial_position_ = Vector3d(current_pose.block<3,1>(0,3));
  initial_orientation_ = Quaterniond(current_pose.block<3,3>(0,0));
  initial_qn_ = Vector7d(franka_robot_model_->getRobotState()->q.data());

  initial_f_ext_hat_ = Vector6d(franka_robot_model_->getRobotState()->O_F_ext_hat_K.data());
  initial_tau_ext_ = Vector7d(franka_robot_model_->getRobotState()->tau_ext_hat_filtered.data());

  time_traj_ = 0.0;
  f_error_integral_z_ = 0.0;
  current_z_force_ = 0.0;
  initialized_start_pose_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardWipingController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type StandardWipingController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  double dt = period.seconds();
  if (dt <= 0.0) dt = 0.001;
  time_traj_ += dt;

  Matrix4d current_pose(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  Vector3d current_position(current_pose.block<3,1>(0,3));
  Quaterniond current_orientation(current_pose.block<3,3>(0,0));

  Matrix7d inertia(franka_robot_model_->getMassMatrix().data());
  Vector7d coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());
  Vector7d qD(franka_robot_model_->getRobotState()->dq.data());
  Vector7d q(franka_robot_model_->getRobotState()->q.data());

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

  // Compute Cartesian pose error
  Vector6d error;
  error.head(3) << current_position - p_des;

  Quaterniond des_orient = initial_orientation_;
  if (des_orient.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() << -current_orientation.coeffs();
  }
  Quaterniond rot_error(current_orientation * des_orient.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();

  // Cartesian Stiffness & Damping matrices
  Matrix6d K = Matrix6d::Identity();
  K.topLeftCorner(3, 3) << 200.0 * Eigen::Matrix3d::Identity();
  K.bottomRightCorner(3, 3) << 20.0 * Eigen::Matrix3d::Identity();
  K(2, 2) = 100.0; // 100 N/m Z stiffness

  Vector6d damping_ratio = Vector6d::Constant(1.0);
  Matrix6d D = panda_controllers::sqrtDesign<6>(
      panda_controllers::pandaCartesianInertia(jacobian, inertia), K, damping_ratio);

  Vector6d f_imp = -K * error - D * (jacobian * qD - dx_des);

  // --- Z Force Control Loop (Max 5.0 N downward force) ---
  double target_force_z = -5.0; // -5.0 N downward force
  double filter_gain = 0.001;
  current_z_force_ = filter_gain * target_force_z + (1.0 - filter_gain) * current_z_force_;

  // Read estimated external wrench from Franka robot state & zero out recorded start offset
  Vector6d f_ext_hat_raw(franka_robot_model_->getRobotState()->O_F_ext_hat_K.data());
  Vector6d f_ext_hat = f_ext_hat_raw - initial_f_ext_hat_;
  double raw_z_force = f_ext_hat[2];
  double alpha_f = 0.05;
  double measured_z_force = alpha_f * raw_z_force + (1.0 - alpha_f) * current_z_force_;

  double f_err_z = current_z_force_ - measured_z_force;
  f_error_integral_z_ += dt * f_err_z;
  f_error_integral_z_ = std::max(-5.0, std::min(5.0, f_error_integral_z_));

  double k_p = 0.2;
  double k_i = 0.05;
  double f_cmd_z = current_z_force_ + k_p * f_err_z + k_i * f_error_integral_z_;
  f_cmd_z = std::max(-10.0, std::min(0.0, f_cmd_z));

  f_imp[2] = f_cmd_z;

  // Torque computation
  Vector7d tau_task, tau_nullspace, tau_d;
  tau_task << jacobian.transpose() * f_imp;

  double nullspace_stiffness = 10.0;
  tau_nullspace << panda_controllers::getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
                       (nullspace_stiffness * (initial_qn_ - q) -
                        (2.0 * std::sqrt(nullspace_stiffness)) * qD);

  tau_d << tau_task + tau_nullspace + coriolis;

  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

}  // namespace reflex_mc_controller

PLUGINLIB_EXPORT_CLASS(reflex_mc_controller::StandardWipingController,
                       controller_interface::ControllerInterface)

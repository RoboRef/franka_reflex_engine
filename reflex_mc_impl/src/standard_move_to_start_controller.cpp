#include <reflex_mc_controller_impl/standard_move_to_start_controller.h>
#include <reflex_controllers/utils/damping_design.h>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace reflex_mc_controller {

controller_interface::InterfaceConfiguration
StandardMoveToStartController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
StandardMoveToStartController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

CallbackReturn StandardMoveToStartController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>("q_goal", {});
    auto_declare<std::vector<double>>("recovery_goal_position", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during StandardMoveToStartController init stage: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardMoveToStartController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model", arm_id_));

  std::vector<double> default_goal = {0.0218849, 0.0737021, -0.0380198, -1.63171, 0.0606058, 1.78036, 0.800907};
  std::vector<double> q_goal_param;

  if (get_node()->has_parameter("q_goal") && !get_node()->get_parameter("q_goal").as_double_array().empty()) {
    q_goal_param = get_node()->get_parameter("q_goal").as_double_array();
  } else if (get_node()->has_parameter("recovery_goal_position") && !get_node()->get_parameter("recovery_goal_position").as_double_array().empty()) {
    q_goal_param = get_node()->get_parameter("recovery_goal_position").as_double_array();
  }

  if (q_goal_param.size() == static_cast<uint>(num_joints_)) {
    for (int i = 0; i < num_joints_; ++i) q_goal_(i) = q_goal_param.at(i);
  } else {
    for (int i = 0; i < num_joints_; ++i) q_goal_(i) = default_goal.at(i);
  }

  std::vector<double> default_k = {300.0, 300.0, 300.0, 300.0, 100.0, 50.0, 50.0};
  std::vector<double> default_d = {20.0, 20.0, 20.0, 10.0, 10.0, 10.0, 5.0};

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  if (k_gains.size() == static_cast<uint>(num_joints_)) {
    for (int i = 0; i < num_joints_; ++i) k_gains_(i) = k_gains.at(i);
  } else {
    for (int i = 0; i < num_joints_; ++i) k_gains_(i) = default_k.at(i);
  }

  if (d_gains.size() == static_cast<uint>(num_joints_)) {
    for (int i = 0; i < num_joints_; ++i) d_gains_(i) = d_gains.at(i);
  } else {
    for (int i = 0; i < num_joints_; ++i) d_gains_(i) = default_d.at(i);
  }

  dq_filtered_.setZero();
  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardMoveToStartController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  q_ = Vector7d(franka_robot_model_->getRobotState()->q.data());
  dq_ = Vector7d(franka_robot_model_->getRobotState()->dq.data());
  dq_filtered_.setZero();

  motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_goal_);
  start_time_ = get_node()->now();

  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardMoveToStartController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void StandardMoveToStartController::updateJointStates() {
  q_ = Vector7d(franka_robot_model_->getRobotState()->q.data());
  dq_ = Vector7d(franka_robot_model_->getRobotState()->dq.data());
}

controller_interface::return_type StandardMoveToStartController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();

  Vector7d coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Matrix7d inertia(franka_robot_model_->getMassMatrix().data());

  auto trajectory_time = get_node()->now() - start_time_;
  auto motion_output = motion_generator_->getDesiredJointPositions(trajectory_time);
  Vector7d q_desired = motion_output.first;
  bool finished = motion_output.second;

  const double kAlpha = 0.2;
  dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;

  Vector7d damping_ratio = Vector7d::Constant(0.8);
  Matrix7d stiffness_mat = k_gains_.asDiagonal();
  Matrix7d designed_damping = panda_controllers::sqrtDesign<7>(inertia, stiffness_mat, damping_ratio);

  Vector7d target_q = finished ? q_goal_ : q_desired;
  Vector7d tau_d = coriolis + stiffness_mat * (target_q - q_) - designed_damping * dq_filtered_;

  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

}  // namespace reflex_mc_controller

PLUGINLIB_EXPORT_CLASS(reflex_mc_controller::StandardMoveToStartController,
                       controller_interface::ControllerInterface)

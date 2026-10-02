#include <reflex_mc_controller_impl/standard_joint_impedance_controller.h>
#include <reflex_controllers/utils/damping_design.h>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace reflex_mc_controller {

controller_interface::InterfaceConfiguration
StandardJointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
StandardJointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

CallbackReturn StandardJointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    arm_id_ = get_node()->get_parameter("arm_id").as_string();

    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});

    std::string topic_name = "joint_impedance/" + arm_id_ + "_joints_desired";
    sub_desired_joint_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      topic_name, 1,
      std::bind(&StandardJointImpedanceController::desiredJointCallback, this, std::placeholders::_1)
    );
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during StandardJointImpedanceController init stage: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardJointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model", arm_id_));

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

CallbackReturn StandardJointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  q_ = Vector7d(franka_robot_model_->getRobotState()->q.data());
  dq_ = Vector7d(franka_robot_model_->getRobotState()->dq.data());
  dq_filtered_.setZero();
  q_d_ = q_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardJointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void StandardJointImpedanceController::updateJointStates() {
  q_ = Vector7d(franka_robot_model_->getRobotState()->q.data());
  dq_ = Vector7d(franka_robot_model_->getRobotState()->dq.data());
}

void StandardJointImpedanceController::desiredJointCallback(const sensor_msgs::msg::JointState& msg) {
  if (msg.name.empty() || msg.position.empty()) return;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    for (int j = 0; j < num_joints_; ++j) {
      std::string expected_joint_name = arm_id_ + "_joint" + std::to_string(j + 1);
      if (msg.name[i] == expected_joint_name && i < msg.position.size()) {
        q_d_(j) = msg.position[i];
        break;
      }
    }
  }
}

controller_interface::return_type StandardJointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();

  Vector7d coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Matrix7d inertia(franka_robot_model_->getMassMatrix().data());

  const double kAlpha = 0.2;
  dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;

  Vector7d damping_ratio = Vector7d::Constant(0.8);
  Matrix7d stiffness_mat = k_gains_.asDiagonal();
  Matrix7d designed_damping = panda_controllers::sqrtDesign<7>(inertia, stiffness_mat, damping_ratio);

  Vector7d tau_d = coriolis + stiffness_mat * (q_d_ - q_) - designed_damping * dq_filtered_;

  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

}  // namespace reflex_mc_controller

PLUGINLIB_EXPORT_CLASS(reflex_mc_controller::StandardJointImpedanceController,
                       controller_interface::ControllerInterface)

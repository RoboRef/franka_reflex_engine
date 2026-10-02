#include <reflex_mc_controller_impl/standard_collision_recorder_controller.h>
#include <reflex_controllers/utils/damping_design.h>

#include <cassert>
#include <cmath>
#include <exception>
#include <iomanip>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace reflex_mc_controller {

// ──────────────────────────────────────────────────────────────────────────────
//  Lifecycle
// ──────────────────────────────────────────────────────────────────────────────

CallbackReturn StandardCollisionRecorderController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    arm_id_ = get_node()->get_parameter("arm_id").as_string();

    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("record_duration", 60.0);
    auto_declare<std::string>("log_file", "/tmp/collision_recorder_data.csv");

    // Desired joint position subscriber (same pattern as StandardJointImpedanceController)
    std::string topic_name = "collision_recorder/" + arm_id_ + "_joints_desired";
    sub_desired_joint_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      topic_name, 1,
      std::bind(&StandardCollisionRecorderController::desiredJointCallback, this, std::placeholders::_1)
    );

    // ATI wrench subscriber
    ati_sub_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/ati_wrench", 10,
      std::bind(&StandardCollisionRecorderController::atiCallback, this, std::placeholders::_1)
    );

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during StandardCollisionRecorderController init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardCollisionRecorderController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model", arm_id_));

  std::vector<double> default_k = {300.0, 300.0, 300.0, 300.0, 100.0, 50.0, 50.0};
  std::vector<double> default_d = {20.0, 20.0, 20.0, 10.0, 10.0, 10.0, 5.0};

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  if (k_gains.size() == static_cast<uint>(NUM_JOINTS)) {
    for (int i = 0; i < NUM_JOINTS; ++i) k_gains_(i) = k_gains.at(i);
  } else {
    for (int i = 0; i < NUM_JOINTS; ++i) k_gains_(i) = default_k.at(i);
  }

  if (d_gains.size() == static_cast<uint>(NUM_JOINTS)) {
    for (int i = 0; i < NUM_JOINTS; ++i) d_gains_(i) = d_gains.at(i);
  } else {
    for (int i = 0; i < NUM_JOINTS; ++i) d_gains_(i) = default_d.at(i);
  }

  dq_filtered_.setZero();
  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardCollisionRecorderController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  q_ = Vector7d(franka_robot_model_->getRobotState()->q.data());
  dq_ = Vector7d(franka_robot_model_->getRobotState()->dq.data());
  dq_filtered_.setZero();
  q_d_ = q_;  // hold current position until subscriber sends a new goal

  // Read recording parameters
  record_duration_ = get_node()->get_parameter("record_duration").as_double();
  log_file_ = get_node()->get_parameter("log_file").as_string();

  time_elapsed_ = 0.0;
  recording_done_ = false;

  openCsvFiles(log_file_);

  RCLCPP_INFO(get_node()->get_logger(),
    "StandardCollisionRecorderController activated. record_duration=%.1f s, log_file=%s",
    record_duration_, log_file_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn StandardCollisionRecorderController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  closeCsvFiles();
  RCLCPP_INFO(get_node()->get_logger(),
    "StandardCollisionRecorderController deactivated — data saved to %s", log_file_.c_str());
  return CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────────────────────
//  Interface configuration
// ──────────────────────────────────────────────────────────────────────────────

controller_interface::InterfaceConfiguration
StandardCollisionRecorderController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= NUM_JOINTS; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
StandardCollisionRecorderController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(name);
  }
  return config;
}

// ──────────────────────────────────────────────────────────────────────────────
//  Callbacks
// ──────────────────────────────────────────────────────────────────────────────

void StandardCollisionRecorderController::updateJointStates() {
  q_ = Vector7d(franka_robot_model_->getRobotState()->q.data());
  dq_ = Vector7d(franka_robot_model_->getRobotState()->dq.data());
}

void StandardCollisionRecorderController::desiredJointCallback(
    const sensor_msgs::msg::JointState& msg) {
  if (msg.name.empty() || msg.position.empty()) return;
  for (size_t i = 0; i < msg.name.size(); ++i) {
    for (int j = 0; j < NUM_JOINTS; ++j) {
      std::string expected_joint_name = arm_id_ + "_joint" + std::to_string(j + 1);
      if (msg.name[i] == expected_joint_name && i < msg.position.size()) {
        q_d_(j) = msg.position[i];
        break;
      }
    }
  }
}

void StandardCollisionRecorderController::atiCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(ati_mutex_);
  latest_ati_wrench_[0] = msg->wrench.force.x;
  latest_ati_wrench_[1] = msg->wrench.force.y;
  latest_ati_wrench_[2] = msg->wrench.force.z;
  latest_ati_wrench_[3] = msg->wrench.torque.x;
  latest_ati_wrench_[4] = msg->wrench.torque.y;
  latest_ati_wrench_[5] = msg->wrench.torque.z;
}

// ──────────────────────────────────────────────────────────────────────────────
//  Update (1 kHz)
// ──────────────────────────────────────────────────────────────────────────────

controller_interface::return_type StandardCollisionRecorderController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {

  updateJointStates();

  // ── Dynamics ──────────────────────────────────────────────────────────────
  auto mass_arr     = franka_robot_model_->getMassMatrix();
  auto coriolis_arr = franka_robot_model_->getCoriolisForceVector();

  Eigen::Map<const Matrix7d> M(mass_arr.data());
  Eigen::Map<const Vector7d> coriolis(coriolis_arr.data());

  // ── Jacobians ─────────────────────────────────────────────────────────────
  auto jEE_arr = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<const Matrix6x7> J_EE(jEE_arr.data());

  static const franka::Frame kFrames[7] = {
    franka::Frame::kJoint1, franka::Frame::kJoint2, franka::Frame::kJoint3,
    franka::Frame::kJoint4, franka::Frame::kJoint5, franka::Frame::kJoint6,
    franka::Frame::kJoint7
  };
  std::array<std::array<double, 42>, 7> jLink_arr;
  for (int l = 0; l < 7; ++l) {
    jLink_arr[l] = franka_robot_model_->getZeroJacobian(kFrames[l]);
  }

  // ── Low-pass velocity filter ──────────────────────────────────────────────
  const double kAlpha = 0.2;
  dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;

  // ── Joint impedance control law ───────────────────────────────────────────
  Matrix7d inertia(mass_arr.data());
  Vector7d damping_ratio = Vector7d::Constant(0.8);
  Matrix7d stiffness_mat = k_gains_.asDiagonal();
  Matrix7d designed_damping = panda_controllers::sqrtDesign<7>(inertia, stiffness_mat, damping_ratio);

  Vector7d tau_d = coriolis + stiffness_mat * (q_d_ - q_) - designed_damping * dq_filtered_;

  // ── Command torques ───────────────────────────────────────────────────────
  for (int i = 0; i < NUM_JOINTS; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  // ── Data recording ────────────────────────────────────────────────────────
  if (!recording_done_) {
    const double dt = period.seconds();
    time_elapsed_ += dt;

    auto* rs = franka_robot_model_->getRobotState();
    Eigen::Map<const Vector7d> tau_J(rs->tau_J.data());
    Eigen::Map<const Vector7d> tau_ext(rs->tau_ext_hat_filtered.data());

    std::array<double, 6> O_F_ext;
    for (int i = 0; i < 6; ++i) O_F_ext[i] = rs->O_F_ext_hat_K[i];

    std::array<double, 6> ati_wrench{};
    {
      std::lock_guard<std::mutex> lock(ati_mutex_);
      ati_wrench = latest_ati_wrench_;
    }

    Eigen::Matrix<double, 6, 1> ee_vel = J_EE * dq_;

    writePrimaryRow(time.seconds(), q_, dq_, tau_J, tau_ext, O_F_ext, ati_wrench, ee_vel);
    writeExtraRow(time.seconds(), mass_arr, jEE_arr, jLink_arr);

    if (time_elapsed_ > record_duration_) {
      RCLCPP_INFO(get_node()->get_logger(),
                  "StandardCollisionRecorderController: record_duration %.1f s reached.",
                  record_duration_);
      recording_done_ = true;
    }
  }

  return controller_interface::return_type::OK;
}

// ──────────────────────────────────────────────────────────────────────────────
//  CSV helpers
// ──────────────────────────────────────────────────────────────────────────────

void StandardCollisionRecorderController::openCsvFiles(const std::string& log_file) {
  // Primary CSV
  primary_file_.open(log_file, std::ios::out);
  if (primary_file_.is_open()) {
    primary_file_ << "timestamp,approach_speed";
    for (int i = 0; i < 7; ++i) primary_file_ << ",q_" << i;
    for (int i = 0; i < 7; ++i) primary_file_ << ",qd_" << i;
    for (int i = 0; i < 7; ++i) primary_file_ << ",tau_" << i;
    primary_file_ << ",robot_stiffness_translational,robot_stiffness_rotational"
                  << ",piece_stiffness,payload_mass";
    for (int i = 0; i < 7; ++i) primary_file_ << ",tau_ext_" << i;
    for (int i = 0; i < 6; ++i) primary_file_ << ",wrench_" << i;
    for (int i = 0; i < 6; ++i) primary_file_ << ",ati_" << (i < 3 ? "f" : "t")
                                               << (i < 3 ? std::string(1, 'x' + i)
                                                         : std::string(1, 'x' + i - 3));
    for (int i = 0; i < 6; ++i) primary_file_ << ",ee_vel_" << i;
    primary_file_ << "\n";
    primary_file_.flush();
    primary_header_written_ = true;
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "Could not open primary file %s!", log_file.c_str());
  }

  // Extra CSV (M, J_EE, J_link1..7)
  std::string base = log_file;
  auto dot_pos = base.find_last_of('.');
  if (dot_pos != std::string::npos) base = base.substr(0, dot_pos);
  std::string extra_path = base + "_chain_extra.csv";

  extra_file_.open(extra_path, std::ios::out);
  if (extra_file_.is_open()) {
    extra_file_ << "timestamp";
    for (int i = 0; i < 49; ++i) extra_file_ << ",M_" << i;
    for (int i = 0; i < 42; ++i) extra_file_ << ",J_EE_" << i;
    const char* lnames[7] = {
      "J_link1","J_link2","J_link3","J_link4","J_link5","J_link6","J_link7"};
    for (int l = 0; l < 7; ++l)
      for (int i = 0; i < 42; ++i)
        extra_file_ << "," << lnames[l] << "_" << i;
    extra_file_ << "\n";
    extra_file_.flush();
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "Could not open extra file %s!", extra_path.c_str());
  }
}

void StandardCollisionRecorderController::closeCsvFiles() {
  if (primary_file_.is_open()) primary_file_.close();
  if (extra_file_.is_open()) extra_file_.close();
}

void StandardCollisionRecorderController::writePrimaryRow(
    double timestamp,
    const Vector7d& q, const Vector7d& dq,
    const Vector7d& tau, const Vector7d& tau_ext,
    const std::array<double, 6>& O_F_ext,
    const std::array<double, 6>& ati_wrench,
    const Eigen::Matrix<double, 6, 1>& ee_vel) {
  if (!primary_file_.is_open()) return;

  primary_file_ << std::fixed << std::setprecision(8) << timestamp
                << "," << 0.0;  // approach_speed placeholder
  for (int i = 0; i < 7; ++i) primary_file_ << "," << q(i);
  for (int i = 0; i < 7; ++i) primary_file_ << "," << dq(i);
  for (int i = 0; i < 7; ++i) primary_file_ << "," << tau(i);
  primary_file_ << "," << k_gains_(0) << "," << d_gains_(0)
                << "," << 0.0 << "," << 0.0;  // piece_stiffness, payload_mass placeholders
  for (int i = 0; i < 7; ++i) primary_file_ << "," << tau_ext(i);
  for (int i = 0; i < 6; ++i) primary_file_ << "," << O_F_ext[i];
  for (int i = 0; i < 6; ++i) primary_file_ << "," << ati_wrench[i];
  for (int i = 0; i < 6; ++i) primary_file_ << "," << ee_vel(i);
  primary_file_ << "\n";
  primary_file_.flush();
}

void StandardCollisionRecorderController::writeExtraRow(
    double timestamp,
    const std::array<double, 49>& mass,
    const std::array<double, 42>& jEE,
    const std::array<std::array<double, 42>, 7>& jLinks) {
  if (!extra_file_.is_open()) return;

  extra_file_ << std::fixed << std::setprecision(8) << timestamp;
  for (double v : mass)  extra_file_ << "," << v;
  for (double v : jEE)   extra_file_ << "," << v;
  for (int l = 0; l < 7; ++l)
    for (double v : jLinks[l]) extra_file_ << "," << v;
  extra_file_ << "\n";
  extra_file_.flush();
}

}  // namespace reflex_mc_controller

PLUGINLIB_EXPORT_CLASS(reflex_mc_controller::StandardCollisionRecorderController,
                       controller_interface::ControllerInterface)

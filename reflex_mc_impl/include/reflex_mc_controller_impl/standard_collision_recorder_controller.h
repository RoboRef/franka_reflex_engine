#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <array>
#include <fstream>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace reflex_mc_controller {

/**
 * StandardCollisionRecorderController
 *
 * A joint-impedance controller that:
 *   1. Accepts desired joint positions from a subscriber (sensor_msgs/JointState)
 *   2. Records full dynamics data (q, dq, tau, tau_ext, M, J_links, ATI wrench)
 *      at 1 kHz to CSV files for offline collision analysis
 *   3. Lives inside the reflex_mc_controller_impl so the ReflexEngine can
 *      automatically switch to zero-g on collision detection
 *
 * CSV output format is identical to CollisionChainController:
 *   - Primary CSV: timestamp, q(7), dq(7), tau(7), tau_ext(7), wrench(6), ATI(6), ee_vel(6)
 *   - Extra CSV:   timestamp, M(49), J_EE(42), J_link1..7(7×42)
 */
class StandardCollisionRecorderController : public controller_interface::ControllerInterface {
 public:
  using Vector7d  = Eigen::Matrix<double, 7, 1>;
  using Matrix7d  = Eigen::Matrix<double, 7, 7>;
  using Matrix6x7 = Eigen::Matrix<double, 6, 7>;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // ── Robot model ─────────────────────────────────────────────────────────────
  std::string arm_id_;
  static constexpr int NUM_JOINTS = 7;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  // ── Joint impedance state ───────────────────────────────────────────────────
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  Vector7d q_d_;          // desired joint positions (from subscriber)

  // ── Desired joint subscriber ────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_desired_joint_;
  void desiredJointCallback(const sensor_msgs::msg::JointState& msg);
  void updateJointStates();

  // ── ATI Sensor ──────────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ati_sub_;
  std::array<double, 6> latest_ati_wrench_{};
  std::mutex ati_mutex_;
  void atiCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // ── Data recording ──────────────────────────────────────────────────────────
  std::string log_file_;
  double record_duration_{60.0};
  double time_elapsed_{0.0};
  bool recording_done_{false};

  std::ofstream primary_file_;
  std::ofstream extra_file_;
  bool primary_header_written_{false};

  void openCsvFiles(const std::string& log_file);
  void closeCsvFiles();
  void writePrimaryRow(double timestamp,
                       const Vector7d& q, const Vector7d& dq,
                       const Vector7d& tau, const Vector7d& tau_ext,
                       const std::array<double, 6>& O_F_ext,
                       const std::array<double, 6>& ati_wrench,
                       const Eigen::Matrix<double, 6, 1>& ee_vel);
  void writeExtraRow(double timestamp,
                     const std::array<double, 49>& mass,
                     const std::array<double, 42>& jEE,
                     const std::array<std::array<double, 42>, 7>& jLinks);
};

}  // namespace reflex_mc_controller

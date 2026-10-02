#pragma once

#include <string>
#include <memory>
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <reflex_controllers/utils/damping_design.h>
#include <reflex_controllers/utils/nullspace_projection.h>
#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace reflex_mc_controller {

class StandardWipingController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;
  using Matrix4d = Eigen::Matrix<double, 4, 4>;
  using Quaterniond = Eigen::Quaterniond;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_{"panda"};
  const int num_joints_{7};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  double time_traj_{0.0};
  double target_z_force_{-5.0};
  double current_z_force_{0.0};
  double f_error_integral_z_{0.0};

  Vector3d initial_position_{Vector3d::Zero()};
  Quaterniond initial_orientation_{Quaterniond::Identity()};
  Vector7d initial_qn_{Vector7d::Zero()};
  Vector6d initial_f_ext_hat_{Vector6d::Zero()};
  Vector7d initial_tau_ext_{Vector7d::Zero()};
  bool initialized_start_pose_{false};
};

}  // namespace reflex_mc_controller

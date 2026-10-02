#pragma once

#include <reflex_engine/reflex_engine_interface.h>
#include <reflex_engine/contact_detection/panda_contact_checker.h>
#include <multi_mode_control_msgs/srv/set_controllers.hpp>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <atomic>
#include <string>
#include <vector>
#include <array>

namespace reflex_engine {

enum class SimpleState { NORMAL, REFLEX, WAIT_FOR_RECOVERY, RECOVERY };

class SimpleReflexEngine : public ReflexEngineInterface {
public:
  SimpleReflexEngine();
  ~SimpleReflexEngine();

private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                      panda_controllers::PandaControllerInterface* control) override;
  bool initImpl(const std::vector<panda_controllers::RobotData*>& robot_data,
                rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string name, std::string resource) override;
  void startImpl(panda_controllers::PandaControllerInterface* control) override;
  void stopImpl(panda_controllers::PandaControllerInterface* control) override;

  // Observers
  void checkHazards(panda_controllers::PandaControllerInterface* control);
  bool checkJointLimits(double margin);
  std::vector<std::string> getJointLimitViolationReasons(double margin);
  bool checkJointVelocityLimits(double buffer = 0.2);
  std::vector<std::string> getJointVelocityLimitViolationReasons(double buffer = 0.2);
  void applySoftVelocityDamping(const std::vector<std::array<double, 7>*>& tau);
  bool checkContact();
  bool checkHapticRecovery();
  
  // State machine actions
  void triggerReflex(const std::string& safety_controller);
  void triggerRecovery();
  void returnToNormal();

  std::string last_reflex_reason_;

  SimpleState state_ = SimpleState::NORMAL;
  std::string resource_;
  std::vector<panda_controllers::RobotData*> robot_data_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  PandaContactChecker contact_checker_; 

  // Services and Clients
  rclcpp::Client<multi_mode_control_msgs::srv::SetControllers>::SharedPtr set_controllers_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr recover_service_;
  rclcpp::Service<multi_mode_control_msgs::srv::GetRobotStates>::SharedPtr get_robot_states_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;

  void publishState(const std::string& state_name);

  bool recoverServiceCallback(const std_srvs::srv::Empty::Request::SharedPtr req,
                              std_srvs::srv::Empty::Response::SharedPtr res);
  void getRobotStatesCallback(const multi_mode_control_msgs::srv::GetRobotStates::Request::SharedPtr req,
                              multi_mode_control_msgs::srv::GetRobotStates::Response::SharedPtr res);
  
  // Parameter loading
  void loadParameters(const std::string& resource);

  std::string nominal_controller_ = "panda_joint_impedance_controller";
  std::string reflex_for_collision_ = "panda_zero_g_controller";
  std::string reflex_for_joint_pos_limit_ = "panda_stop_controller";
  std::string reflex_for_joint_vel_limit_ = "panda_stop_controller";

  
  // Joint limits for Franka Panda
  std::array<double, 7> q_min_{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  std::array<double, 7> q_max_{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
  double joint_limit_margin_ = 0.15; // rad (distance to limit before triggering)
  double joint_limit_margin_recovery_ = 0.1; // rad (relaxed margin during recovery to allow escaping the limit zone)

  // Joint velocity limits for Franka Panda (rad/s)
  std::array<double, 7> qd_max_{2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
  double joint_velocity_soft_buffer_ = 0.2;  // rad/s (soft buffer: apply active virtual damping)
  double joint_velocity_hard_buffer_ = 0.05; // rad/s (hard buffer: trigger panda_stop_controller)
  std::array<double, 7> joint_velocity_damping_gains_{500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0}; // Nms/rad

  // Engine behavior settings
  bool verbose_logging_ = true;
  bool enable_gesture_recovery_ = false;
  double gesture_threshold_ = 20.0;
  double recovery_cooldown_time_ = 3.0;

  // Recovery goal position
  std::array<double, 7> q_start_{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};

  std::atomic<bool> recovery_requested_{false};
  rclcpp::Time reflex_time_;
  
  // For instantaneous torque safety while waiting for SetControllers service
  std::vector<std::array<double, 7>> safe_tau_;
};

}

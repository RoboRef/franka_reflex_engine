#pragma once

#include <mutex>

#include <Eigen/Dense>
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
// #include <ros/time.h>

// #include <franka_hw/franka_model_interface.h>
// #include <franka_hw/franka_state_interface.h>
// #include <hardware_interface/robot_hw.h>
#include <reflex_controllers/utils/buffered.h>
#include <reflex_controllers/utils/robot_data.h>

namespace reflex_engine {

struct ContactThresholds {
  Eigen::Matrix<double, 6, 1> F_max;
  Eigen::Matrix<double, 6, 1> F_min;
  std::vector<Eigen::Matrix<double, 7, 1>> tau_max;
  std::vector<Eigen::Matrix<double, 7, 1>> tau_min;
  double f_abs_max;
  double m_abs_max;
  bool abs_mode;
  bool directional_mode;
  bool torque_mode;
};

class PandaContactChecker {
 public:
  PandaContactChecker();
  ~PandaContactChecker() = default;
  bool init(const std::vector<panda_controllers::RobotData*>& robot_data,
            rclcpp_lifecycle::LifecycleNode::SharedPtr nh, const std::string& resource);
  bool checkGesture();
  const Eigen::Matrix<double, 6, 1>& getContactWrench();
  bool hasLevel1Contact();
  bool hasLevel2Contact();
  const std::vector<std::string>& getLevel1ContactReasons() const;
  const std::vector<std::string>& getLevel2ContactReasons() const;
  bool hasGesture();
  void setGestureThresholdBuffered(const double& t);
  void setLevel1ThresholdsBuffered(const ContactThresholds& t);
  void setLevel2ThresholdsBuffered(const ContactThresholds& t);
  void updateThresholdsFromBuffer();
 private:
  std::recursive_mutex threshold_mutex_;
  std::vector<Eigen::Vector3d> r_B_0_;
  std::vector<Eigen::Matrix3d> B_R_0_;
  panda_controllers::Buffered<ContactThresholds> level1_thresholds_;
  panda_controllers::Buffered<ContactThresholds> level2_thresholds_;
  panda_controllers::Buffered<double> gesture_threshold_;
  rclcpp::Time last_gesture_;
  std::vector<panda_controllers::RobotData*> robot_data_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  bool calculated_wrench_ = false;
  bool checked_level1_ = false;
  bool checked_level2_ = false;
  bool checked_gesture_ = false;
  bool has_level1_contact_ = false;
  bool has_level2_contact_ = false;
  bool has_gesture_ = false;
  std::vector<std::string> level1_reasons_;
  std::vector<std::string> level2_reasons_;
  Eigen::Matrix<double, 6, 1> contact_wrench_;
  void calculateContactWrench();
  bool hasContact(const ContactThresholds& th);
  std::vector<std::string> getContactReasons(const ContactThresholds& th);
};

}

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <mutex>
#include <string>
#include <memory>
#include <vector>
#include <array>

#include <reflex_controllers/base/panda_controller_ros_interface.h>
#include <reflex_controllers/controllers/comless_panda_joint_impedance_controller.h>
#include <multi_mode_control_msgs/srv/set_joint_impedance.hpp>
#include <multi_mode_control_msgs/msg/joint_goal.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace panda_controllers {

using GoalMsg = multi_mode_control_msgs::msg::JointGoal;
using ServiceParameter = multi_mode_control_msgs::srv::SetJointImpedance;
using Pose = PandaJointImpedanceControllerPose;

class PandaCollisionRecorderController :
    public virtual PandaControllerBase<double, Pose>,
    public virtual ControllerRosInterface<ServiceParameter,
                                          GoalMsg,
                                          double,
                                          Pose> {
public:
  virtual ~PandaCollisionRecorderController() = default;

protected:
  void initROSComImpl() override;
  void startImpl() override;
  void stopImpl() override;
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                      const Pose& desired,
                      const double& p) override;
  double defaultParameters() override;
  Pose getCurrentPoseImpl() override;
  bool hasOffsetImpl() override;
  void resetOffset() override;

private:
  bool desiredPoseCallbackImpl(Pose& p_d,
                               const Pose& p,
                               const GoalMsg& msg) override;

  bool setParametersCallbackImpl(double& p_d, const double& p,
                                 const ServiceParameter::Request::SharedPtr& req,
                                 const ServiceParameter::Response::SharedPtr& res) override;

  void atiCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void jointStateGoalCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void openCsvFiles();
  void closeCsvFiles();

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ati_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_goal_sub_;

  std::mutex ati_mutex_;
  std::array<double, 6> ati_wrench_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool ati_received_{false};

  std::ofstream csv_file_;
  std::ofstream csv_extra_file_;
  std::string log_file_{"/tmp/collision_recording.csv"};
  double record_duration_{10.0};

  rclcpp::Time start_time_;
  bool is_recording_{false};
  size_t sample_count_{0};

  std::mutex goal_mutex_;
  bool has_received_goal_{false};
  double time_elapsed_{0.0};
  double j1_position_{0.0};
  double peak_vel_{0.3};
  double accel_{0.5};
  double t_ramp_{0.6};

  Eigen::Matrix<double, 7, 1> dq_filtered_;
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Matrix<double, 7, 1> q_target_;
  Eigen::Matrix<double, 7, 1> q_cmd_;
  Eigen::Matrix<double, 7, 1> qd_cmd_;
  Eigen::Matrix<double, 7, 1> max_joint_vel_;
  Eigen::Matrix<double, 7, 1> max_joint_acc_;
};

} // namespace panda_controllers



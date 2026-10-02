#include <reflex_controllers/controllers/panda_collision_recorder_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace panda_controllers;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Controller = PandaCollisionRecorderController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "panda_collision_recorder_controller");

void Controller::initROSComImpl() {
  if (PandaControllerInterface::node_) {
    ati_sub_ = PandaControllerInterface::node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/ati_wrench",
        rclcpp::QoS(1),
        std::bind(&Controller::atiCallback, this, std::placeholders::_1));

    joint_state_goal_sub_ = PandaControllerInterface::node_->create_subscription<sensor_msgs::msg::JointState>(
        "/collision_recorder/" + PandaControllerInterface::resource_ + "_joints_desired",
        rclcpp::QoS(1),
        std::bind(&Controller::jointStateGoalCallback, this, std::placeholders::_1));
  }
}

void Controller::atiCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(ati_mutex_);
  ati_wrench_[0] = msg->wrench.force.x;
  ati_wrench_[1] = msg->wrench.force.y;
  ati_wrench_[2] = msg->wrench.force.z;
  ati_wrench_[3] = msg->wrench.torque.x;
  ati_wrench_[4] = msg->wrench.torque.y;
  ati_wrench_[5] = msg->wrench.torque.z;
  ati_received_ = true;
}

void Controller::jointStateGoalCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() >= 7) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      q_target_[i] = msg->position[i];
    }
    has_received_goal_ = true;
  }
}

bool Controller::desiredPoseCallbackImpl(Pose& p_d, const Pose& p, const GoalMsg& msg) {
  (void)p;
  if (msg.q.size() >= 7) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      q_target_[i] = msg.q[i];
      p_d.q[i] = msg.q[i];
      p_d.qD[i] = (msg.qd.size() >= 7) ? msg.qd[i] : 0.0;
    }
    has_received_goal_ = true;
    return true;
  }
  return false;
}

bool Controller::setParametersCallbackImpl(double& p_d, const double& p,
                                            const ServiceParameter::Request::SharedPtr& req,
                                            const ServiceParameter::Response::SharedPtr& res) {
  (void)req;
  (void)res;
  p_d = p;
  return true;
}

double Controller::defaultParameters() {
  return 1.0;
}

Pose Controller::getCurrentPoseImpl() {
  Pose current;
  current.q = Vector7d(robot_data_[0]->state().q.data());
  current.qD = Vector7d(robot_data_[0]->state().dq.data());
  return current;
}

bool Controller::hasOffsetImpl() {
  return getOffset().q.norm() != 0;
}

void Controller::resetOffset() {
  Pose p;
  p.q = Vector7d::Zero();
  p.qD = Vector7d::Zero();
  setOffset(p);
}

void Controller::openCsvFiles() {
  closeCsvFiles();

  if (log_file_.empty()) {
    is_recording_ = false;
    return;
  }

  csv_file_.open(log_file_);
  if (csv_file_.is_open()) {
    csv_file_ << "timestamp";
    for (int i = 0; i < 7; ++i) csv_file_ << ",q_" << i;
    for (int i = 0; i < 7; ++i) csv_file_ << ",dq_" << i;
    for (int i = 0; i < 7; ++i) csv_file_ << ",tau_" << i;
    for (int i = 0; i < 7; ++i) csv_file_ << ",tau_ext_" << i;
    csv_file_ << ",ati_fx,ati_fy,ati_fz,ati_tx,ati_ty,ati_tz\n";
    csv_file_ << std::fixed << std::setprecision(6);
  }

  std::string extra_path = log_file_;
  size_t dot_pos = extra_path.rfind('.');
  if (dot_pos != std::string::npos) {
    extra_path.insert(dot_pos, "_chain_extra");
  } else {
    extra_path += "_chain_extra.csv";
  }

  csv_extra_file_.open(extra_path);
  if (csv_extra_file_.is_open()) {
    csv_extra_file_ << "timestamp";
    for (int i = 0; i < 49; ++i) csv_extra_file_ << ",M_" << i;
    for (int i = 0; i < 42; ++i) csv_extra_file_ << ",J_EE_" << i;
    const char* lnames[7] = {
      "J_link1","J_link2","J_link3","J_link4","J_link5","J_link6","J_link7"};
    for (int l = 0; l < 7; ++l)
      for (int i = 0; i < 42; ++i)
        csv_extra_file_ << "," << lnames[l] << "_" << i;
    csv_extra_file_ << ",ee_vx,ee_vy,ee_vz,ee_wx,ee_wy,ee_wz\n";
    csv_extra_file_ << std::fixed << std::setprecision(6);
  }

  sample_count_ = 0;
  is_recording_ = true;
}

void Controller::closeCsvFiles() {
  if (csv_file_.is_open()) csv_file_.close();
  if (csv_extra_file_.is_open()) csv_extra_file_.close();
  is_recording_ = false;
}

void Controller::startImpl() {
  if (PandaControllerInterface::node_) {
    start_time_ = PandaControllerInterface::node_->get_clock()->now();
    PandaControllerInterface::node_->get_parameter_or("panda_collision_recorder_controller.log_file", log_file_, std::string("/tmp/collision_recording.csv"));
    PandaControllerInterface::node_->get_parameter_or("panda_collision_recorder_controller.record_duration", record_duration_, 10.0);

    peak_vel_ = 0.3;
    accel_ = 0.5;
    PandaControllerInterface::node_->get_parameter_or("panda_collision_recorder_controller.peak_velocity", peak_vel_, 0.3);
    PandaControllerInterface::node_->get_parameter_or("panda_collision_recorder_controller.max_velocity", peak_vel_, peak_vel_);
    PandaControllerInterface::node_->get_parameter_or("panda_collision_recorder_controller.acceleration", accel_, 0.5);
    PandaControllerInterface::node_->get_parameter_or("panda_collision_recorder_controller.max_acceleration", accel_, accel_);

    max_joint_vel_.setConstant(peak_vel_);
    max_joint_acc_.setConstant(accel_);
  } else {
    start_time_ = rclcpp::Clock().now();
    peak_vel_ = 0.3;
    accel_ = 0.5;
    record_duration_ = 10.0;
    max_joint_vel_.setConstant(0.3);
    max_joint_acc_.setConstant(0.5);
  }

  time_elapsed_ = 0.0;
  j1_position_ = 0.0;
  t_ramp_ = (accel_ > 0.0) ? (peak_vel_ / accel_) : 0.0;

  has_received_goal_ = false;
  dq_filtered_.setZero();

  {
    std::lock_guard<std::mutex> lock(ati_mutex_);
    ati_wrench_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  // Snapshot physical initial joint configuration at activation to eliminate step error spikes
  q_init_ = Vector7d(robot_data_[0]->state().q.data());
  q_target_ = q_init_;
  q_cmd_ = q_init_;
  qd_cmd_.setZero();

  openCsvFiles();
}

void Controller::stopImpl() {
  closeCsvFiles();
}

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                                 const Pose& desired,
                                 const double& p) {
  Pose current = getCurrentPose();
  current.q -= getOffset().q;

  // Filter joint velocity (matching collision_chain_controller.cpp)
  constexpr double kAlphaVel = 0.2;
  Eigen::Map<const Vector7d> dq(robot_data_[0]->state().dq.data());
  dq_filtered_ = (1.0 - kAlphaVel) * dq_filtered_ + kAlphaVel * dq;

  constexpr double dt = 0.001;
  time_elapsed_ += dt;

  Vector7d target;
  bool external_goal = false;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    external_goal = has_received_goal_;
    target = q_target_;
  }

  if (external_goal) {
    // 1000 Hz smooth trajectory filter towards external ROS 2 topic target
    constexpr double kTauV = 0.04; // Velocity tracking time constant (s)
    for (size_t i = 0; i < 7; ++i) {
      double e = target[i] - q_cmd_[i];
      if (std::abs(e) < 1e-6 && std::abs(qd_cmd_[i]) < 1e-5) {
        q_cmd_[i] = target[i];
        qd_cmd_[i] = 0.0;
        continue;
      }
      double v_des = e / kTauV;
      v_des = std::clamp(v_des, -max_joint_vel_[i], max_joint_vel_[i]);
      double dv = v_des - qd_cmd_[i];
      double max_dv = max_joint_acc_[i] * dt;
      dv = std::clamp(dv, -max_dv, max_dv);
      qd_cmd_[i] += dv;
      q_cmd_[i] += qd_cmd_[i] * dt;
    }
  } else {
    // Autonomous trapezoidal velocity trajectory for Joint 1 (matching real2sim_openloop & collision_chain controllers)
    double vel_j1 = 0.0;
    if (time_elapsed_ <= t_ramp_) {
      vel_j1 = -accel_ * time_elapsed_;
    } else {
      vel_j1 = -peak_vel_;
    }
    j1_position_ += vel_j1 * dt;

    q_cmd_ = q_init_;
    q_cmd_(0) = q_init_(0) + j1_position_;
  }

  Eigen::Map<const Matrix7d> mass(robot_data_[0]->mass().data());
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());

  static const Vector7d stiffness = (Vector7d() << 300.0, 300.0, 300.0, 300.0, 100.0, 50.0, 50.0).finished();
  static const Vector7d damping   = (Vector7d() << 20.0, 20.0, 20.0, 10.0, 10.0, 10.0, 5.0).finished();

  // Control law matching collision_chain_controller.cpp (diagonal stiffness & velocity-filtered damping)
  Vector7d tau_d = coriolis 
                 + p * stiffness.asDiagonal() * (q_cmd_ - current.q) 
                 - p * damping.asDiagonal() * dq_filtered_;

  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }


  // --- CSV Recording ---
  if (!is_recording_ || !PandaControllerInterface::node_) {
    return;
  }

  double t_rel = (PandaControllerInterface::node_->get_clock()->now() - start_time_).seconds();
  if (t_rel > record_duration_) {
    closeCsvFiles();
    return;
  }

  const auto& state = robot_data_[0]->state();
  const auto& J_ee = robot_data_[0]->eeZeroJacobian();

  std::array<double, 6> wrench_copy{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  {
    std::lock_guard<std::mutex> lock(ati_mutex_);
    wrench_copy = ati_wrench_;
  }

  if (csv_file_.is_open()) {
    csv_file_ << t_rel;
    for (int i = 0; i < 7; ++i) csv_file_ << "," << state.q[i];
    for (int i = 0; i < 7; ++i) csv_file_ << "," << state.dq[i];
    for (int i = 0; i < 7; ++i) csv_file_ << "," << state.tau_J[i];
    for (int i = 0; i < 7; ++i) csv_file_ << "," << state.tau_ext_hat_filtered[i];
    for (int i = 0; i < 6; ++i) csv_file_ << "," << wrench_copy[i];
    csv_file_ << "\n";
  }

  if (csv_extra_file_.is_open()) {
    csv_extra_file_ << t_rel;
    for (int i = 0; i < 7; ++i) {
      for (int j = 0; j < 7; ++j) {
        csv_extra_file_ << "," << mass(i, j);
      }
    }
    // 6 link Jacobians: 6 * 42 = 252 values + padding to 294 to match 42x7 matrix layout
    for (int l = 0; l < 6; ++l) {
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 7; ++c) {
          csv_extra_file_ << "," << J_ee(r, c);
        }
      }
    }
    for (int i = 252; i < 294; ++i) {
      csv_extra_file_ << ",0.0";
    }

    Eigen::Map<const Vector7d> dq_vec(state.dq.data());
    Eigen::Matrix<double, 6, 1> ee_vel = J_ee * dq_vec;
    for (int i = 0; i < 6; ++i) {
      csv_extra_file_ << "," << ee_vel[i];
    }
    csv_extra_file_ << "\n";
  }

  sample_count_++;
}


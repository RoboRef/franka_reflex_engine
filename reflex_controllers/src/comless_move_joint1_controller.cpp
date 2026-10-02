#include <reflex_controllers/controllers/comless_move_joint1_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <cmath>

using namespace panda_controllers;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = ComlessMoveJoint1ControllerPose;
using Params = ComlessMoveJoint1ControllerParams;
using Controller = ComlessMoveJoint1Controller;

static auto registration = ControllerFactory::registerClass<Controller>(
    "comless_move_joint1_controller");

bool Controller::initImpl(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) {
  if (!node->has_parameter("comless_move_joint1_controller.max_velocity")) {
    node->declare_parameter("comless_move_joint1_controller.max_velocity", 1.0);
  }
  if (!node->has_parameter("comless_move_joint1_controller.acceleration")) {
    node->declare_parameter("comless_move_joint1_controller.acceleration", 1.0);
  }
  if (!node->has_parameter("comless_move_joint1_controller.distance")) {
    node->declare_parameter("comless_move_joint1_controller.distance", 2.0);
  }
  return true;
}

void Controller::startImpl() {
  initialized_start_pose_ = false;
  time_traj_ = 0.0;
  Params p_curr = getParametersBuffered();
  if (node_->has_parameter("comless_move_joint1_controller.max_velocity")) {
    node_->get_parameter("comless_move_joint1_controller.max_velocity", p_curr.max_velocity);
  }
  if (node_->has_parameter("comless_move_joint1_controller.acceleration")) {
    node_->get_parameter("comless_move_joint1_controller.acceleration", p_curr.acceleration);
  }
  if (node_->has_parameter("comless_move_joint1_controller.distance")) {
    node_->get_parameter("comless_move_joint1_controller.distance", p_curr.distance);
  }
  setParametersBuffered(p_curr);
}

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
    const Pose& /*desired*/, const Params& p) {
  Pose current = getCurrentPose();
  current.q -= getOffset().q;

  if (!initialized_start_pose_) {
    initial_q_ = current.q;
    time_traj_ = 0.0;
    initialized_start_pose_ = true;
  }

  double v_max = p.max_velocity;
  double accel = p.acceleration;
  double distance = p.distance;
  
  double t_r = v_max / accel;
  double d_r = 0.5 * accel * t_r * t_r;
  
  double t_c = 0.0;
  double d_c = 0.0;
  if (2 * d_r > distance) {
    d_r = distance / 2.0;
    t_r = std::sqrt(distance / accel);
    v_max = accel * t_r;
    t_c = 0.0;
    d_c = 0.0;
  } else {
    d_c = distance - 2 * d_r;
    t_c = d_c / v_max;
  }
  
  double q_d_disp = 0.0;
  double dq_d_disp = 0.0;

  if (time_traj_ < t_r) {
    // Ramp up
    q_d_disp = 0.5 * accel * time_traj_ * time_traj_;
    dq_d_disp = accel * time_traj_;
  } else if (time_traj_ < t_r + t_c) {
    // Coast
    double t_coast = time_traj_ - t_r;
    q_d_disp = d_r + v_max * t_coast;
    dq_d_disp = v_max;
  } else if (time_traj_ < 2 * t_r + t_c) {
    // Ramp down
    double t_dec = time_traj_ - t_r - t_c;
    q_d_disp = d_r + d_c + v_max * t_dec - 0.5 * accel * t_dec * t_dec;
    dq_d_disp = v_max - accel * t_dec;
  } else {
    // Finished
    q_d_disp = 2 * d_r + d_c;
    dq_d_disp = 0.0;
  }

  // Desired pose (Joints 1 and 3 move by same displacement)
  Vector7d q_d = initial_q_;
  q_d[0] = initial_q_[0] + q_d_disp;
  q_d[2] = initial_q_[2] + q_d_disp;
  
  Vector7d dq_d = Vector7d::Zero();
  dq_d[0] = dq_d_disp;
  dq_d[2] = dq_d_disp;

  time_traj_ += 0.001; // 1kHz

  // Read robot state
  Eigen::Map<const Vector7d> q(robot_data_[0]->state().q.data());
  Eigen::Map<const Vector7d> dq(robot_data_[0]->state().dq.data());
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());

  // Compute joint impedance torque
  Vector7d tau_d;
  for (size_t i = 0; i < 7; ++i) {
    double err_q = q_d[i] - q[i];
    double err_dq = dq_d[i] - dq[i];
    
    // Critical damping: D = 2 * damping_ratio * sqrt(K * M_ii)
    // For simplicity without M, we just use a predefined damping or compute it.
    // Damping design is often D = 2 * damping_ratio * sqrt(K) if inertia is ~1, 
    // but typically we can just specify a direct damping gain.
    double damping = 2.0 * p.damping_ratio[i] * std::sqrt(p.stiffness[i]);
    
    tau_d[i] = p.stiffness[i] * err_q + damping * err_dq + coriolis[i];
  }
  
  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}

Params Controller::defaultParameters() {
  Params p;
  p.stiffness << 600, 600, 600, 600, 250, 150, 50;
  p.damping_ratio = Vector7d::Constant(1.0); // Critically damped
  p.max_velocity = 1.0;
  p.acceleration = 1.0;
  p.distance = 2.0;
  return p;
}

Pose Controller::getCurrentPoseImpl() {
  Pose p;
  Eigen::Map<const Vector7d> q(robot_data_[0]->state().q.data());
  Eigen::Map<const Vector7d> dq(robot_data_[0]->state().dq.data());
  p.q = q;
  p.qD = dq;
  return p;
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

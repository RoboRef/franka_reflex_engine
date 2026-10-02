#include <reflex_controllers/controllers/panda_move_to_start_controller.h>
#include <reflex_controllers/utils/controller_factory.h>
#include <cmath>

using namespace panda_controllers;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = PandaMoveToStartControllerPose;
using Controller = PandaMoveToStartController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "panda_move_to_start_controller");

template<typename T>
static T declare_or_get(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& name, const T& default_val) {
  if (!node) return default_val;
  if (node->has_parameter(name)) {
    T val;
    node->get_parameter(name, val);
    return val;
  }
  try {
    return node->declare_parameter<T>(name, default_val);
  } catch (const std::exception&) {
    if (node->has_parameter(name)) {
      T val;
      node->get_parameter(name, val);
      return val;
    }
  }
  return default_val;
}

void Controller::loadGoalPosition() {
  if (!node_) return;
  std::vector<double> default_goal = {0.0, -M_PI_4, 0.0, -3 * M_PI_4, 0.0, M_PI_2, M_PI_4};
  
  std::string prefix = resource_ + ".";
  std::vector<double> goal_vec = declare_or_get(node_, prefix + "recovery_goal_position", default_goal);
  if (goal_vec.size() != 7) {
    goal_vec = declare_or_get(node_, prefix + name_ + ".q_goal", default_goal);
  }

  if (goal_vec.size() == 7) {
    for (int i = 0; i < 7; ++i) {
      q_goal_(i) = goal_vec[i];
    }
    RCLCPP_INFO(node_->get_logger(),
      "PandaMoveToStartController: Target goal position for resource '%s': [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
      resource_.c_str(), q_goal_(0), q_goal_(1), q_goal_(2), q_goal_(3), q_goal_(4), q_goal_(5), q_goal_(6));
  } else {
    q_goal_ << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
  }
}

bool Controller::initImpl(const std::vector<RobotData*>& robot_data,
                          rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) {
  loadGoalPosition();
  k_gains_ << 300.0, 300.0, 300.0, 300.0, 100.0, 50.0, 50.0;
  d_gains_ << 20.0, 20.0, 20.0, 10.0, 10.0, 10.0, 5.0;
  dq_filtered_.setZero();
  return true;
}

void Controller::startImpl() {
  loadGoalPosition();
  Pose current = getCurrentPose();
  motion_generator_ = std::make_unique<MotionGenerator>(0.5, current.q, q_goal_);
  start_time_ = node_->now();
  dq_filtered_.setZero();
}

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const double& p) {
  
  Pose current = getCurrentPose();
  auto trajectory_time = node_->now() - start_time_;
  auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
  Vector7d q_desired = motion_generator_output.first;
  bool finished = motion_generator_output.second;
  
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * current.qD;
  
  // Adding coriolis compensation similar to comless_panda_joint_impedance_controller
  // since the standalone plugin relied on external gravity/coriolis compensation.
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());

  if (!finished) {
    Vector7d tau_d_calculated = coriolis + 
        k_gains_.cwiseProduct(q_desired - current.q) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < 7; ++i) {
      (*tau[0])[i] = tau_d_calculated(i);
    }
  } else {
    // Hold final position
    Vector7d tau_d_calculated = coriolis + 
        k_gains_.cwiseProduct(q_goal_ - current.q) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < 7; ++i) {
      (*tau[0])[i] = tau_d_calculated(i);
    }
  }
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
  return false;
}

void Controller::resetOffset() {
}

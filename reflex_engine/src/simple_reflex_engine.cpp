#include <reflex_engine/simple_reflex_engine.h>
#include <multi_mode_control_msgs/msg/controller.hpp>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <reflex_controllers/utils/sync_service_client.hpp>
#include <reflex_controllers/utils/conversions.h>
#include <reflex_controllers/utils/get_robot_state_msg.h>
#include <cmath>

using namespace reflex_engine;
using franka_msgs::srv::SetForceTorqueCollisionBehavior;

SimpleReflexEngine::SimpleReflexEngine() {}
SimpleReflexEngine::~SimpleReflexEngine() {}

bool SimpleReflexEngine::initImpl(const std::vector<panda_controllers::RobotData*>& robot_data,
                                  rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string name, std::string resource) {
  node_ = node;
  resource_ = resource;
  robot_data_ = robot_data;
  
  if (robot_data_.empty()) return false;
  safe_tau_.resize(robot_data_.size());

  std::string ns = node_->get_namespace();
  if (ns.back() != '/') ns += "/";

  // Derive the service name from the node (controller) name so this works regardless
  // of what instance name is given in the controller_manager YAML.
  set_controllers_client_ = node_->create_client<multi_mode_control_msgs::srv::SetControllers>(
      std::string(node_->get_name()) + "/set_controllers");
  
  recover_service_ = node_->create_service<std_srvs::srv::Empty>(
      ns + resource + "/recover",
      std::bind(&SimpleReflexEngine::recoverServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  get_robot_states_service_ = node_->create_service<multi_mode_control_msgs::srv::GetRobotStates>(
      ns + resource + "/get_robot_states",
      [this](const multi_mode_control_msgs::srv::GetRobotStates::Request::SharedPtr req,
             multi_mode_control_msgs::srv::GetRobotStates::Response::SharedPtr res) {
        this->getRobotStatesCallback(req, res);
      });

  state_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      ns + resource + "/safety_engine_state", 10);

  // Set hardware FT collision behavior for Franka robot(s) so libfranka does not trip prematurely
  auto request = std::make_shared<SetForceTorqueCollisionBehavior::Request>();
  auto response = std::make_shared<SetForceTorqueCollisionBehavior::Response>();
  request->lower_force_thresholds_nominal = {100, 100, 100, 30, 30, 30};
  request->upper_force_thresholds_nominal = {100, 100, 100, 30, 30, 30};
  request->lower_torque_thresholds_nominal = {100, 100, 100, 80, 80, 40, 40};
  request->upper_torque_thresholds_nominal = {100, 100, 100, 80, 80, 40, 40};

  std::vector<std::string> resources = resourceToVector(resource_);
  for (size_t i = 0; i < resources.size(); ++i) {
    std::string srv_name = ns + resources[i] + "_param_service_server/set_force_torque_collision_behavior";
    panda_controllers::ExecutorNode<SetForceTorqueCollisionBehavior> cli(srv_name);
    bool success = cli.callWithTimeout(request, response, 2.0);
    if (!success || (response && !response->success)) {
      RCLCPP_WARN(node_->get_logger(),
        "SimpleReflexEngine: Could not set hardware collision thresholds for resource %s (expected in simulation).",
        resources[i].c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(),
        "SimpleReflexEngine: Successfully configured hardware FT collision thresholds for resource %s.",
        resources[i].c_str());
    }
  }

  loadParameters(resource);

  if (!contact_checker_.init(robot_data, node, resource)) {
    RCLCPP_ERROR(node_->get_logger(), "SimpleReflexEngine: failed to initialize contact checker");
    return false;
  }
  
  reflex_time_ = node_->now();
  return true;
}

void SimpleReflexEngine::startImpl(panda_controllers::PandaControllerInterface* control) {
  control->start();
  // Don't arbitrarily reset state if we are actively managing a reflex or recovery sequence
  if (state_ == SimpleState::NORMAL) {
    state_ = SimpleState::NORMAL;
  }
}

void SimpleReflexEngine::stopImpl(panda_controllers::PandaControllerInterface* control) {
  control->stop();
}

bool SimpleReflexEngine::recoverServiceCallback(const std_srvs::srv::Empty::Request::SharedPtr req,
                                                std_srvs::srv::Empty::Response::SharedPtr res) {
  if (state_ == SimpleState::WAIT_FOR_RECOVERY) {
    recovery_requested_ = true;
    RCLCPP_INFO(node_->get_logger(), "SimpleReflexEngine: Recovery requested via service call.");
  } else {
    RCLCPP_WARN(node_->get_logger(), "SimpleReflexEngine: Recovery requested, but engine is not waiting for recovery.");
  }
  return true;
}

void SimpleReflexEngine::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
                                        panda_controllers::PandaControllerInterface* control) {
  contact_checker_.updateThresholdsFromBuffer();
  
  switch (state_) {
    case SimpleState::NORMAL: {
      checkHazards(control);
      control->computeTau(tau);
      applySoftVelocityDamping(tau);
      break;
    }
    case SimpleState::REFLEX:
    case SimpleState::WAIT_FOR_RECOVERY: {
      // 1. Joint Position limits check
      bool joint_pos_limits_violated = checkJointLimits(joint_limit_margin_);
      if (joint_pos_limits_violated) {
        // Apply heavy damping immediately to arrest motion during the controller switch latency!
        if (!robot_data_.empty()) {
          auto& dq = robot_data_[0]->state().dq;
          auto& coriolis = robot_data_[0]->coriolis();
          for (size_t i = 0; i < tau.size(); ++i) {
            for (size_t j = 0; j < 7; ++j) {
              double damping = (j < 4) ? 20.0 : ((j < 6) ? 20.0 : 10.0);
              (*tau[i])[j] = coriolis[j] - damping * dq[j];
            }
          }
        }
        
        if (state_ == SimpleState::WAIT_FOR_RECOVERY && control->getName().find("stop") == std::string::npos) {
          triggerReflex(reflex_for_joint_pos_limit_);
          auto pos_reasons = getJointLimitViolationReasons(joint_limit_margin_);
          std::string reason_str;
          for (size_t i = 0; i < pos_reasons.size(); ++i) {
            reason_str += pos_reasons[i];
            if (i < pos_reasons.size() - 1) reason_str += " | ";
          }
          RCLCPP_WARN(node_->get_logger(),
            "SimpleReflexEngine: Joint Position Limit Hazard Detected during WAIT_FOR_RECOVERY! Reason: [%s]. Triggering %s...", reason_str.c_str(), reflex_for_joint_pos_limit_.c_str());
        }
      }

      // 2. Joint Velocity hard limit check
      bool joint_vel_hard_limits_violated = checkJointVelocityLimits(joint_velocity_hard_buffer_);
      if (joint_vel_hard_limits_violated) {
        if (state_ == SimpleState::WAIT_FOR_RECOVERY && control->getName().find("stop") == std::string::npos) {
          triggerReflex(reflex_for_joint_vel_limit_);
          auto vel_reasons = getJointVelocityLimitViolationReasons(joint_velocity_hard_buffer_);
          std::string reason_str;
          for (size_t i = 0; i < vel_reasons.size(); ++i) {
            reason_str += vel_reasons[i];
            if (i < vel_reasons.size() - 1) reason_str += " | ";
          }
          RCLCPP_WARN(node_->get_logger(),
            "SimpleReflexEngine: Joint Velocity Hard Limit Hazard Detected during WAIT_FOR_RECOVERY! Reason: [%s]. Triggering %s...", reason_str.c_str(), reflex_for_joint_vel_limit_.c_str());
        }
      }

      // Check if the formal controller manager has successfully swapped out the nominal controller
      bool is_stop = control->getName().find("stop") != std::string::npos;
      bool is_retract = control->getName().find("retract") != std::string::npos;
      bool is_zero_g = control->getName().find("zero_g") != std::string::npos;

      if (is_stop || is_retract || is_zero_g) {

        bool allowed_to_run = true;
        // Position limits or hard velocity limits block non-stop controllers
        if ((joint_pos_limits_violated || joint_vel_hard_limits_violated) && !is_stop) {
          allowed_to_run = false;
        }

        // Handover: Let the formal safety controller compute the torques natively and apply soft velocity damping
        if (allowed_to_run) {
          control->computeTau(tau);
          applySoftVelocityDamping(tau);
        }
        
        if (state_ == SimpleState::REFLEX && allowed_to_run) {
          state_ = SimpleState::WAIT_FOR_RECOVERY;
          publishState("WAIT_FOR_RECOVERY");
          RCLCPP_INFO(node_->get_logger(), "SimpleReflexEngine: Safety controller active. Waiting for recovery signal...");
        } else if (state_ == SimpleState::WAIT_FOR_RECOVERY) {
          // In WAIT_FOR_RECOVERY, monitor for haptic tap or service call to resume
          bool haptic_ready = (node_->now() - reflex_time_).seconds() > recovery_cooldown_time_;
          if (recovery_requested_) {
            recovery_requested_ = false;
            triggerRecovery(); // Service call triggers move_to_start
          } else if (haptic_ready && checkHapticRecovery()) {
            RCLCPP_INFO(node_->get_logger(), "SimpleReflexEngine: Haptic recovery requested.");
            triggerRecovery();
          }
        }
      } else {
        // Intercept: The SetControllers service hasn't finished yet (takes a few ms).
        // Guarantee instant safety by locally applying heavy damping to stop motion.
        if (!robot_data_.empty()) {
          auto& dq = robot_data_[0]->state().dq;
          auto& coriolis = robot_data_[0]->coriolis();
          for (size_t i = 0; i < tau.size(); ++i) {
            for (size_t j = 0; j < 7; ++j) {
              double damping = (j < 4) ? 50.0 : ((j < 6) ? 20.0 : 10.0);
              (*tau[i])[j] = coriolis[j] - damping * dq[j];
            }
          }
        }
      }
      break;
    }
    case SimpleState::RECOVERY: {
      loadParameters(resource_); // Refresh recovery_goal_position and nominal_controller parameters dynamically
      // 1. Joint Position limits check during RECOVERY
      bool joint_pos_limits_violated = checkJointLimits(joint_limit_margin_recovery_);
      if (joint_pos_limits_violated) {
        for (size_t i = 0; i < tau.size(); ++i) {
          for (size_t j = 0; j < 7; ++j) {
            (*tau[i])[j] = 0.0;
          }
        }

        if (control->getName().find("stop") == std::string::npos) {
          triggerReflex(reflex_for_joint_pos_limit_);
          auto pos_reasons = getJointLimitViolationReasons(joint_limit_margin_recovery_);
          std::string reason_str;
          for (size_t i = 0; i < pos_reasons.size(); ++i) {
            reason_str += pos_reasons[i];
            if (i < pos_reasons.size() - 1) reason_str += " | ";
          }
          RCLCPP_ERROR(node_->get_logger(),
            "SimpleReflexEngine: Joint Position Limit Hazard Detected during RECOVERY! Reason: [%s]. Aborting recovery and triggering %s...", reason_str.c_str(), reflex_for_joint_pos_limit_.c_str());
        }
        break;
      }

      // 2. Joint Velocity hard limit check during RECOVERY
      bool joint_vel_hard_limits_violated = checkJointVelocityLimits(joint_velocity_hard_buffer_);
      if (joint_vel_hard_limits_violated) {
        if (control->getName().find("stop") == std::string::npos) {
          triggerReflex(reflex_for_joint_vel_limit_);
          auto vel_reasons = getJointVelocityLimitViolationReasons(joint_velocity_hard_buffer_);
          std::string reason_str;
          for (size_t i = 0; i < vel_reasons.size(); ++i) {
            reason_str += vel_reasons[i];
            if (i < vel_reasons.size() - 1) reason_str += " | ";
          }
          RCLCPP_ERROR(node_->get_logger(),
            "SimpleReflexEngine: Joint Velocity Hard Limit Hazard Detected during RECOVERY! Reason: [%s]. Aborting recovery and triggering %s...", reason_str.c_str(), reflex_for_joint_vel_limit_.c_str());
        }
        break;
      }

      // Nominal controller or move_to_start controller is active.
      control->computeTau(tau);
      applySoftVelocityDamping(tau);
      
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
        "SimpleReflexEngine: In RECOVERY. Active control: %s", control->getName().c_str());

      // Check if we are using the move_to_start_controller and have reached the goal
      if (control->getName().find("move_to_start") != std::string::npos) {
        if (!robot_data_.empty()) {
          auto& q = robot_data_[0]->state().q;
          auto& dq = robot_data_[0]->state().dq;
          double err = 0;
          double dq_err = 0;
          for (int i = 0; i < 7; ++i) {
            err += std::abs(q[i] - q_start_[i]);
            dq_err += std::abs(dq[i]);
          }

          if (err < 0.25 && dq_err < 0.1) { // Within 0.25 rad total error and settled
            RCLCPP_INFO(node_->get_logger(), "SimpleReflexEngine: Reached start pose. Switching to nominal controller (%s)...", nominal_controller_.c_str());
            state_ = SimpleState::NORMAL;
            publishState("NORMAL");
            
            auto request = std::make_shared<multi_mode_control_msgs::srv::SetControllers::Request>();
            multi_mode_control_msgs::msg::Controller new_ctrl;
            new_ctrl.name = nominal_controller_;
            new_ctrl.resources.push_back(resource_);
            request->controllers.push_back(new_ctrl);
            
            if (set_controllers_client_->service_is_ready()) {
              set_controllers_client_->async_send_request(request);
            }
          }
        }
      } else {
        // If nominal controller is already active, we are back to normal
        if (control->getName().find(nominal_controller_) != std::string::npos) {
          state_ = SimpleState::NORMAL;
          publishState("NORMAL");
          RCLCPP_INFO(node_->get_logger(), "SimpleReflexEngine: Recovery complete, resuming NORMAL operations.");
        }
      }
      break;
    }
  }
}

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

void SimpleReflexEngine::loadParameters(const std::string& resource) {
  if (!node_) return;

  resource_ = resource;
  std::string prefix = resource + ".";

  // 1. Joint Position limits & buffers
  std::vector<double> default_q_min(q_min_.begin(), q_min_.end());
  std::vector<double> q_min_vec = declare_or_get(node_, prefix + "joint_position_limits.q_min", default_q_min);
  if (q_min_vec.size() == 7) std::copy(q_min_vec.begin(), q_min_vec.end(), q_min_.begin());

  std::vector<double> default_q_max(q_max_.begin(), q_max_.end());
  std::vector<double> q_max_vec = declare_or_get(node_, prefix + "joint_position_limits.q_max", default_q_max);
  if (q_max_vec.size() == 7) std::copy(q_max_vec.begin(), q_max_vec.end(), q_max_.begin());

  joint_limit_margin_ = declare_or_get(node_, prefix + "joint_position_buffers.joint_limit_buffer", joint_limit_margin_);
  joint_limit_margin_recovery_ = declare_or_get(node_, prefix + "joint_position_buffers.recovery_limit_buffer", joint_limit_margin_recovery_);

  // 2. Joint Velocity limits & buffers
  std::vector<double> default_qd_max(qd_max_.begin(), qd_max_.end());
  std::vector<double> qd_max_vec = declare_or_get(node_, prefix + "joint_velocity_limits.qd_max", default_qd_max);
  if (qd_max_vec.size() == 7) std::copy(qd_max_vec.begin(), qd_max_vec.end(), qd_max_.begin());

  joint_velocity_soft_buffer_ = declare_or_get(node_, prefix + "joint_velocity_limits.joint_velocity_soft_buffer", joint_velocity_soft_buffer_);
  joint_velocity_hard_buffer_ = declare_or_get(node_, prefix + "joint_velocity_limits.joint_velocity_hard_buffer", joint_velocity_hard_buffer_);

  std::vector<double> default_gains(joint_velocity_damping_gains_.begin(), joint_velocity_damping_gains_.end());
  std::vector<double> gains_vec = declare_or_get(node_, prefix + "joint_velocity_limits.joint_velocity_damping_gains", default_gains);
  if (gains_vec.size() == 7) std::copy(gains_vec.begin(), gains_vec.end(), joint_velocity_damping_gains_.begin());

  // 3. Dynamic Reflex controllers
  reflex_for_collision_ = declare_or_get(node_, prefix + "reflexes.reflex_for_collision", reflex_for_collision_);
  reflex_for_joint_pos_limit_ = declare_or_get(node_, prefix + "reflexes.reflex_for_joint_pos_limit", reflex_for_joint_pos_limit_);
  reflex_for_joint_vel_limit_ = declare_or_get(node_, prefix + "reflexes.reflex_for_joint_vel_limit", reflex_for_joint_vel_limit_);

  // 4. Engine behavior settings
  verbose_logging_ = declare_or_get(node_, prefix + "verbose_logging", verbose_logging_);
  enable_gesture_recovery_ = declare_or_get(node_, prefix + "enable_gesture_recovery", enable_gesture_recovery_);
  gesture_threshold_ = declare_or_get(node_, prefix + "gesture_threshold", gesture_threshold_);
  recovery_cooldown_time_ = declare_or_get(node_, prefix + "recovery_cooldown_time", recovery_cooldown_time_);
  nominal_controller_ = declare_or_get(node_, prefix + "nominal_controller", nominal_controller_);

  std::vector<double> default_q_start(q_start_.begin(), q_start_.end());
  std::vector<double> q_start_vec = declare_or_get(node_, prefix + "recovery_goal_position", default_q_start);
  if (q_start_vec.size() == 7) std::copy(q_start_vec.begin(), q_start_vec.end(), q_start_.begin());

  RCLCPP_INFO(node_->get_logger(),
    "SimpleReflexEngine: Loaded parameters for resource '%s'. (Collision reflex: %s, PosLimit reflex: %s, VelLimit reflex: %s, Verbose: %s, GestureRecovery: %s)",
    resource.c_str(), reflex_for_collision_.c_str(), reflex_for_joint_pos_limit_.c_str(), reflex_for_joint_vel_limit_.c_str(),
    verbose_logging_ ? "true" : "false", enable_gesture_recovery_ ? "true" : "false");
}

void SimpleReflexEngine::checkHazards(panda_controllers::PandaControllerInterface* control) {
  // Dynamically re-sync runtime safety parameters if updated via ROS 2 parameter service
  if (node_ && !resource_.empty()) {
    std::string prefix = resource_ + ".";
    node_->get_parameter(prefix + "reflexes.reflex_for_collision", reflex_for_collision_);
    node_->get_parameter(prefix + "reflexes.reflex_for_joint_pos_limit", reflex_for_joint_pos_limit_);
    node_->get_parameter(prefix + "reflexes.reflex_for_joint_vel_limit", reflex_for_joint_vel_limit_);
    node_->get_parameter(prefix + "nominal_controller", nominal_controller_);
    std::vector<double> q_start_vec;
    if (node_->get_parameter(prefix + "recovery_goal_position", q_start_vec) && q_start_vec.size() == 7) {
      std::copy(q_start_vec.begin(), q_start_vec.end(), q_start_.begin());
    }
  }
  // 1. Joint Position Limits have the highest priority and trigger configured pos limit reflex
  if (checkJointLimits(joint_limit_margin_)) {
    triggerReflex(reflex_for_joint_pos_limit_); 
    auto limit_reasons = getJointLimitViolationReasons(joint_limit_margin_);
    std::string reason_str;
    for (size_t i = 0; i < limit_reasons.size(); ++i) {
      reason_str += limit_reasons[i];
      if (i < limit_reasons.size() - 1) reason_str += " | ";
    }
    RCLCPP_WARN(node_->get_logger(), "SimpleReflexEngine: Joint Position Limit Hazard Detected! Reason: [%s]. Triggering %s...",
                reason_str.c_str(), reflex_for_joint_pos_limit_.c_str());
    return; // Don't process contacts if we are stopping due to limits
  }

  // 2. Joint Velocity Hard Limit Hazard triggers configured vel limit reflex before hardware trip
  if (checkJointVelocityLimits(joint_velocity_hard_buffer_)) {
    triggerReflex(reflex_for_joint_vel_limit_);
    auto vel_reasons = getJointVelocityLimitViolationReasons(joint_velocity_hard_buffer_);
    std::string reason_str;
    for (size_t i = 0; i < vel_reasons.size(); ++i) {
      reason_str += vel_reasons[i];
      if (i < vel_reasons.size() - 1) reason_str += " | ";
    }
    RCLCPP_WARN(node_->get_logger(), "SimpleReflexEngine: Joint Velocity Hard Limit Hazard Detected! Reason: [%s]. Triggering %s...",
                reason_str.c_str(), reflex_for_joint_vel_limit_.c_str());
    return;
  }

  // 3. Contact Hazards trigger configured collision reflex
  if (checkContact()) {
    triggerReflex(reflex_for_collision_);
    auto reasons = contact_checker_.getLevel2ContactReasons();
    std::string reason_str;
    for (size_t i = 0; i < reasons.size(); ++i) {
      reason_str += reasons[i];
      if (i < reasons.size() - 1) reason_str += " | ";
    }
    RCLCPP_WARN(node_->get_logger(), "SimpleReflexEngine: Contact Hazard Detected! Reason: [%s]. Triggering %s...",
                reason_str.c_str(), reflex_for_collision_.c_str());
  }
}

bool SimpleReflexEngine::checkContact() {
  return contact_checker_.hasLevel2Contact();
}

bool SimpleReflexEngine::checkJointLimits(double margin) {
  if (robot_data_.empty()) return false;
  auto& q = robot_data_[0]->state().q;
  auto& dq = robot_data_[0]->state().dq;

  
  for (int i = 0; i < 7; ++i) {
    if ((q[i] > q_max_[i] - margin && dq[i] > 0) ||
        (q[i] < q_min_[i] + margin && dq[i] < 0)) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> SimpleReflexEngine::getJointLimitViolationReasons(double margin) {
  std::vector<std::string> reasons;
  if (robot_data_.empty()) return reasons;
  auto& q = robot_data_[0]->state().q;
  auto& dq = robot_data_[0]->state().dq;

  for (int i = 0; i < 7; ++i) {
    if (q[i] > q_max_[i] - margin && dq[i] > 0) {
      char buf[128];
      snprintf(buf, sizeof(buf), "Joint %d upper limit hazard (q=%.3f rad > limit %.3f rad)",
               i + 1, q[i], q_max_[i] - margin);
      reasons.push_back(buf);
    } else if (q[i] < q_min_[i] + margin && dq[i] < 0) {
      char buf[128];
      snprintf(buf, sizeof(buf), "Joint %d lower limit hazard (q=%.3f rad < limit %.3f rad)",
               i + 1, q[i], q_min_[i] + margin);
      reasons.push_back(buf);
    }
  }
  return reasons;
}

bool SimpleReflexEngine::checkJointVelocityLimits(double buffer) {
  if (robot_data_.empty()) return false;
  auto& dq = robot_data_[0]->state().dq;

  for (int i = 0; i < 7; ++i) {
    if (std::abs(dq[i]) > qd_max_[i] - buffer) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> SimpleReflexEngine::getJointVelocityLimitViolationReasons(double buffer) {
  std::vector<std::string> reasons;
  if (robot_data_.empty()) return reasons;
  auto& dq = robot_data_[0]->state().dq;

  for (int i = 0; i < 7; ++i) {
    if (std::abs(dq[i]) > qd_max_[i] - buffer) {
      char buf[128];
      snprintf(buf, sizeof(buf), "Joint %d velocity hazard (|dq|=%.3f rad/s > limit-buffer %.3f rad/s)",
               i + 1, std::abs(dq[i]), qd_max_[i] - buffer);
      reasons.push_back(buf);
    }
  }
  return reasons;
}

void SimpleReflexEngine::applySoftVelocityDamping(const std::vector<std::array<double, 7>*>& tau) {
  if (robot_data_.empty()) return;
  auto& dq = robot_data_[0]->state().dq;
  for (size_t r = 0; r < tau.size(); ++r) {
    for (int j = 0; j < 7; ++j) {
      double overspeed = std::abs(dq[j]) - (qd_max_[j] - joint_velocity_soft_buffer_);
      if (overspeed > 0.0) {
        if (verbose_logging_) {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
            "SimpleReflexEngine: Joint %d velocity overspeed detected (dq=%.3f rad/s > limit-buffer %.3f rad/s). Applying damping.",
            j+1, std::abs(dq[j]), qd_max_[j] - joint_velocity_soft_buffer_);
        }
        double sign = (dq[j] > 0.0) ? 1.0 : -1.0;
        double damping_tau = -sign * joint_velocity_damping_gains_[j] * overspeed;
        (*tau[r])[j] += damping_tau;
      }
    }
  }
}

bool SimpleReflexEngine::checkHapticRecovery() {
  if (!enable_gesture_recovery_) return false;
  if (contact_checker_.hasGesture()) {
     RCLCPP_INFO(node_->get_logger(), "SimpleReflexEngine: Recovery requested via haptic tap.");
     return true;
  }
  return false;
}

void SimpleReflexEngine::triggerReflex(const std::string& safety_controller) {
  state_ = SimpleState::REFLEX;
  publishState("REFLEX");
  reflex_time_ = node_->now();
  
  RCLCPP_WARN(node_->get_logger(),
    "SimpleReflexEngine: Reflex triggered. Switching to safety controller: %s",
    safety_controller.c_str());

  auto request = std::make_shared<multi_mode_control_msgs::srv::SetControllers::Request>();
  multi_mode_control_msgs::msg::Controller new_ctrl;
  new_ctrl.name = safety_controller;
  new_ctrl.resources.push_back(resource_);
  request->controllers.push_back(new_ctrl);
  
  if (set_controllers_client_->service_is_ready()) {
    set_controllers_client_->async_send_request(request);
  } else {
    RCLCPP_WARN(node_->get_logger(), "SimpleReflexEngine: set_controllers service not ready!");
  }
}

void SimpleReflexEngine::triggerRecovery() {
  state_ = SimpleState::RECOVERY;
  publishState("RECOVERY");
  RCLCPP_INFO(node_->get_logger(), "SimpleReflexEngine: Initiating recovery motion to start pose...");
  
  auto request = std::make_shared<multi_mode_control_msgs::srv::SetControllers::Request>();
  multi_mode_control_msgs::msg::Controller new_ctrl;
  new_ctrl.name = "panda_move_to_start_controller";
  new_ctrl.resources.push_back(resource_);
  request->controllers.push_back(new_ctrl);
  
  if (set_controllers_client_->service_is_ready()) {
    set_controllers_client_->async_send_request(request);
  }
}

void SimpleReflexEngine::returnToNormal() {
  state_ = SimpleState::NORMAL;
  publishState("NORMAL");
}

void SimpleReflexEngine::publishState(const std::string& state_name) {
  if (state_publisher_) {
    std_msgs::msg::String msg;
    msg.data = state_name;
    state_publisher_->publish(msg);
  }
}

void SimpleReflexEngine::getRobotStatesCallback(
    const multi_mode_control_msgs::srv::GetRobotStates::Request::SharedPtr,
    multi_mode_control_msgs::srv::GetRobotStates::Response::SharedPtr res) {
  for (const auto& rd : robot_data_) {
    res->states.push_back(getRobotStateMsg(rd->state()));
  }
}

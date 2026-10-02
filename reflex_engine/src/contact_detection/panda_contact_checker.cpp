#include <reflex_engine/contact_detection/panda_contact_checker.h>
#include <reflex_engine/contact_detection/utils.h>

#include <rclcpp/duration.hpp>

#include <reflex_controllers/utils/conversions.h>
// #include <reflex_controllers/utils/get_panda_interfaces.h>

using namespace reflex_engine;
using panda_controllers::RobotData;
using Eigen::Matrix3d;
using Matrix67d = Eigen::Matrix<double, 6, 7>;
using Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

PandaContactChecker::PandaContactChecker(){};

void PandaContactChecker::calculateContactWrench() {
  if (!calculated_wrench_) {
    contact_wrench_ = Vector6d::Zero();
    for (int i=0;i<robot_data_.size();++i) {
      // Todo: take into account reference point for torque calculation
      contact_wrench_ += getContactWrenchB(robot_data_[i]->eeZeroJacobian(),
                                           robot_data_[i]->state(), B_R_0_[i]);
    }
    calculated_wrench_ = true;
  }
}

bool PandaContactChecker::checkGesture() {
  if (!checked_gesture_) {
    checked_gesture_ = true;
    has_gesture_ = false;
    calculateContactWrench();
    if (contact_wrench_.head(3).norm() > gesture_threshold_) {
      has_gesture_ = true;
    }
  }
  return has_gesture_;
}

const Eigen::Matrix<double, 6, 1>& PandaContactChecker::getContactWrench() {
  calculateContactWrench();
  return contact_wrench_;
}

bool PandaContactChecker::init(const std::vector<RobotData*>& robot_data,
  rclcpp_lifecycle::LifecycleNode::SharedPtr nh, const std::string& resource) {
  node_ = nh; // TODO: should we make a second node?
  
  last_gesture_ = node_->now();
  robot_data_ = robot_data;
  std::string detection_frame = "panda_link8";
  if (!node_->get_parameter(resource + ".contact_detection.detection_frame", detection_frame)) {
    node_->get_parameter("contact_detection.detection_frame", detection_frame);
  }

  std::vector<std::string> resources = resourceToVector(resource);
  for (int i=0;i<resources.size();++i) {
    r_B_0_.push_back(Vector3d::Zero());
    B_R_0_.push_back(Matrix3d::Identity());
    
    std::string base_frame = resources[i] + "_link0";
    std::string ee_frame = resources[i] + "_link8";
    node_->get_parameter(resource + ".base_frame", base_frame);
    node_->get_parameter(resource + ".ee_frame", ee_frame);

    // Fast path to avoid TF lookups (and timeouts) during controller initialization for known frames
    if (detection_frame == base_frame) {
      B_R_0_[i] = Matrix3d::Identity();
    } else if (detection_frame == ee_frame && i < robot_data_.size() && robot_data_[i] != nullptr) {
      Eigen::Matrix4d O_T_EE(robot_data_[i]->state().O_T_EE.data());
      B_R_0_[i] = O_T_EE.topLeftCorner(3,3).transpose();
      if (B_R_0_[i].isZero(1e-3)) {
        B_R_0_[i] = Eigen::Matrix3d::Identity();
      }
    } else {
      if (!getTransform(node_, base_frame, detection_frame, r_B_0_[i], B_R_0_[i])) {
        RCLCPP_ERROR(node_->get_logger(), "PandaContactChecker: Failed to get transform to detection "
                  "frame %s for resource %s.", detection_frame.c_str(),
                  resources[i].c_str());
        return false;
      }
    }
  }
  // Load task perserving (level 1) collision threshold from ROS 2 parameters if available
  // TBD: not implemented
  Vector7d tau_max;
  ContactThresholds level1_thresholds, level2_thresholds;
  tau_max << 4, 4, 3, 3, 1, 1, 0.5;
  level1_thresholds.abs_mode = true;
  level1_thresholds.directional_mode = false;
  level1_thresholds.torque_mode = robot_data_.size() == 1 ? true : false;
  level1_thresholds.f_abs_max = 10;
  level1_thresholds.m_abs_max = 3;
  for (int i=0;i<robot_data_.size();++i) {
    level1_thresholds.tau_max.push_back(tau_max);
    level1_thresholds.tau_min.push_back(-tau_max);
  }

  // Load task abandoning (level2) collision threshold from ROS 2 parameters if available
  tau_max << 12, 12, 9, 9, 3, 3, 1.5;
  std::vector<double> tau_max_param;
  if (node_->get_parameter(resource + ".collision_thresholds.tau_max", tau_max_param) && tau_max_param.size() == 7) {
    for (int j = 0; j < 7; ++j) tau_max[j] = tau_max_param[j];
  }
  level2_thresholds.abs_mode = true;
  level2_thresholds.directional_mode = false;
  level2_thresholds.torque_mode = robot_data_.size() == 1 ? true : false;
  
  for (int i=0;i<robot_data_.size();++i) {
    level2_thresholds.tau_max.push_back(tau_max);
    level2_thresholds.tau_min.push_back(-tau_max);
  }

  // Load collision thresholds of the detection frame (usually EE) from ROS 2 parameters if available
  double f_max_param = 30.0;
  double w_max_param = 9.0;
  node_->get_parameter(resource + ".collision_thresholds.F_max_EE", f_max_param);
  node_->get_parameter(resource + ".collision_thresholds.W_max_EE", w_max_param);
  level2_thresholds.f_abs_max = f_max_param;
  level2_thresholds.m_abs_max = w_max_param;

  // Load gesture threshold for recovery from ROS 2 parameters if available
  double gesture_thresh = 20.0;
  if (!node_->get_parameter(resource + ".gesture_threshold", gesture_thresh)) {
    node_->get_parameter("gesture_threshold", gesture_thresh);
  }
  gesture_threshold_.setBuffer(gesture_thresh);
  level1_thresholds_.setBuffer(level1_thresholds);
  level2_thresholds_.setBuffer(level2_thresholds);
  return true;
}

bool PandaContactChecker::hasContact(const ContactThresholds& th) {
  if (th.abs_mode) {
    if (contact_wrench_.head(3).norm() > th.f_abs_max ||
        contact_wrench_.tail(3).norm() > th.m_abs_max) {
      return true;
    }
  }
  if (th.directional_mode) {
    if (checkThresholds(th.F_min, th.F_max, contact_wrench_)) {
      return true;
    }
  }
  if (th.torque_mode) {
    for (size_t i=0;i<robot_data_.size();++i) {
      if (checkThresholds(th.tau_min[i], th.tau_max[i],
          Eigen::Map<const Vector7d>(
          robot_data_[i]->state().tau_ext_hat_filtered.data()))) {
        return true;
      }
    }
  }
  return false;
}

std::vector<std::string> PandaContactChecker::getContactReasons(const ContactThresholds& th) {
  std::vector<std::string> reasons;
  calculateContactWrench();

  if (th.abs_mode) {
    double f_norm = contact_wrench_.head(3).norm();
    if (f_norm > th.f_abs_max) {
      char buf[128];
      snprintf(buf, sizeof(buf), "EE force magnitude (%.2f N > max %.2f N)", f_norm, th.f_abs_max);
      reasons.push_back(buf);
    }
    double m_norm = contact_wrench_.tail(3).norm();
    if (m_norm > th.m_abs_max) {
      char buf[128];
      snprintf(buf, sizeof(buf), "EE moment magnitude (%.2f Nm > max %.2f Nm)", m_norm, th.m_abs_max);
      reasons.push_back(buf);
    }
  }

  if (th.directional_mode) {
    for (int i = 0; i < 6; ++i) {
      if (contact_wrench_[i] > th.F_max[i]) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Directional EE wrench axis %d (%.2f > max %.2f)", i, contact_wrench_[i], th.F_max[i]);
        reasons.push_back(buf);
      } else if (contact_wrench_[i] < th.F_min[i]) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Directional EE wrench axis %d (%.2f < min %.2f)", i, contact_wrench_[i], th.F_min[i]);
        reasons.push_back(buf);
      }
    }
  }

  if (th.torque_mode) {
    for (size_t i = 0; i < robot_data_.size(); ++i) {
      Eigen::Map<const Vector7d> tau_ext(robot_data_[i]->state().tau_ext_hat_filtered.data());
      for (int j = 0; j < 7; ++j) {
        if (tau_ext[j] > th.tau_max[i][j]) {
          char buf[128];
          snprintf(buf, sizeof(buf), "Joint %d torque (%.2f Nm > max %.2f Nm)", j + 1, tau_ext[j], th.tau_max[i][j]);
          reasons.push_back(buf);
        } else if (tau_ext[j] < th.tau_min[i][j]) {
          char buf[128];
          snprintf(buf, sizeof(buf), "Joint %d torque (%.2f Nm < min %.2f Nm)", j + 1, tau_ext[j], th.tau_min[i][j]);
          reasons.push_back(buf);
        }
      }
    }
  }

  return reasons;
}

bool PandaContactChecker::hasLevel1Contact() {
  if (!checked_level1_) {
    calculateContactWrench();
    checked_level1_ = true;
    level1_reasons_ = getContactReasons(level1_thresholds_);
    has_level1_contact_ = !level1_reasons_.empty();
  }
  return has_level1_contact_;
}

bool PandaContactChecker::hasLevel2Contact() {
  if (!checked_level2_) {
    calculateContactWrench();
    checked_level2_ = true;
    level2_reasons_ = getContactReasons(level2_thresholds_);
    has_level2_contact_ = !level2_reasons_.empty();
  }
  return has_level2_contact_;
}

const std::vector<std::string>& PandaContactChecker::getLevel1ContactReasons() const {
  return level1_reasons_;
}

const std::vector<std::string>& PandaContactChecker::getLevel2ContactReasons() const {
  return level2_reasons_;
}

bool PandaContactChecker::hasGesture() {
  if (node_->now() - last_gesture_ > rclcpp::Duration(0, 2e8)) {
    checkGesture();
    if (has_gesture_) {
      last_gesture_ = node_->now();
    }
    return has_gesture_;
  }
  return false;
}

void PandaContactChecker::setGestureThresholdBuffered(const double& t) {
  gesture_threshold_.setBuffer(t);
}

void PandaContactChecker::setLevel1ThresholdsBuffered(
    const ContactThresholds& t) {
  level1_thresholds_.setBuffer(t);
}

void PandaContactChecker::setLevel2ThresholdsBuffered(
    const ContactThresholds& t) {
  level2_thresholds_.setBuffer(t);
}

void PandaContactChecker::updateThresholdsFromBuffer() {
  gesture_threshold_.update();
  level1_thresholds_.update();
  level2_thresholds_.update();
  calculated_wrench_ = false;
  checked_level1_ = false;
  checked_level2_ = false;
  checked_gesture_ = false;
  level1_reasons_.clear();
  level2_reasons_.clear();
}

#include <reflex_controllers/controllers/panda_super_zero_g_controller.h>

// #include <joint_limits_interface/joint_limits_urdf.h>
// #include <urdf/model.h>

#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>

using namespace panda_controllers;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Params = PandaSuperZeroGControllerParams;
using Pose = PandaSuperZeroGControllerPose;
using Controller = PandaSuperZeroGController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "panda_super_zero_g_controller");

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& /*desired*/, const Params& p) {
  Pose current = getCurrentPose();
  Vector7d tau_ext(robot_data_[0]->state().tau_ext_hat_filtered.data());
  Vector7d tau_ctrl = -p.K_r*tau_ext;
  static const std::array<double, 7> pd_width =
      {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
  static const std::array<double, 7> d_width =
      {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
  static const std::array<double, 7> P =
      {{2000.0, 2000.0, 1000.0, 1000.0, 500.0, 200.0, 200.0}};
  static const std::array<double, 7> D =
      {{30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}};
  for (uint64_t i = 0; i < 7; ++i) {
    double d_limit_max;
    double d_limit_min;
    if (moving_wall_[i]) {
      d_limit_max = upper_soft_limit_[i] - pd_width[i] - d_width[i];
      d_limit_min = lower_soft_limit_[i] + pd_width[i] + d_width[i];
      if (current.q[i] > d_limit_min && current.q[i] < d_limit_max) {
        moving_wall_[i] = false;
        wall_scale_[i] = 1;
      } else if (current.q[i] < d_limit_min && current.qD[i] > 0) {
        wall_scale_[i] = (d_limit_min-current.q[i]) / (pd_width[i]+d_width[i]);
      } else if (current.q[i] > d_limit_max && current.qD[i] < 0) {
        wall_scale_[i] = (current.q[i]-d_limit_max) / (pd_width[i]+d_width[i]);
      }
    }
    double pd_limit_max = upper_soft_limit_[i] - wall_scale_[i] * pd_width[i];
    double pd_limit_min = lower_soft_limit_[i] + wall_scale_[i] * pd_width[i];
    d_limit_max = upper_soft_limit_[i] -
                  wall_scale_[i] * (pd_width[i] + d_width[i]);
    d_limit_min = lower_soft_limit_[i] +
                  wall_scale_[i] * (pd_width[i] + d_width[i]);
    if (current.q[i] > pd_limit_max) {
      (*tau[0])[i] = -D[i] * current.qD[i] + P[i] * (pd_limit_max-current.q[i]);
    } else if (current.q[i] < pd_limit_min) {
      (*tau[0])[i] = -D[i] * current.qD[i] + P[i] * (pd_limit_min-current.q[i]);
    } else if (current.q[i] > d_limit_max) {
      (*tau[0])[i] = -D[i] * current.qD[i] +
          tau_ctrl[i]*(current.q[i]-pd_limit_max)/(d_limit_max - pd_limit_max);
    } else if (current.q[i] < d_limit_min) {
      (*tau[0])[i] = -D[i] * current.qD[i] +
          tau_ctrl[i]*(current.q[i]-pd_limit_min)/(d_limit_min - pd_limit_min);
    } else {
      (*tau[0])[i] = tau_ctrl[i];
    }
  }
}

Params Controller::defaultParameters() {
  Params p;
  p.K_r = Matrix7d::Identity();
  return p;
}

Pose Controller::getCurrentPoseImpl() {
  Pose current;
  current.q = Vector7d(robot_data_[0]->state().q.data());
  current.qD = Vector7d(robot_data_[0]->state().dq.data());
  return current;
}

bool Controller::initImpl(const std::vector<RobotData*>& robot_data,
    rclcpp_lifecycle::LifecycleNode::SharedPtr& nh, std::string name, std::string resource) {
  std::vector<std::string> joints;
  // try{
  //   if (!nh->get_parameter("joints." + resource, joints)) {
  //     RCLCPP_ERROR(nh->get_logger(), "PandaZeroGController: Could not get joints for robot %s from "
  //               "parameter server.", resource.c_str());
  //     return false;
  //   }
  // }
  // catch(std::exception& e){
  //   RCLCPP_ERROR(nh->get_logger(), "PandaZeroGController: Could not get joints for robot %s from "
  //       "parameter server due to an exception: %s.", resource.c_str(), e.what());
  //   return false;
  // }
  // Patch for Humble due to lack of joint_limits_urdf.h
  // Limits from https://frankaemika.github.io/docs/control_parameters.html
  upper_soft_limit_ = std::array<double, 7>{2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973};
  lower_soft_limit_ = std::array<double, 7>{-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973};
  // urdf::Model urdf_model;
  // std::string full_ns = nh.getNamespace();
  // std::size_t last = full_ns.find_last_of("/");
  // std::string description_ns;
  // if (last!=std::string::npos) {
  //   description_ns = full_ns.substr(0, last) + "/robot_description";
  // } else {
  //   description_ns = full_ns;
  // }
  // if (nh.hasParam(description_ns)) {
  //   if (!urdf_model.initParamWithNodeHandle(description_ns, nh)) {
  //     ROS_ERROR_STREAM("PandaSuperZeroGController: Could not initialize urdf "
  //                      "model from robot_description (namespace: " <<
  //                      description_ns << ").");
  //     return false;
  //   }
  //   joint_limits_interface::SoftJointLimits soft_limits;
  //   std::size_t i = 0;
  //   for (const auto& joint : joints) {
  //     auto urdf_joint = urdf_model.getJoint(joint);
  //     if (!urdf_joint) {
  //       ROS_ERROR_STREAM("PandaSuperZeroGController: Could not get joint "
  //                        << joint << " from urdf.");
  //       return false;
  //     }
  //     if (!urdf_joint->safety) {
  //       ROS_ERROR_STREAM("PandaSuperZeroGController: Joint " << joint
  //                        << " has no limits.");
  //       return false;
  //     }
  //     if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
  //       upper_soft_limit_[i] = soft_limits.max_position;
  //       lower_soft_limit_[i++] = soft_limits.min_position;
  //     } else {
  //       ROS_ERROR_STREAM("PandaSuperZeroGController: Could not parse joint "
  //                        "limit for joint " << joint << " from joint limit "
  //                        "interfaces.");
  //       return false;
  //     }
  //   }
  // } else {
  //   ROS_ERROR_STREAM("PandaSuperZeroGController: No parameter robot_description"
  //                   " (namespace: " << description_ns <<
  //                   ") found to set joint limits.");
  //   return false;
  // }
  return true;
}

void Controller::startImpl() {
  moving_wall_ = {true, true, true, true, true, true, true};
  wall_scale_ = {1, 1, 1, 1, 1, 1, 1};
}

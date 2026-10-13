#include <reflex_controllers/reflexes/franka_super_zero_g_controller.h>

#include <reflex_controllers/utils/controller_factory.h>
#include <reflex_controllers/utils/damping_design.h>

using namespace panda_controllers;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Params = FrankaSuperZeroGControllerParams;
using Pose = FrankaSuperZeroGControllerPose;
using Controller = FrankaSuperZeroGController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "franka_super_zero_g_controller");

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
  p.K_r = 0.5*Matrix7d::Identity();
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
  upper_soft_limit_ = std::array<double, 7>{2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973};
  lower_soft_limit_ = std::array<double, 7>{-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973};
  return true;
}

void Controller::startImpl() {
  moving_wall_ = {true, true, true, true, true, true, true};
  wall_scale_ = {1, 1, 1, 1, 1, 1, 1};
}

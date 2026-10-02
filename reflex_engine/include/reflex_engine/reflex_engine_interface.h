#pragma once

#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <reflex_controllers/base/panda_controller_interface.h>

namespace reflex_engine {

class ReflexEngineInterface {
 public:
  void computeTau(const std::vector<std::array<double, 7>*>& tau,
                  panda_controllers::PandaControllerInterface* control) {
    return computeTauImpl(tau, control);
  };
  bool init(const std::vector<panda_controllers::RobotData*>& robot_data,
            rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string name, std::string resource) {
    return initImpl(robot_data, node, name, resource);
  };
  void start(panda_controllers::PandaControllerInterface* control) {
    return startImpl(control);
  };
  void stop(panda_controllers::PandaControllerInterface* control) {
    return stopImpl(control);
  }
 private:
  virtual void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      panda_controllers::PandaControllerInterface* control) = 0;
  virtual bool initImpl(
      const std::vector<panda_controllers::RobotData*>& robot_data,
      rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string name, std::string resource) = 0;
  virtual void startImpl(
      panda_controllers::PandaControllerInterface* control) = 0;
  virtual void stopImpl(
      panda_controllers::PandaControllerInterface* control) = 0;
  rclcpp::Logger get_my_logger(){return rclcpp::get_logger("reflex_engine");};
};

}

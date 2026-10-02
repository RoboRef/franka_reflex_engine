#pragma once

#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <reflex_engine/reflex_engine_interface.h>
#include <reflex_controllers/utils/get_robot_state_msg.h>
#include <franka/robot_state.h>

namespace reflex_engine {

  using multi_mode_control_msgs::srv::GetRobotStates;
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

class NoReflexEngine : public ReflexEngineInterface {
 private:

  std::vector<panda_controllers::RobotData*> robot_data_;
  rclcpp::Service<GetRobotStates>::SharedPtr get_robot_states_server_;

  virtual void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      panda_controllers::PandaControllerInterface* control) override {
    return control->computeTau(tau);
  }

  bool getRobotStatesCallback(
        const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const GetRobotStates::Request::SharedPtr& /*req*/,
        const GetRobotStates::Response::SharedPtr& res) {
    for (const auto& rd : robot_data_) {
      res->states.push_back(getRobotStateMsg(rd->state()));
    }
    return true;
  }
  virtual bool initImpl(
      const std::vector<panda_controllers::RobotData*>& robot_data,
      rclcpp_lifecycle::LifecycleNode::SharedPtr node, 
      std::string name, 
      std::string resource) override {
    robot_data_ = robot_data;
    std::string topic_prefix = resource;
    std::size_t pos = topic_prefix.find("&");
    while (pos != std::string::npos) {
      topic_prefix.replace(pos, 1, "_and_");
      pos = topic_prefix.find("&");
    }
    
    get_robot_states_server_ = node->create_service<GetRobotStates>(
      topic_prefix + "/get_robot_states",
      std::bind(&NoReflexEngine::getRobotStatesCallback, this, _1, _2, _3));
    return true;
  }
  virtual void startImpl(
      panda_controllers::PandaControllerInterface* control) override {
    return control->start();
  }
  virtual void stopImpl(
      panda_controllers::PandaControllerInterface* control) override {
    return control->stop();
  }
};

}

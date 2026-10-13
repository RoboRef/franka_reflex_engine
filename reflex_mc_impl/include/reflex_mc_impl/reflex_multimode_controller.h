// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <mutex>
#include <shared_mutex>
#include <utility>
#include <vector>
#include <map>
#include <memory>
#include <array>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

// panda_ros2 includes
#include "franka_msgs/msg/franka_state.hpp"
#include "franka_msgs/msg/errors.hpp"

// multi_mode_controller includes
#include "reflex_controllers/utils/robot_data.h"
#include "reflex_controllers/utils/conversions.h"
#include "reflex_engine/simple_reflex_engine.h"
#include "reflex_controllers/base/panda_controller_interface.h"
#include "multi_mode_control_msgs/srv/get_controllers.hpp"
#include "multi_mode_control_msgs/srv/set_controllers.hpp"
#include "reflex_controllers/utils/controller_factory.h"
#include "task_controllers/task_controllers.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace reflex_multimode_controller {   

using panda_controllers::PandaControllerInterface;

/**
 * The core reflex multimodal controller plugin for ros2_control.
 */
class ReflexMultimodeController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  struct ArmContainer {
    std::string arm_id_;
    std::array<double, 7> tau_;
    int resource_usage_;
  };

  int num_robots;
  const int num_joints = 7;

  // multi_mode_controller classes
  struct ControllerPtrs {
    PandaControllerInterface* control;
    std::vector<std::array<double, 7>*> tau;
    reflex_engine::ReflexEngineInterface* switcher;
  };
  struct ControllerInfo {
    std::string name;
    std::string resource;
  };
  struct Controller {
    ControllerPtrs ptrs;
    ControllerInfo info;
  };

  std::map<std::string, size_t> robot_index_;
  std::vector<panda_controllers::RobotData> robot_data_;
  std::vector<ArmContainer> arms_;
  
  std::map<std::string, 
            std::map<std::string, std::unique_ptr<PandaControllerInterface>>>
            control_map_; // control_map_[controller][resource] 

  std::map<std::string, std::vector<panda_controllers::RobotData*>> rd_map_;
  std::map<std::string, std::unique_ptr<reflex_engine::ReflexEngineInterface>>
      switch_engines_;
  std::map<std::string, std::vector<std::array<double, 7>*>> tau_map_;

  std::vector<Controller> active_control_set_;
  std::shared_mutex control_mtx_;

  rclcpp::Service<multi_mode_control_msgs::srv::SetControllers>::SharedPtr set_ctrl_srv_;
  rclcpp::Service<multi_mode_control_msgs::srv::GetControllers>::SharedPtr get_ctrl_srv_;

  // loop functions
  void updateFrankaState();
  
  // custom initializers
  bool initServices();
  bool initControllers();

  // Utilities
  std::unique_ptr<std::vector<Controller>> getControlSetFromInfo(
      const std::vector<ControllerInfo>& info);
  void printControlSet();

  std::unique_ptr<std::vector<Controller>> setupControlSet(
        const std::vector<ControllerInfo>& info);

  // Service callbacks
  bool setControllersCallback(const multi_mode_control_msgs::srv::SetControllers::Request::SharedPtr req, 
                              const multi_mode_control_msgs::srv::SetControllers::Response::SharedPtr res);

  bool getControllersCallback(const multi_mode_control_msgs::srv::GetControllers::Request::SharedPtr req, 
                              const multi_mode_control_msgs::srv::GetControllers::Response::SharedPtr res);

  // helper function for parameters
  std::vector<std::string> get_string_array_param(std::string param) {
    if (get_node()->has_parameter(param)) {
      return get_node()->get_parameter(param).as_string_array();
    }
    return std::vector<std::string>();
  }                              
};

}  // namespace reflex_multimode_controller

namespace reflex_mc_impl {
  using ReflexMultimodeController = reflex_multimode_controller::ReflexMultimodeController;
}

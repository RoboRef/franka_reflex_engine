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

#include <reflex_mc_impl/reflex_multimode_controller.h>
#include <reflex_engine/simple_reflex_engine.h>
#include <task_controllers/task_controllers.h>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <algorithm>
#include <sstream>

#include <Eigen/Eigen>

using namespace panda_controllers;

namespace reflex_multimode_controller {

using panda_controllers::PandaControllerInterface;
using SetControllers = multi_mode_control_msgs::srv::SetControllers;
using GetControllers = multi_mode_control_msgs::srv::GetControllers;


//*************** ros2_control overrides *************//
controller_interface::InterfaceConfiguration
ReflexMultimodeController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto& arm : arms_) {
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm.arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
ReflexMultimodeController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto& arm : arms_) {
    config.names.push_back(arm.arm_id_ + "/robot_state");
    config.names.push_back(arm.arm_id_ + "/robot_model");
  }
  return config;
}

controller_interface::return_type ReflexMultimodeController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateFrankaState();
  std::shared_lock<std::shared_mutex> lock(control_mtx_);
  for (auto& control : active_control_set_) {
    control.ptrs.switcher->computeTau(control.ptrs.tau, control.ptrs.control);
  }

  int k = 0;
  for (auto& arm : arms_) {
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[k].set_value(arm.tau_[i]);
      k++;
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn ReflexMultimodeController::on_init() {
  task_controllers::init();

  try {
    rclcpp::Parameter arm_count;
    bool bHas_arm_count = get_node()->get_parameter("arm_count", arm_count);
    if (!bHas_arm_count) {
      fprintf(stderr, "Failed to get arm_count parameter. Make sure it's set in the yaml file.\n");
      return CallbackReturn::ERROR;
    }
    num_robots = arm_count.as_int();

  } catch (const std::exception& e) {
    fprintf(stderr, "Failed to get arm_count parameter. Make sure it's set in the yaml file.\n%s \n", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Finished initializing reflex multimodal controller for %d arms", num_robots);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ReflexMultimodeController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  task_controllers::init();
      
  for (int i = 1; i <= num_robots; i++) {
    try {
      std::string arm_id_param_name = "arm_" + std::to_string(i) + ".arm_id";
      rclcpp::Parameter arm_id_param = this->get_node()->get_parameter(arm_id_param_name);
      auto arm_id_name = arm_id_param.as_string();

      robot_index_.insert(std::make_pair(arm_id_name, i - 1));
      arms_.push_back(ArmContainer{arm_id_name, std::array<double, 7>(), 0});
      robot_data_.push_back(RobotData(arm_id_name));

      if (!robot_data_[robot_index_[arm_id_name]].init("/robot_model", "/robot_state")) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize the robot data for arm %s", arm_id_name.c_str());
        return CallbackReturn::ERROR;  
      }
      RCLCPP_INFO(get_node()->get_logger(), "Arm set for %s", arm_id_param.as_string().c_str());
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize the arm container for index %d: %s", i, e.what());
      return CallbackReturn::ERROR;
    }
  }

  initControllers();
  initServices();

  return CallbackReturn::SUCCESS;
}

bool ReflexMultimodeController::initServices() {
  set_ctrl_srv_ = this->get_node()->create_service<SetControllers>(
      "~/set_controllers", 
      std::bind(&ReflexMultimodeController::setControllersCallback, 
                this, 
                std::placeholders::_1, 
                std::placeholders::_2));

  get_ctrl_srv_ = this->get_node()->create_service<GetControllers>(
      "~/get_controllers",
      std::bind(&ReflexMultimodeController::getControllersCallback, 
                this, 
                std::placeholders::_1, 
                std::placeholders::_2));
  return true;
}

bool ReflexMultimodeController::initControllers() {
  std::vector<std::string> controllers = get_string_array_param("controllers");
  if (controllers.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: could not get controllers from parameter server.");
    return false;
  }

  for (const auto& controller : controllers) {
    std::vector<std::string> resources = get_string_array_param("resources." + controller);
    if (resources.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: Could not get resources for controller %s from parameter server.", controller.c_str());
      return false;
    }
    for (const auto& resource : resources) {
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "ReflexMultimodeController: Creating controller "
                      << controller << " for resource " << resource << ".");
      control_map_[controller][resource] = ControllerFactory::create(controller);
      if (switch_engines_.find(resource) == switch_engines_.end()) {
        switch_engines_[resource] = std::make_unique<reflex_engine::SimpleReflexEngine>();

        std::vector<std::string> resource_vector = resourceToVector(resource);
        std::vector<std::array<double, 7>*> tau_vec;
        std::vector<RobotData*> robot_data_vec;
        for (const auto& res : resource_vector) {
          tau_vec.push_back(&arms_[robot_index_[res]].tau_);
          robot_data_vec.push_back(&robot_data_[robot_index_[res]]);
        }
        rd_map_[resource] = robot_data_vec;
        tau_map_[resource] = tau_vec;
      }
    }
  }

  std::vector<std::string> start_controllers = get_string_array_param("start_controllers.names");
  if (start_controllers.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: could not get start_controllers from parameter server.");
    return false;
  }

  std::vector<ControllerInfo> new_control;
  for (const auto& controller : start_controllers) {
    ControllerInfo info;
    std::vector<std::string> resources = get_string_array_param("start_controllers.resources." + controller);
    if (resources.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: could not get resources for start_controller %s from parameter server.", controller.c_str());
      return false;
    }
    info.name = controller;
    for (const auto& resource : resources) {
      info.resource = std::move(resource);
      new_control.push_back(info);
    }
  }
  std::unique_ptr<std::vector<Controller>> new_control_set = getControlSetFromInfo(new_control);
  if (!new_control_set) {
    return false;
  }
  {
    std::lock_guard<std::shared_mutex> lock(control_mtx_);
    auto node_shared = get_node()->shared_from_this();
    for (auto& re : switch_engines_) {
      if (!re.second->init(rd_map_.at(re.first), node_shared, re.first + "_switch_engine", re.first)) {
        RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: Could not initialize switch_engine for resource %s.", re.first.c_str());
        return false;
      }
    }
    active_control_set_ = std::move(*new_control_set.release());
    for (const auto& control : control_map_) {
      for (const auto& resource : control.second) {
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "ReflexMultimodeController: Initializing controller "
                        << control.first << " for resource " << resource.first << ".");
        if (!resource.second->init(rd_map_.at(resource.first), node_shared, control.first, resource.first)) {
          return false;
        }
      }
    }
  }
  printControlSet();
  return true;
}

std::unique_ptr<std::vector<ReflexMultimodeController::Controller>> 
ReflexMultimodeController::getControlSetFromInfo(const std::vector<ControllerInfo>& info) {
  std::vector<ReflexMultimodeController::Controller> control_set;
  
  for (const auto& i : info) {
    Controller tmp;
    if (control_map_.find(i.name) == control_map_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: Requested Controller %s is not loaded.", i.name.c_str());
      return nullptr;
    } else if (control_map_.at(i.name).find(i.resource) != control_map_.at(i.name).end()) {
      tmp.info.name = i.name;
      tmp.info.resource = i.resource;
      tmp.ptrs.tau = tau_map_.at(i.resource);
      tmp.ptrs.control = control_map_.at(i.name).at(i.resource).get();
      tmp.ptrs.switcher = switch_engines_.at(i.resource).get();
      control_set.push_back(tmp);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: Requested Controller %s is not available for resource %s.", i.name.c_str(), i.resource.c_str());
      return nullptr;
    }
  }
  
  return std::make_unique<std::vector<ReflexMultimodeController::Controller>>(std::move(control_set));
}

void ReflexMultimodeController::printControlSet() {
  std::stringstream ss;
  ss << "ReflexMultimodeController: New active control set:";
  for (const auto& controller : active_control_set_) {
    ss << "\n" << controller.info.name << ": " << controller.info.resource;
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());
}

std::unique_ptr<std::vector<ReflexMultimodeController::Controller>> 
ReflexMultimodeController::setupControlSet(const std::vector<ControllerInfo>& info) {
  std::unique_ptr<std::vector<Controller>> out = getControlSetFromInfo(info);
  if (!out) {
    return nullptr;
  }
  for (const auto& old : active_control_set_) {
    bool add = true;
    for (const auto& nc : *out) {
      if (std::any_of(old.ptrs.tau.begin(), old.ptrs.tau.end(),
          [&](std::array<double, 7>* n) {
            return std::any_of(nc.ptrs.tau.begin(), nc.ptrs.tau.end(),
                               [&](std::array<double, 7>* o) {
              return n == o;
            });
          })) {
        add = false;
        break;
      }
    }
    if (add) {
      out->push_back(old);
    }
  }
  std::vector<int> resource_usage(arms_.size(), 0);
  for (const auto& nc : *out) {
    for (const auto& res : resourceToVector(nc.info.resource)) {
      ++resource_usage[robot_index_[res]];
    }
  }
  for (auto const& arm : arms_) {
    if (resource_usage[robot_index_[arm.arm_id_]] > 1) {
      RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: specified control has conflicting resource specification for resource %s.", arm.arm_id_.c_str());
      return nullptr;
    } else if (resource_usage[robot_index_[arm.arm_id_]] < 1) {
      RCLCPP_ERROR(get_node()->get_logger(), "ReflexMultimodeController: specified control does not control resource %s.", arm.arm_id_.c_str());
      return nullptr;
    }
  }
  return out;
}

bool ReflexMultimodeController::setControllersCallback(
    const SetControllers::Request::SharedPtr req, 
    __attribute_maybe_unused__ const SetControllers::Response::SharedPtr res) {
  std::vector<ControllerInfo> new_control;
  for (const auto& controller : req->controllers) {
    ControllerInfo info;
    info.name = controller.name;
    for (const auto& resource : controller.resources) {
      info.resource = std::move(resource);
      new_control.push_back(info);
    }
  }
  std::unique_ptr<std::vector<Controller>> new_control_set = setupControlSet(new_control);
  if (!new_control_set) {
    return false;
  }
  {
    std::lock_guard<std::shared_mutex> lock(control_mtx_);
    for (const auto& control : active_control_set_) {
      control.ptrs.switcher->stop(control.ptrs.control);
    }
    active_control_set_ = std::move(*new_control_set.release());
    for (const auto& control : active_control_set_) {
      control.ptrs.switcher->start(control.ptrs.control);
    }
  }
  printControlSet();
  return true;
}

bool ReflexMultimodeController::getControllersCallback(
    __attribute_maybe_unused__ GetControllers::Request::SharedPtr req, 
    const GetControllers::Response::SharedPtr res) {
  for (const auto& controller : active_control_set_) {
    multi_mode_control_msgs::msg::Controller c;
    c.name = controller.info.name;
    c.resources.push_back(controller.info.resource);
    res->controllers.push_back(c);
  }
  return true;
}

CallbackReturn ReflexMultimodeController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (auto& rd : robot_data_) {
    rd.activate_model(state_interfaces_);
  }
  updateFrankaState();
  for (const auto& control : active_control_set_) {
    control.ptrs.switcher->start(control.ptrs.control);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ReflexMultimodeController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (auto& rd : robot_data_) {
    rd.deactivate_model();
  }
  return CallbackReturn::SUCCESS;
}

void ReflexMultimodeController::updateFrankaState() {
  for (auto& rd : robot_data_) {
    rd.update();
  }
}

}  // namespace reflex_multimode_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(reflex_multimode_controller::ReflexMultimodeController,
                       controller_interface::ControllerInterface)

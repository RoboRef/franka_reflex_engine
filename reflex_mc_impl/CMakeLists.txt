cmake_minimum_required(VERSION 3.8)
project(reflex_mc_impl)

set(default_build_type "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(reflex_controllers REQUIRED)
find_package(task_controllers REQUIRED)
find_package(reflex_engine REQUIRED)
find_package(multi_mode_control_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/reflex_multimode_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)


ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  hardware_interface
  franka_semantic_components
  pluginlib
  rclcpp
  rclcpp_lifecycle
  reflex_controllers
  task_controllers
  reflex_engine
  multi_mode_control_msgs
  geometry_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(
  controller_interface reflex_mc_controllers.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  reflex_controllers
  task_controllers
  reflex_engine
  multi_mode_control_msgs
  geometry_msgs
  sensor_msgs
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(task_controllers)

set(default_build_type "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(multi_mode_control_msgs REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(reflex_controllers REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/link_anchor.cpp
  src/comless_franka_joint_impedance_controller.cpp
  src/comless_franka_cartesian_impedance_controller.cpp
  src/franka_joint_impedance_controller.cpp
  src/franka_cartesian_impedance_controller.cpp
  src/franka_joint_velocity_controller.cpp
  src/franka_move_to_start_controller.cpp
  src/comless_wiping_controller.cpp
  src/wiping_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)


ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  multi_mode_control_msgs
  franka_hardware
  franka_semantic_components
  reflex_controllers
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  reflex_controllers
  multi_mode_control_msgs
  rclcpp
)

ament_package()

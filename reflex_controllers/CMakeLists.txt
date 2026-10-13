cmake_minimum_required(VERSION 3.8)
project(reflex_controllers)

set(default_build_type "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Respect the build type from user and ide
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

set(THIS_PACKAGE_INCLUDE_DEPENDS
builtin_interfaces
control_msgs
geometry_msgs
multi_mode_control_msgs
controller_interface
controller_manager
franka_semantic_components
generate_parameter_library
pluginlib
rclcpp
rclcpp_lifecycle
rcutils
realtime_tools
sensor_msgs
Franka
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(reflex_controllers SHARED 
  # Framework core & utilities
  src/utils/robot_data.cpp
  src/utils/controller_factory.cpp
  src/utils/motion_generator.cpp

  # Reflex behaviours (Stop, Zero-G, Super Zero-G, Admittance, Joint Retract, Cartesian Retract)
  src/reflexes/franka_stop_controller.cpp
  src/reflexes/franka_zero_g_controller.cpp
  src/reflexes/franka_super_zero_g_controller.cpp
  src/reflexes/franka_admittance_controller.cpp
  src/reflexes/franka_joint_retract_controller.cpp
  src/reflexes/franka_cartesian_retract_controller.cpp
)

target_include_directories(
  reflex_controllers
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(reflex_controllers PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS reflex_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  reflex_controllers
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  franka_semantic_components
  rclcpp
  multi_mode_control_msgs
)
ament_package()

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
  # Set the possible values of build type for cmake-gui
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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(openGJK STATIC
    src/openGJK.c
)
target_include_directories(
  openGJK
  PUBLIC
  include
)

install(TARGETS
  openGJK
  DESTINATION lib
)

add_library(reflex_controllers SHARED 
  src/robot_data.cpp
  src/controller_factory.cpp
  src/redundancy_resolution.cpp
  src/comless_panda_cartesian_impedance_controller.cpp
  src/comless_coupled_dual_cartesian_impedance_controller.cpp
  src/comless_panda_joint_impedance_controller.cpp
  src/panda_stop_controller.cpp
  src/motion_generator.cpp
  src/panda_move_to_start_controller.cpp
  src/panda_cartesian_impedance_controller.cpp
  src/panda_joint_impedance_controller.cpp
  src/panda_collision_recorder_controller.cpp
  src/panda_joint_velocity_controller.cpp
  src/panda_contact_evasion_controller.cpp
  src/panda_super_zero_g_controller.cpp
  src/panda_zero_g_controller.cpp
  src/comless_dual_cartesian_impedance_controller.cpp
  src/comless_dual_joint_impedance_controller.cpp
  src/dual_cartesian_impedance_controller.cpp
  src/twin_cartesian_impedance_controller.cpp
  src/des_coupled_dual_cartesian_impedance_controller.cpp
  src/twin_joint_impedance_controller.cpp
  src/panda_joint_impedance_example_controller.cpp
  src/panda_cartesian_impedance_example_controller.cpp
  src/comless_wiping_controller.cpp
  src/wiping_controller.cpp
  src/comless_move_down_controller.cpp
  src/move_down_controller.cpp
  src/comless_move_joint1_controller.cpp
  )

target_include_directories(
  reflex_controllers
    PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(reflex_controllers PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(reflex_controllers PUBLIC 
                      openGJK)

install(
        TARGETS reflex_controllers
        DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

add_library(synchronous_client SHARED src/synchronous_client.cpp)
ament_target_dependencies(synchronous_client PUBLIC rclcpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  synchronous_client
   PRIVATE
   include
   ${EIGEN3_INCLUDE_DIRS}
)
install(TARGETS
  synchronous_client
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        reflex_controllers
        synchronous_client
        openGJK
)
ament_export_dependencies(
        franka_semantic_components
        rclcpp
)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(reflex_engine)

set(default_build_type "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(reflex_controllers REQUIRED)
find_package(multi_mode_control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)

#### Contact detection
add_library(contact_detection SHARED
  src/contact_detection/panda_contact_checker.cpp
)

target_include_directories(contact_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(contact_detection
        Franka::Franka
        Eigen3::Eigen
)

ament_target_dependencies(contact_detection
  franka_hardware
  reflex_controllers
  rclcpp_lifecycle
  rclcpp
  tf2_ros
  tf2_geometry_msgs
)

#### Reflex engine (SimpleReflexEngine only)
add_library(reflex_engine SHARED
  src/simple_reflex_engine.cpp
)

target_include_directories(reflex_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(reflex_engine
        Franka::Franka
        Eigen3::Eigen
        contact_detection
)

ament_target_dependencies(reflex_engine
  reflex_controllers
  rclcpp_lifecycle
  rclcpp
  std_msgs
  std_srvs
  multi_mode_control_msgs
  franka_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
        TARGETS contact_detection reflex_engine
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(contact_detection reflex_engine)
ament_export_dependencies(
  franka_hardware
  rclcpp_lifecycle
  franka_msgs
  hardware_interface
  reflex_controllers
  multi_mode_control_msgs
  rclcpp
  std_srvs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_package()

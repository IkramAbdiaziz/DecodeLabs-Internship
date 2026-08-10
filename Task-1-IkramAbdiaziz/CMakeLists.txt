cmake_minimum_required(VERSION 3.8)
project(robotic_arm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executable
add_executable(kinematics_planner_node src/kinematics_planner_node.cpp)
ament_target_dependencies(kinematics_planner_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  trac_ik_lib
  control_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  sensor_msgs
  geometry_msgs
)

# Install rules
install(TARGETS
  kinematics_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

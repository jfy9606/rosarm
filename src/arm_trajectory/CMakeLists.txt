cmake_minimum_required(VERSION 3.0.2)
project(arm_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  tf2
  tf2_ros
)

add_message_files(
  FILES
  TrajectoryPoint.msg
  TrajectoryPath.msg
)

add_service_files(
  FILES
  PlanTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/trajectory_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 
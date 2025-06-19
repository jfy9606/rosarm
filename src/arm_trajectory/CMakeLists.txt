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

# 注释掉不存在的C++节点
# add_executable(trajectory_planner_node src/trajectory_planner_node.cpp)
# target_link_libraries(trajectory_planner_node ${catkin_LIBRARIES})
# add_dependencies(trajectory_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/whales_optimizer.py
  scripts/trajectory_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 
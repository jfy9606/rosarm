cmake_minimum_required(VERSION 3.0.2)
project(arm_3d_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  servo_wrist
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_3d_control
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs servo_wrist
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_3d_control_node src/arm_3d_control_node.cpp)
target_link_libraries(arm_3d_control_node ${catkin_LIBRARIES})
add_dependencies(arm_3d_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(position_sender src/position_sender_example.cpp)
target_link_libraries(position_sender ${catkin_LIBRARIES})
add_dependencies(position_sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
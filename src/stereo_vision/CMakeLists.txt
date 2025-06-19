cmake_minimum_required(VERSION 3.0.2)
project(stereo_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  geometry_msgs
  tf
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ObjectDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport message_runtime geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/stereo_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 
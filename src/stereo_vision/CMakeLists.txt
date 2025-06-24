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
  tf2_ros
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs cv_bridge image_transport geometry_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/camera_node.py
  scripts/camera_direct.py
  scripts/camera_test.py
  scripts/yolo_detector.py
  scripts/yolo_controller.py
  scripts/depth_estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 
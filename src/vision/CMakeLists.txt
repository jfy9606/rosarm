cmake_minimum_required(VERSION 3.0.2)
project(vision)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
  tf2_ros
)

# Set up Python modules
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Detection.msg
  ObjectDetection.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetViewMode.srv
  DetectionControl.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport message_runtime tf2_ros
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/detector_node.py
  scripts/stereo_node.py
  scripts/camera_node.py
  scripts/view_node.py
  scripts/camera_control.py
  scripts/detector_control.py
  scripts/stereo_control.py
  scripts/view_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
) 

## Mark rviz files for installation
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
) 
cmake_minimum_required(VERSION 3.0.2)
project(vision)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
  eigen_conversions
  cmake_modules
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

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
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport message_runtime eigen_conversions tf2_ros
  DEPENDS OpenCV EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare C++ libraries
add_library(${PROJECT_NAME}_camera_control
  src/camera_control.cpp
)
add_dependencies(${PROJECT_NAME}_camera_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_camera_control
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(${PROJECT_NAME}_view_control
  src/view_control.cpp
)
add_dependencies(${PROJECT_NAME}_view_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_view_control
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_camera_config
  src/camera_config.cpp
)
add_dependencies(${PROJECT_NAME}_camera_config ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_camera_config
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Declare C++ executables
add_executable(camera_node src/camera_node.cpp)
add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_node
  ${PROJECT_NAME}_camera_control
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(view_node src/view_node.cpp)
add_dependencies(view_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(view_node
  ${PROJECT_NAME}_view_control
  ${catkin_LIBRARIES}
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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
install(TARGETS camera_node view_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS 
  ${PROJECT_NAME}_camera_control
  ${PROJECT_NAME}_view_control
  ${PROJECT_NAME}_camera_config
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
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

## Mark rviz files for installation
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
) 
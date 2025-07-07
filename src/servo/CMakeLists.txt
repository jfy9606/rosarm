cmake_minimum_required(VERSION 3.0.2)
project(servo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  motor
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorOrder.msg
  SerControl.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  JointControl.srv
  HomePosition.srv
  VacuumCmd.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs message_runtime motor
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src/SCServo_Linux
)

## 添加SCServo库
add_library(scservo
  src/SCServo_Linux/SCS.cpp
  src/SCServo_Linux/SCSCL.cpp
  src/SCServo_Linux/SMSCL.cpp
  src/SCServo_Linux/SMSBL.cpp
  src/SCServo_Linux/SMS_STS.cpp
  src/SCServo_Linux/SCSerial.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/wrist_control.cpp
  src/vacuum_control.cpp
  src/arm_control.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable - wrist node
add_executable(wrist_node src/wrist_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(wrist_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(wrist_node
  ${PROJECT_NAME}
  scservo
  ${catkin_LIBRARIES}
)

## Declare a C++ executable - vacuum node
add_executable(vacuum_node src/vacuum_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(vacuum_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(vacuum_node
  ${PROJECT_NAME}
  scservo
  ${catkin_LIBRARIES}
)

## Declare a C++ executable - arm node
add_executable(arm_node src/arm_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(arm_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ${PROJECT_NAME} wrist_node vacuum_node arm_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wrist_servo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

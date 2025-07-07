cmake_minimum_required(VERSION 3.0.2)
project(motor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## 生成消息
add_message_files(
  FILES
  MotorOrder.msg
)

## 生成依赖
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motor
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 添加motor库
add_library(motor
  src/motor_control.cpp
)
add_dependencies(motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor ${catkin_LIBRARIES})

# 添加可执行文件
add_executable(motor_node src/main.cpp src/motor_node.cpp)
add_dependencies(motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_node motor ${catkin_LIBRARIES})

# 安装
install(TARGETS ${PROJECT_NAME} motor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
) 
cmake_minimum_required(VERSION 3.10...3.25)
project(servo)

# 默认C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 CMP0148 策略以解决 Python 查找警告
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SerControl.msg"
  "srv/JointControl.srv"
  "srv/VacuumCmd.srv"
  DEPENDENCIES std_msgs
)

# Feetech SDK 源文件
set(FEETECH_SDK_SOURCES
  src/feetech_sdk/port_handler.cpp
  src/feetech_sdk/packet_handler.cpp
  src/feetech_sdk/group_sync_read.cpp
  src/feetech_sdk/group_sync_write.cpp
)

# 添加库
add_library(servo_control
  src/servo_control.cpp
  src/feetech_adapter.cpp
  ${FEETECH_SDK_SOURCES}
)

target_include_directories(servo_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${serial_INCLUDE_DIRS}
)

target_link_libraries(servo_control
  ${serial_LIBRARIES}
  nlohmann_json::nlohmann_json
)

# 添加ROS节点
add_executable(wrist_node
  src/wrist_node.cpp
)

target_include_directories(wrist_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(wrist_node
  servo_control
)

ament_target_dependencies(wrist_node
  rclcpp
  std_msgs
)

rosidl_target_interfaces(wrist_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(vacuum_node
  src/vacuum_node.cpp
)

target_include_directories(vacuum_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(vacuum_node
  servo_control
)

ament_target_dependencies(vacuum_node
  rclcpp
  std_msgs
)

rosidl_target_interfaces(vacuum_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 安装可执行文件
install(
  TARGETS servo_control wrist_node vacuum_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()

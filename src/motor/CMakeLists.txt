cmake_minimum_required(VERSION 3.10...3.25)
project(motor)

# 默认C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(serial REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 包含目录
include_directories(
  include
  ${serial_INCLUDE_DIRS}
)

# 生成接口 - 使用显式相对路径
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorOrder.msg"
  "srv/MotorControl.srv"
  DEPENDENCIES std_msgs
)

# 获取生成接口的包含目录
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 创建电机控制库
add_library(motor_control SHARED
  src/motor_control.cpp
)
target_include_directories(motor_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_control
  rclcpp
  std_msgs
)
target_link_libraries(motor_control
  ${serial_LIBRARIES}
  yaml-cpp
  "${cpp_typesupport_target}"
)

# 创建电机控制节点
add_library(motor_node SHARED
  src/motor_node.cpp
)
target_include_directories(motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motor_node
  motor_control
)
ament_target_dependencies(motor_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  ament_index_cpp
)
rclcpp_components_register_node(motor_node
  PLUGIN "motor_control::MotorNode"
  EXECUTABLE motor_node_main
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  rosidl_default_runtime
  ament_index_cpp
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库
install(
  TARGETS motor_control motor_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装启动文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 导出库目标
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(motor_control motor_node)

ament_package() 
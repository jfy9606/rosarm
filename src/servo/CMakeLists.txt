cmake_minimum_required(VERSION 3.8)
project(servo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motor REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp_components REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SerControl.msg"
  "srv/JointControl.srv"
  "srv/HomePosition.srv"
  "srv/VacuumCmd.srv"
  "srv/MotorControl.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Get the include directories for generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
)

# Create servo control library
add_library(servo_control SHARED
  src/servo_control.cpp
)
target_include_directories(servo_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(servo_control
  serial
)
ament_target_dependencies(servo_control
  rclcpp
  std_msgs
)

# Create wrist node as a component library
add_library(wrist_node SHARED
  src/wrist_node.cpp
)
target_include_directories(wrist_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wrist_node
  servo_control
  "${cpp_typesupport_target}"
)
ament_target_dependencies(wrist_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
)
rclcpp_components_register_node(wrist_node
  PLUGIN "servo_control::WristNode"
  EXECUTABLE wrist_node_exec
)

# Create vacuum node as a component library
add_library(vacuum_node SHARED
  src/vacuum_node.cpp
)
target_include_directories(vacuum_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vacuum_node
  servo_control
  "${cpp_typesupport_target}"
)
ament_target_dependencies(vacuum_node
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(vacuum_node
  PLUGIN "servo_control::VacuumNode"
  EXECUTABLE vacuum_node_exec
)

# Create wrist node standalone executable
add_executable(wrist_node_main
  src/wrist_node_main.cpp
)
target_link_libraries(wrist_node_main
  wrist_node
)

# Create vacuum node standalone executable
add_executable(vacuum_node_main
  src/vacuum_node_main.cpp
)
target_link_libraries(vacuum_node_main
  vacuum_node
)

# Install
install(TARGETS 
  servo_control
  wrist_node wrist_node_exec wrist_node_main
  vacuum_node vacuum_node_exec vacuum_node_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(servo_control wrist_node vacuum_node)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  motor
  serial
  rclcpp_components
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

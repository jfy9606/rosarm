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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motor REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SerControl.msg"
  "srv/HomePosition.srv"
  "srv/JointControl.srv"
  "srv/MotorControl.srv"
  "srv/VacuumCmd.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(
  include
)

# Create servo libraries
add_library(servo_control src/servo_control.cpp)
ament_target_dependencies(servo_control
  rclcpp
  std_msgs
)

add_library(vacuum_control src/vacuum_control.cpp)
ament_target_dependencies(vacuum_control
  rclcpp
  std_msgs
)

add_library(arm_control src/arm_control.cpp)
ament_target_dependencies(arm_control
  rclcpp
  std_msgs
  geometry_msgs
  motor
)

# Create executables
add_executable(wrist_node src/wrist_node.cpp)
ament_target_dependencies(wrist_node
  rclcpp
  std_msgs
)
target_link_libraries(wrist_node servo_control)

add_executable(vacuum_node src/vacuum_node.cpp)
ament_target_dependencies(vacuum_node
  rclcpp
  std_msgs
)
target_link_libraries(vacuum_node vacuum_control)

add_executable(arm_node src/arm_node.cpp)
ament_target_dependencies(arm_node
  rclcpp
  std_msgs
  geometry_msgs
  motor
)
target_link_libraries(arm_node arm_control)

# Install
install(TARGETS 
  servo_control
  vacuum_control
  arm_control
  wrist_node
  vacuum_node
  arm_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py" PATTERN "*.launch.xml" PATTERN "*.yaml"
)

ament_export_include_directories(include)
ament_export_libraries(servo_control vacuum_control arm_control)
ament_export_dependencies(rclcpp std_msgs geometry_msgs motor)

ament_package()

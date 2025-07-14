cmake_minimum_required(VERSION 3.8)
project(motor)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp_components REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorOrder.msg"
  DEPENDENCIES std_msgs
)

# Get the include directories for generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
)

# Create motor library
add_library(motor_control SHARED
  src/motor_control.cpp
)
target_include_directories(motor_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motor_control
  serial
)
ament_target_dependencies(motor_control
  rclcpp
  std_msgs
)

# Create node as a component library
add_library(motor_node SHARED
  src/motor_node.cpp
)
target_include_directories(motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motor_node
  motor_control
  "${cpp_typesupport_target}"
)
ament_target_dependencies(motor_node
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_node(motor_node
  PLUGIN "motor_control::MotorNode"
  EXECUTABLE motor_node_exec
)

# Create standalone executable
add_executable(motor_node_main
  src/main.cpp
)
target_link_libraries(motor_node_main
  motor_node
)

# Install
install(TARGETS motor_control motor_node motor_node_exec motor_node_main
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
ament_export_libraries(motor_control motor_node)
ament_export_dependencies(rclcpp std_msgs serial rclcpp_components)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package() 
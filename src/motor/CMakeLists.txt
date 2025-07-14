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

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorOrder.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
)

# Create motor library
add_library(motor
  src/motor_control.cpp
)
ament_target_dependencies(motor
  rclcpp
  std_msgs
)

# Create executable
add_executable(motor_node src/main.cpp src/motor_node.cpp)
ament_target_dependencies(motor_node
  rclcpp
  std_msgs
)
target_link_libraries(motor_node motor)

# Install
install(TARGETS motor motor_node
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
ament_export_libraries(motor)
ament_export_dependencies(rclcpp std_msgs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package() 
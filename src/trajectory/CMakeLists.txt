cmake_minimum_required(VERSION 3.8)
project(trajectory)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(motor REQUIRED)
find_package(servo REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ForwardKinematics.srv"
  "srv/InverseKinematics.srv"
  "srv/PlanTrajectory.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Include directories
include_directories(
  include
)

# Install Python modules
ament_python_install_package(scripts)

# Install Python executables
install(PROGRAMS
  scripts/trajectory_node.py
  scripts/path_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py" PATTERN "*.launch.xml" PATTERN "*.yaml"
)

ament_export_dependencies(rclcpp rclpy std_msgs geometry_msgs sensor_msgs motor servo)

ament_package()

cmake_minimum_required(VERSION 3.10...3.25)
project(gui)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(servo REQUIRED)
find_package(motor REQUIRED)
find_package(trajectory REQUIRED)
find_package(Python3 REQUIRED)

# Include directories
include_directories(
  include
)

# Install Python executables
install(
  PROGRAMS
  scripts/tkgui_main.py
  scripts/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(
  python_src
  PACKAGE_DIR src/gui_python
)

# Install resource files
install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclpy
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  servo
  motor
  trajectory
)

ament_package() 
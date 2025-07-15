cmake_minimum_required(VERSION 3.10...3.25)
project(vision)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/ObjectDetection.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Include directories
include_directories(
  include
)

# Install Python modules
# 避免目标名冲突
set(PYTHON_PACKAGE_NAME ${PROJECT_NAME}_py)
ament_python_install_package(${PYTHON_PACKAGE_NAME} PACKAGE_DIR ${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/vision_node.py
  scripts/object_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RVIZ config (注释掉，因为目录不存在)
# install(DIRECTORY rviz
#  DESTINATION share/${PROJECT_NAME}
# )

ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  serial
)

ament_package() 
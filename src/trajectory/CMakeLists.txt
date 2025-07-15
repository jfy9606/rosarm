cmake_minimum_required(VERSION 3.10...3.25)
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
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ForwardKinematics.srv"
  "srv/InverseKinematics.srv"
  "srv/PlanTrajectory.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Get the include directories for generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Create kinematics library
add_library(kinematics SHARED
  src/kinematics.cpp
)
target_include_directories(kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics
  ${EIGEN3_LIBRARIES}
  "${cpp_typesupport_target}"
)
ament_target_dependencies(kinematics
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Create kinematics node as a component library
add_library(kinematics_node SHARED
  src/kinematics_node.cpp
)
target_include_directories(kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics_node
  kinematics
  "${cpp_typesupport_target}"
)
ament_target_dependencies(kinematics_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
)
rclcpp_components_register_node(kinematics_node
  PLUGIN "kinematics::KinematicsNode"
  EXECUTABLE kinematics_node_exec
)

# Create kinematics node standalone executable
add_executable(kinematics_node_main
  src/kinematics_node_main.cpp
)
target_link_libraries(kinematics_node_main
  kinematics_node
)

# Install
install(TARGETS 
  kinematics
  kinematics_node
  kinematics_node_exec
  kinematics_node_main
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

# Install Python modules
# 避免目标名冲突
set(PYTHON_PACKAGE_NAME ${PROJECT_NAME}_py)
ament_python_install_package(${PYTHON_PACKAGE_NAME} PACKAGE_DIR ${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/trajectory_node.py
  scripts/path_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(kinematics kinematics_node)
ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  sensor_msgs
  rclcpp_components
  Eigen3
)
ament_export_targets()

ament_package()

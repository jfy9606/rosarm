cmake_minimum_required(VERSION 3.8)
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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(servo REQUIRED)
find_package(motor REQUIRED)
find_package(trajectory REQUIRED)

# Include Qt resource files
set(QT_RESOURCES_FILE
  resources/resources.qrc
)
qt5_add_resources(QT_RESOURCES ${QT_RESOURCES_FILE})

# Include directories
include_directories(
  include
)

# Create GUI library
add_library(${PROJECT_NAME}
  src/main_window.cpp
  src/joint_state_publisher.cpp
  src/control_panel.cpp
  ${QT_RESOURCES}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  servo
  motor
  trajectory
)

# Create main window executable
add_executable(main_window
  src/main_window_node.cpp
)
target_link_libraries(main_window
  ${PROJECT_NAME}
)

# Create joint state publisher node
add_executable(joint_state_publisher_node
  src/joint_state_publisher_node.cpp
)
target_link_libraries(joint_state_publisher_node
  ${PROJECT_NAME}
)

# Install
install(TARGETS
  ${PROJECT_NAME}
  main_window
  joint_state_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  servo
  motor
  trajectory
)

ament_package()

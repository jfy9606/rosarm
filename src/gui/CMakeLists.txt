cmake_minimum_required(VERSION 3.10...3.25)
project(gui)

# 默认C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# 检查Python版本
if(Python3_VERSION VERSION_LESS 3.11)
  message(WARNING "Python 3.11 or newer recommended (found: ${Python3_VERSION})")
else()
  message(STATUS "Using Python ${Python3_VERSION}")
endif()

# 包含目录
include_directories(
  include
)

# 安装Python可执行文件
install(
  PROGRAMS
  scripts/tkgui_main.py
  scripts/joint_state_publisher.py
  scripts/test_tkinter.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python模块
ament_python_install_package(
  python_src
  PACKAGE_DIR src/gui_python
)

# 安装资源文件
install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装包含文件
install(DIRECTORY include/
  DESTINATION include
)

# 导出依赖
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
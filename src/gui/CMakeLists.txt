cmake_minimum_required(VERSION 3.0.2)
project(gui)

# 设置CMake策略
cmake_policy(SET CMP0071 NEW)  # 避免moc警告
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)  # 避免OpenGL GLVND警告
endif()

# Qt设置
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# 编译选项
add_compile_options(-std=c++14)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  servo
  trajectory
)

# 查找Qt5
find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)

# 系统依赖
find_package(OpenGL REQUIRED)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    cv_bridge
    image_transport
    servo
    trajectory
  DEPENDS Qt5Core Qt5Gui Qt5Widgets Qt5OpenGL OPENGL
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OPENGL_INCLUDE_DIRS}
)

# 设置源文件
set(MAIN_WINDOW_SOURCES
  src/main_window.cpp
)

set(MAIN_SOURCES
  src/main.cpp
)

# GUI可执行文件 - 分开编译main_window和main.cpp
add_library(main_window_lib
  src/main_window.cpp
  include/main_window.h
)

target_link_libraries(main_window_lib
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL
  ${OPENGL_LIBRARIES}
)

# 主可执行文件
add_executable(main_window ${MAIN_SOURCES})
target_link_libraries(main_window
  main_window_lib
)

# 关节状态发布节点
add_executable(joint_state_publisher_node 
  src/joint_state_publisher_node.cpp
)

target_link_libraries(joint_state_publisher_node
  ${catkin_LIBRARIES}
)

# 安装目标
install(TARGETS main_window main_window_lib joint_state_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

install(DIRECTORY ui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ui
  FILES_MATCHING PATTERN "*.ui"
)

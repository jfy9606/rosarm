cmake_minimum_required(VERSION 3.0.2)
project(arm_gui)

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
  moveit_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  liancheng_socket
  servo_wrist
)

# 查找Qt5
find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)

# 系统依赖
find_package(OpenGL REQUIRED)

# 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    moveit_msgs
    tf2 
    tf2_ros
    liancheng_socket
    servo_wrist
  DEPENDS Qt5Core Qt5Gui Qt5Widgets Qt5OpenGL
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
)

# 资源文件
qt5_add_resources(QT_RESOURCES_CPP
  resources/arm_gui_resources.qrc
)

# 界面文件
qt5_wrap_ui(QT_UI_CPP
  ui/arm_control_main.ui
)

# 库文件
add_library(${PROJECT_NAME}
  src/arm_control_gui.cpp
  src/scene_3d_renderer.cpp
  include/${PROJECT_NAME}/arm_control_gui.h
  ${QT_UI_CPP}
  ${QT_RESOURCES_CPP}
)

# 添加依赖
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL
  ${OPENGL_LIBRARIES}
)

# 可执行文件
add_executable(arm_gui_node src/arm_gui_node.cpp)
target_link_libraries(arm_gui_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# 安装目标
install(TARGETS ${PROJECT_NAME} arm_gui_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 安装其他文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
) 
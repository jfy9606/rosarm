cmake_minimum_required(VERSION 3.0.2)
project(arm_gui)

# 添加Qt5依赖
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui OpenGL)

# 添加OpenCV依赖
find_package(OpenCV REQUIRED)

# 添加GL库
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  arm_trajectory
  servo_wrist
  stereo_vision
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport arm_trajectory servo_wrist stereo_vision
)

# 添加Qt5定义
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5Core_DEFINITIONS})
add_definitions(${Qt5Gui_DEFINITIONS})
add_definitions(${Qt5OpenGL_DEFINITIONS})

# 设置自动MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 设置UI文件路径
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
)

# 头文件
set(HEADER_FILES
  include/arm_gui/arm_control_gui.h
  include/arm_gui/scene_3d_renderer.h
)

# 源文件
set(SOURCE_FILES
  src/arm_control_gui.cpp
  src/scene_3d_renderer.cpp
)

# 自动处理Qt元对象
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${MOC_FILES}
)

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# 链接Qt5库和OpenCV库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
)

add_executable(${PROJECT_NAME}_node src/arm_gui.cpp)
add_executable(joint_state_publisher_node src/joint_state_publisher_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
)

target_link_libraries(joint_state_publisher_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node joint_state_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
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

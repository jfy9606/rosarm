cmake_minimum_required(VERSION 3.0.2)
project(arm_bringup)

find_package(catkin REQUIRED)

catkin_package()

# 将启动文件安装到正确的位置
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

#!/bin/bash

# 快速启动脚本 - 启动机械臂控制系统
echo "启动机械臂控制系统..."

# 确保ROS环境已设置
source /opt/ros/noetic/setup.bash
source ~/arm/rosarm/devel/setup.bash

# 启动核心功能
echo "启动核心功能节点..."
roslaunch launch/run.launch 
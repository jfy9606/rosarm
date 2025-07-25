#!/bin/bash

# ROS 2 机械臂启动脚本

# 设置颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 输出带颜色的消息
echo_color() {
  echo -e "${1}${2}${NC}"
}

# 检查conda是否安装
if ! command -v conda &> /dev/null; then
  echo_color $RED "错误: 未检测到conda，请确保已安装Anaconda或Miniconda"
  exit 1
fi

# 检查ros_env环境是否存在
if ! conda env list | grep -q "ros_env"; then
  echo_color $RED "错误: 未找到ros_env环境，请按照README.md中的说明创建环境"
  exit 1
fi

# 激活ros_env环境
echo_color $YELLOW "正在激活ros_env环境..."
eval "$(conda shell.bash hook)"
conda activate ros_env

# 检查ROS 2环境是否正确加载
if [ -z "$ROS_VERSION" ]; then
  echo_color $RED "错误: 无法加载ROS 2环境，请确保ros_env环境已正确配置"
  exit 1
fi

echo_color $GREEN "已加载ROS 2环境"

# 检查工作空间是否已构建
if [ ! -f "install/setup.bash" ]; then
  echo_color $YELLOW "工作空间未构建，正在构建..."
  colcon build --symlink-install
  
  # 检查构建是否成功
  if [ $? -ne 0 ]; then
    echo_color $RED "错误: 工作空间构建失败"
    exit 1
  fi
fi

# 加载工作空间
source install/setup.bash
echo_color $GREEN "已加载工作空间环境"

# 显示可用的启动选项
echo_color $YELLOW "可用的启动选项:"
echo "1. 启动完整系统 (包括GUI和视觉)"
echo "2. 启动基本系统 (不包括视觉)"
echo "3. 仅启动硬件控制 (电机和舵机)"
echo "4. 仅启动GUI"
echo "5. 退出"

# 读取用户选择
read -p "请选择启动选项 [1-5]: " choice

case $choice in
  1)
    echo_color $GREEN "启动完整系统..."
    ros2 launch arm_bringup arm_control.launch.py use_gui:=true use_vision:=true use_rviz:=true
    ;;
  2)
    echo_color $GREEN "启动基本系统 (不包括视觉)..."
    ros2 launch arm_bringup arm_control.launch.py use_gui:=true use_vision:=false use_rviz:=true
    ;;
  3)
    echo_color $GREEN "仅启动硬件控制..."
    ros2 launch arm_bringup arm_control.launch.py use_gui:=false use_vision:=false use_rviz:=false
    ;;
  4)
    echo_color $GREEN "仅启动GUI..."
    ros2 run gui gui_node
    ;;
  5)
    echo_color $YELLOW "退出"
    exit 0
    ;;
  *)
    echo_color $RED "无效选择，退出"
    exit 1
    ;;
esac 
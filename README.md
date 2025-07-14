# ROS 2 机械臂控制系统

这是一个基于ROS 2的机械臂控制系统，从ROS 1 Noetic迁移而来。

## 系统架构

系统由以下几个主要包组成：

- **motor**: 电机控制包，负责与电机驱动器通信
- **servo**: 舵机控制包，包括腕部控制和真空吸盘控制
- **trajectory**: 轨迹规划包，包括运动学计算和路径规划
- **vision**: 视觉处理包，用于目标检测和位置估计
- **gui**: 图形用户界面包，提供用户交互界面
- **arm_bringup**: 系统启动包，包含启动整个系统的launch文件

## 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-rolling-rclcpp \
  ros-rolling-rclpy \
  ros-rolling-std-msgs \
  ros-rolling-sensor-msgs \
  ros-rolling-geometry-msgs \
  ros-rolling-cv-bridge \
  ros-rolling-image-transport \
  ros-rolling-rviz2 \
  ros-rolling-qt-gui-cpp \
  ros-rolling-rqt-gui-cpp \
  ros-rolling-launch-ros
```

## 编译

```bash
cd ~/rosarm_ros2
colcon build --symlink-install
```

## 运行

使用提供的启动脚本：

```bash
cd ~/rosarm_ros2
./start.sh
```

或者手动启动：

```bash
# 启动完整系统
ros2 launch arm_bringup arm_control.launch.py

# 仅启动GUI
ros2 launch gui gui.launch.py

# 仅启动硬件控制
ros2 launch arm_bringup arm_control.launch.py use_gui:=false use_vision:=false use_rviz:=false
```

## 功能

- 电机控制：通过串口通信控制电机
- 舵机控制：控制腕部舵机和真空吸盘
- 轨迹规划：提供正逆运动学计算和路径规划
- 视觉处理：提供目标检测和位置估计
- 图形界面：提供用户友好的控制界面

## 系统要求

- ROS 2 Rolling
- Ubuntu 20.04或更高版本
- Python 3.8或更高版本
- C++14或更高版本

## 许可证

待定 
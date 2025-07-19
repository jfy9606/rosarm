# ROS 2 机械臂控制系统

这是一个基于ROS 2的机械臂控制系统，从ROS 1 Noetic迁移而来。

## 系统架构

系统由以下几个主要包组成：

- **motor**: 电机控制包，负责与电机驱动器通信
- **servo**: 舵机控制包，包括腕部控制和真空吸盘控制，以及独立的Feetech SDK实现
- **servo_interfaces**: 舵机控制接口定义包，包含消息和服务定义
- **serial**: 串口通信包，提供串口通信功能
- **trajectory**: 轨迹规划包，包括运动学计算和路径规划
- **vision**: 视觉处理包，用于目标检测和位置估计
- **gui**: 图形用户界面包，提供用户交互界面
- **arm_bringup**: 系统启动包，包含启动整个系统的launch文件

## 环境部署

### 安装ROS 2 Humble（使用mamba/conda）

```bash
# 安装Anaconda或Miniconda
# 下载地址: https://docs.conda.io/en/latest/miniconda.html

# 安装mamba（可选，但推荐使用）
conda install mamba -c conda-forge

# 创建ROS环境
conda create -n ros_env
conda activate ros_env
# 或者使用mamba
# mamba create -n ros_env
# mamba activate ros_env

# 添加conda-forge频道
conda config --env --add channels conda-forge
# 移除defaults频道（如果存在）
conda config --env --remove channels defaults

# 添加ROS 2 Humble频道
conda config --env --add channels robostack-humble

# 安装ROS 2 Humble（使用conda）
conda install ros-humble-desktop
# 或者使用mamba（更快）
conda install mamba -c conda-forge
mamba install ros-humble-desktop

# 安装构建工具（使用conda）
conda install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
conda install ros-humble-sensor-msgs ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime -c robostack-humble
# 或者使用mamba（更快）
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
mamba install ros-humble-sensor-msgs ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime -c robostack-humble

# 安装GUI和 nlohmann_json（使用conda）
conda install ros-humble-rqt ros-humble-rqt-common-plugins nlohmann_json
# 或者使用mamba（更快）
mamba install ros-humble-rqt ros-humble-rqt-common-plugins nlohmann_json

# Windows需要安装vs2019/vs2022，安装基于C++的图形开发环境
```

### 初始化环境

```bash
# 为ROS环境设置
conda deactivate
conda activate ros_env

#安装OpenGL开发库
sudo apt-get update && sudo apt-get install -y libgl1-mesa-dev
```

## 使用串口

```bash
# 添加当前用户到dialout组以访问串口
sudo usermod -a -G dialout $USER
# 需要注销并重新登录才能生效
```

## 编译

```bash
# 在工作空间根目录下执行
conda activate ros_env
colcon build --symlink-install
```

## 运行

使用提供的启动脚本：

```bash
# 在工作空间根目录下执行
./start.sh
# 脚本会自动激活ros_env环境
```

或者手动启动：

```bash
# 激活环境
conda activate ros_env
# 加载工作空间
source install/setup.bash

# 启动完整系统
ros2 launch arm_bringup arm_control.launch.py

# 仅启动GUI
ros2 run rqt_gui rqt_gui

# 仅启动硬件控制
ros2 launch arm_bringup arm_control.launch.py use_gui:=false use_vision:=false use_rviz:=false
```

## 功能

### 电机控制
- 通过串口通信控制电机
- 提供位置、速度和加速度控制

### 舵机控制
- 控制腕部舵机和真空吸盘
- 提供独立的Feetech SDK实现，可脱离ROS使用
- 支持单个舵机控制和同步控制多个舵机
- 提供位置、速度和时间参数控制

### 轨迹规划
- 提供正逆运动学计算和路径规划
- 支持直线、圆弧和样条曲线路径

### 视觉处理
- 提供目标检测和位置估计
- 支持颜色识别和形状识别

### 图形界面
- 使用ROS 2内置的rqt工具提供用户友好的控制界面
- 支持手动控制和自动控制模式

## 系统要求

- Anaconda或Miniconda
- ROS 2 Humble (通过conda安装)
- Python 3.11
- C++14或更高版本
- nlohmann_json库 (用于配置文件解析)
- ROS 2 rqt组件 (用于GUI)

## 常见问题

### 串口权限问题

如果遇到串口权限问题，请确保当前用户已添加到dialout组：

```bash
sudo usermod -a -G dialout $USER
# 需要注销并重新登录才能生效
```

### 找不到包或节点

如果遇到找不到包或节点的问题，请确保已正确加载工作空间：

```bash
source install/setup.bash
```

### 编译错误

如果遇到编译错误，请检查是否已安装所有依赖：

```bash
conda activate ros_env
conda install nlohmann_json
conda install ros-humble-sensor-msgs ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime -c robostack-humble
```

### GUI问题

如果遇到GUI相关问题，请确保已安装ROS 2的rqt组件：

```bash
conda install ros-humble-rqt ros-humble-rqt-common-plugins
```

### 环境问题

如果遇到环境相关的问题，可以尝试重新创建环境：

```bash
conda deactivate
conda env remove -n ros_env
conda create -n ros_env 
```

## 许可证

[待添加]


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

## 环境部署

### 安装ROS 2 Humble（使用mamba）

```bash
# 安装Mambaforge（如果尚未安装）
wget -O Mambaforge.sh https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh
bash Mambaforge.sh -b -p $HOME/mambaforge
source $HOME/mambaforge/etc/profile.d/conda.sh
source $HOME/mambaforge/etc/profile.d/mamba.sh

# 创建ROS环境
mamba create -n ros_env
mamba activate ros_env

# 添加conda-forge频道
conda config --env --add channels conda-forge
# 移除defaults频道（如果存在）
conda config --env --remove channels defaults

# 添加ROS 2 Humble频道
conda config --env --add channels robostack
conda config --env --add channels robostack-humble

# 安装ROS 2 Humble
mamba install ros-humble-desktop

# 安装构建工具
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep ros2-for-serial-driver
```

### 初始化环境

```bash
# 添加环境激活到bashrc（可选）
echo "source $HOME/mambaforge/etc/profile.d/conda.sh" >> ~/.bashrc
echo "source $HOME/mambaforge/etc/profile.d/mamba.sh" >> ~/.bashrc
echo "mamba activate ros_env" >> ~/.bashrc

# 为ROS环境设置
mamba activate ros_env
echo "source $CONDA_PREFIX/setup.bash" >> ~/.bashrc

# 初始化rosdep
rosdep init
rosdep update

# 使配置在当前终端生效
source ~/.bashrc
```

### 初始化Python环境

```bash
# 激活ROS环境
mamba activate ros_env

# 安装必要的Python依赖
mamba install -y pip lark-parser numpy opencv pyyaml yaml-cpp
```

## 安装依赖

```bash
# 激活ROS环境
mamba activate ros_env

# 安装系统依赖
mamba install -y \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rviz2 \
  ros-humble-qt-gui-cpp \
  ros-humble-rqt-gui-cpp \
  ros-humble-launch-ros
```

### 串口

```bash
# 添加当前用户到dialout组以访问串口
sudo usermod -a -G dialout $USER
```

## 编译

```bash
# 在工作空间根目录下执行
mamba activate ros_env
source $CONDA_PREFIX/setup.bash
colcon build --symlink-install
```

## 运行

使用提供的启动脚本：

```bash
# 在工作空间根目录下执行
mamba activate ros_env
source $CONDA_PREFIX/setup.bash
source install/setup.bash  # 加载编译后的环境
./start.sh
```

或者手动启动：

```bash
# 激活环境
mamba activate ros_env
source $CONDA_PREFIX/setup.bash
source install/setup.bash

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

- ROS 2 Humble
- 支持Conda/Mamba的Linux/macOS/Windows系统
- Python 3.8或更高版本
- C++14或更高版本

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

## 许可证

待定

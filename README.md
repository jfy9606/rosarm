# ROS机械臂控制系统

这是一个基于ROS的机械臂控制系统，用于控制带有多个关节的机械臂，支持电机控制、舵机控制、真空吸盘控制以及轨迹规划功能。

## 环境初始化配置

### 系统要求
- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- OpenCV 4.2+

### 环境配置步骤

1. 安装ROS Noetic
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```

2. 初始化ROS环境
   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   sudo rosdep init
   rosdep update
   ```

3. 创建工作空间并克隆代码
   ```bash
   mkdir -p ~/rosarm/src
   cd ~/rosarm
   catkin_make
   source devel/setup.bash
   cd src
   # 克隆代码仓库
   git clone <repository_url> .
   cd ..
   ```

4. 安装依赖
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. 编译工作空间
   ```bash
   catkin_make
   ```

6. 设置串口权限
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB1
   ```
   (需要重新登录后生效)

7. 启动系统
   ```bash
   cd ~/rosarm
   ./start.sh
   ```

## 系统架构

系统由以下几个主要包组成：

### motor
电机控制基础包，负责大臂电机的底层控制，通过串口与电机驱动器通信。

- 支持位置控制和速度控制
- 提供电机状态监控
- 通过串口（默认/dev/ttyUSB0，115200波特率）与电机驱动器通信

### servo
舵机控制包，负责小臂舵机和真空吸盘的控制。

- 舵机控制：控制腕部和夹爪舵机（通过/dev/ttyUSB1，1000000波特率）
- 真空吸盘控制：控制真空吸盘的开关和吸力（通过/dev/ttyUSB1，115200波特率）
- 提供高级机械臂控制接口，整合电机和舵机控制

### trajectory
轨迹规划包，提供机械臂的运动学计算和轨迹规划功能。

- 正向运动学：根据关节角度计算末端执行器位姿
- 逆向运动学：根据末端执行器位姿计算关节角度
- 轨迹规划：生成平滑的关节轨迹

### gui
图形用户界面包，提供机械臂控制的可视化界面。

- 显示机械臂状态
- 提供手动控制界面
- 支持轨迹规划和执行

## 安装与配置

### 系统要求

- Ubuntu 20.04
- ROS Noetic
- 串口设备访问权限

### 安装步骤

1. 克隆仓库到ROS工作空间的src目录：
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/yourusername/rosarm.git
   ```

2. 安装依赖：
   ```
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. 编译：
   ```
   catkin_make
   ```

4. 设置串口权限：
   ```
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB1
   ```
   (重新登录后生效)

## 使用方法

### 启动完整系统

使用launch文件启动完整系统：

```
roslaunch arm_bringup arm_control.launch
```

### 单独启动组件

1. 启动电机控制：
   ```
   roslaunch motor motor_control.launch
   ```

2. 启动舵机控制：
   ```
   roslaunch servo servo_control.launch
   ```

3. 启动真空吸盘控制：
   ```
   roslaunch servo vacuum_control.launch
   ```

4. 启动GUI：
   ```
   roslaunch gui gui.launch
   ```

### 控制接口

系统提供以下ROS话题和服务：

#### 话题

- `/joint_command`：发送关节角度命令
- `/pose_command`：发送末端执行器位姿命令
- `/vacuum/command`：控制真空吸盘开关
- `/vacuum/power`：设置真空吸盘吸力
- `/gripper/command`：控制夹爪开关
- `/joint_states`：获取当前关节状态
- `/end_effector_pose`：获取当前末端执行器位姿

#### 服务

- `/joint_control`：控制关节位置
- `/home_position_service`：将机械臂移动到初始位置
- `/forward_kinematics`：计算正向运动学
- `/inverse_kinematics`：计算逆向运动学

## 开发者文档

### 包结构

- `motor`：电机控制基础包
- `servo`：舵机控制和机械臂控制包
- `trajectory`：运动学和轨迹规划包
- `gui`：图形用户界面包
- `arm_bringup`：启动文件和系统配置

### 添加新功能

要添加新功能，请遵循以下步骤：

1. 在适当的包中创建新的类或节点
2. 更新CMakeLists.txt和package.xml
3. 添加新的launch文件或更新现有launch文件
4. 编写测试代码
5. 更新文档

## 许可证

本项目采用MIT许可证。详情请参阅LICENSE文件。

## 贡献者

- [您的姓名]
- [其他贡献者]

## 联系方式

如有问题，请联系[您的邮箱]。 
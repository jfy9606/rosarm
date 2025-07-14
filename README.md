# ROS 2机械臂控制系统

## 系统概述
本系统是基于ROS 2的机械臂控制系统，从ROS 1 Noetic版本升级而来。系统集成了舵机控制、电机控制、路径规划和图形用户界面。

## 硬件配置
- FT SCServo舵机(STS3020和STS3032)：波特率1000000
- 电机控制器：波特率115200

## 串口配置
系统使用两个串口设备进行通信：
1. 舵机控制串口：用于控制FT SCServo舵机，波特率1000000
2. 电机控制串口：用于控制电机，波特率115200

## 快速启动
1. 确保硬件连接正确
2. 运行启动脚本：
```bash
./start.sh
```
3. 从菜单中选择相应选项：
   - 1：启动完整系统
   - 2：检测串口设备
   - 3：修复设备权限
   - 4：构建系统
   - 0：退出

## 系统要求
- ROS 2 Rolling
- Ubuntu 22.04或兼容系统
- Qt5图形库
- Python 3.10+

## 许可证
MIT License

## 环境初始化配置

### 系统要求
- Ubuntu 22.04
- ROS 2 Rolling
- Python 3.10+
- OpenCV 4.5+

### 环境配置步骤

1. 安装ROS 2 Rolling
   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-rolling-desktop
   ```

2. 初始化ROS 2环境
   ```bash
   echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   sudo rosdep init
   rosdep update
   ```

3. 克隆仓库并构建
   ```bash
   mkdir -p ~/rosarm_ros2/src
   cd ~/rosarm_ros2
   # 克隆代码仓库
   git clone <repository_url> .
   # 安装依赖
   rosdep install --from-paths src --ignore-src -r -y
   # 构建
   colcon build --symlink-install
   source install/setup.bash
   ```

4. 设置串口权限
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB1
   ```
   (需要重新登录后生效)

5. 启动系统
   ```bash
   cd ~/rosarm_ros2
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

### vision
视觉处理包，提供摄像头图像处理和目标检测功能。

- 图像采集：从摄像头获取图像
- 目标检测：使用计算机视觉算法检测场景中的物体
- 三维位置估计：根据立体视觉计算物体的3D位置

### gui
图形用户界面包，提供机械臂控制的可视化界面。

- 显示机械臂状态
- 提供手动控制界面
- 支持轨迹规划和执行
- 显示摄像头图像和检测结果

### arm_bringup
启动文件和配置包，集成其他所有功能包并提供完整系统启动。

- 包含系统启动文件
- 集成所有节点配置

## 升级变更说明
从ROS 1 Noetic到ROS 2 Rolling的主要升级内容：

1. 所有包结构升级至ROS 2格式
   - package.xml升级至format="3"
   - CMakeLists.txt适应ament_cmake构建系统
   - 消息和服务定义更新

2. 节点实现升级
   - 使用ROS 2 C++ API (rclcpp)
   - 使用ROS 2 Python API (rclpy)
   - 使用参数而非ROS参数服务器

3. 启动系统升级
   - XML启动文件转换为Python启动脚本
   - 参数传递方式更新

4. 通信模式升级
   - 发布/订阅模式升级至ROS 2 DDS
   - 服务和动作升级

5. GUI系统升级
   - 使用ROS 2兼容Qt插件
   - 更新视图组件以使用ROS 2消息

## 控制接口

系统提供以下ROS 2话题和服务：

### 话题

- `/joint_command`：发送关节角度命令
- `/pose_command`：发送末端执行器位姿命令
- `/vacuum/command`：控制真空吸盘开关
- `/vacuum/power`：设置真空吸盘吸力
- `/gripper/command`：控制夹爪开关
- `/joint_states`：获取当前关节状态
- `/end_effector_pose`：获取当前末端执行器位姿

### 服务

- `/joint_control`：控制关节位置
- `/home_position_service`：将机械臂移动到初始位置
- `/forward_kinematics`：计算正向运动学
- `/inverse_kinematics`：计算逆向运动学
# ROSARM 机械臂控制系统

这是一个基于ROS的机械臂控制系统，可以控制舵机驱动的机械臂执行各种任务。

## 项目结构

本项目包含以下主要组件：

- **servo_wrist**: 负责控制机械臂的舵机
- **arm_3d_control**: 提供通过三维坐标控制机械臂末端位置的功能
- **arm_trajectory**: 实现基于鲸鱼优化算法的轨迹规划功能
- **liancheng_socket**: 提供通信功能和示例动作

## 环境要求

- ROS Noetic (Ubuntu 20.04)或兼容版本
- C++11或更高版本
- Python 3.6+
- 舵机硬件和驱动程序
- USB串口设备

## 安装方法

1. 克隆仓库到您的ROS工作空间的src目录中：

```bash
git clone <repository-url> rosarm
```

2. 安装Python依赖 (用于轨迹规划功能):
```bash
pip install numpy
```

3. 编译工程：

```bash
source /opt/ros/noetic/setup.bash
catkin build
```

4. 设置环境变量：

```bash
source ~/rosarm_old/devel/setup.bash
```

## 基本使用方法

### 启动系统

使用以下命令启动系统的所有功能（舵机控制和轨迹规划）：

```bash
cd ~/rosarm_old && roslaunch launch/run.launch
```

这将启动以下节点：
- `talker`: 负责通信功能
- `serial`: 处理串口通信
- `servo`: 舵机控制节点
- `trajectory_bridge`: 轨迹规划功能

### 直接控制舵机

通过发布消息到`servo_control_topic`来直接控制舵机：

```bash
# 控制ID为1的舵机移动到位置2000，速度500，加速度30
rostopic pub /servo_control_topic servo_wrist/SerControl "servo_id: 1
target_position: 2000
velocity: 500
acceleration: 30"
```

## 3D位置控制功能

这个功能允许您通过指定三维坐标(x,y,z)来控制机械臂末端移动到指定位置。系统会自动计算逆运动学，并控制各个舵机移动到适当的位置。

### 启动3D位置控制

```bash
cd ~/rosarm_old && roslaunch src/arm_3d_control/launch/arm_3d_control.launch
```

### 使用方法

#### 方法1: 使用提供的客户端工具

运行提供的客户端工具，可以通过命令行交互方式输入目标坐标：

```bash
cd ~/rosarm_old && rosrun arm_3d_control position_sender
```

然后按照提示输入目标位置的x、y、z坐标（单位：mm），例如：
```
请输入机械臂末端目标位置 (x y z)，单位mm，输入'q'退出: 100 100 150
```

#### 方法2: 直接发布ROS消息

也可以通过直接发布ROS消息来控制机械臂位置：

```bash
rostopic pub /arm_target_position geometry_msgs/Point "x: 100.0
y: 100.0
z: 150.0"
```

## 轨迹规划功能

该功能使用鲸鱼优化算法(WOA)进行机械臂轨迹规划，实现平滑高效的动作路径。

### 轨迹规划服务

可以通过以下服务请求轨迹规划：
```bash
rosservice call /plan_arm_trajectory "arm_id: 'arm1'
start_pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
end_pose:
  position:
    x: 0.2
    y: 0.1
    z: 0.15
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
execution_time: 5.0"
```

### 命令控制

轨迹规划节点接受以下控制台命令：
```bash
# 设置目标位置（必须先设置，然后才能抓取）
rostopic pub /arm_command std_msgs/String "data: 'set_target object1 0.2 0.1 0.15'"

# 抓取已设置的目标
rostopic pub /arm_command std_msgs/String "data: 'pick arm1 object1'"

# 放置到指定位置
rostopic pub /arm_command std_msgs/String "data: 'place arm1 0.2 0.1 0.15'"

# 回到初始位置
rostopic pub /arm_command std_msgs/String "data: 'home arm1'"
```

## 示例动作

使用 `talk_test` 节点可以执行预设的机械臂动作：

```bash
# 启动通信节点
rosrun liancheng_socket talk_test
```

然后根据提示输入1-3选择不同的预设动作：
- 1: 演示机械臂自由度
- 2: 演示正常姿势
- 3: 演示大角度姿势

## 参数配置

### 机械臂尺寸参数

如需调整机械臂尺寸参数，可以修改以下文件：

```
~/rosarm_old/src/arm_3d_control/include/arm_3d_control/arm_parameters.h
```

主要参数：
- LINK1_LENGTH: 底座到第一个关节的距离
- LINK2_LENGTH: 第一个关节到第二个关节的距离
- LINK3_LENGTH: 第二个关节到末端执行器的距离

### 舵机参数

舵机ID和默认控制参数也在arm_parameters.h中定义：

- BASE_SERVO_ID: 底座旋转舵机的ID
- SHOULDER_SERVO_ID: 肩关节舵机的ID
- ELBOW_SERVO_ID: 肘关节舵机的ID
- DEFAULT_VELOCITY: 默认速度
- DEFAULT_ACCELERATION: 默认加速度

## 编译问题解决

如果在编译时遇到问题：

1. 确保已经源引入ROS环境:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```

2. SCServo库问题:
   如果遇到SCServo库链接问题，请重新编译库:
   ```bash
   cd ~/rosarm_old/src/servo_wrist/src/SCServo_Linux
   rm -f CMakeCache.txt
   mkdir -p build
   cd build
   cmake ..
   make
   cp libSCServo.a ..
   ```

3. Python依赖问题:
   如果遇到与Python库相关的错误，确保安装了所有依赖:
   ```bash
   pip install numpy
   ```

## 故障排除

### 串口连接问题

如果遇到串口连接错误，请检查：
1. 检查USB设备是否正确连接
2. 验证设备权限：`ls -l /dev/ttyUSB*`
3. 如需要，添加权限：`sudo chmod a+rw /dev/ttyUSB1`

### 舵机控制问题

如果舵机无法正确移动：
1. 检查舵机ID是否正确设置
2. 验证舵机供电是否充足
3. 使用较低的速度和加速度值进行测试

## 贡献指南

如果您希望为本项目做出贡献，请遵循以下步骤：

1. Fork本项目
2. 创建您的特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交您的更改 (`git commit -m 'Add some amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 开启一个Pull Request

## 许可证

本项目遵循 BSD 许可证 - 详情请查看仓库中的LICENSE文件。


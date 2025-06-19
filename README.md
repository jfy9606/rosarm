# 机械臂控制系统

这是一个用于控制机械臂的ROS系统，集成了双目视觉感知与轨迹规划功能。

## 系统结构

系统包含以下主要部分：

1. **核心功能**：基本的机械臂控制功能，包括串口通信和舵机控制
2. **GUI界面**：用于人机交互的图形界面
3. **示例动作**：预设的示例动作，用于展示机械臂功能
4. **双目摄像头视觉模块** (`stereo_vision`): 
   - 利用YOLOv8进行物体检测
   - 使用立体匹配算法计算深度信息
   - 将2D检测结果转换为3D位置坐标
5. **轨迹规划模块** (`arm_trajectory`): 
   - 使用鲸鱼优化算法(WOA)进行逆运动学求解
   - 基于立体视觉提供的目标点生成平滑轨迹
   - 处理机械臂的协同规划

## 运行方式

### 1. 运行完整系统（推荐）

```bash
# 加载ROS环境
cd ~/arm/catkin && source devel/setup.bash

# 启动核心功能
roslaunch ~/arm/catkin/launch/run.launch

# 在新终端中启动GUI
cd ~/arm/catkin && source devel/setup.bash
roslaunch arm_gui arm_gui.launch
```

### 2. 仅运行示例动作

```bash
# 加载ROS环境
cd ~/arm/catkin && source devel/setup.bash

# 启动示例动作节点
roslaunch ~/arm/catkin/launch/demo.launch
```

### 3. 仅使用图形界面控制

```bash
# 加载ROS环境
cd ~/arm/catkin && source devel/setup.bash

# 启动核心功能
roslaunch ~/arm/catkin/launch/run.launch

# 在新终端中启动GUI
cd ~/arm/catkin && source devel/setup.bash
roslaunch arm_gui arm_gui.launch
```

## 硬件要求

- 舵机控制器应连接到 `/dev/ttyUSB1`（可在launch文件中修改）
- 如果使用立体摄像头，默认使用 `/dev/video0`

## 安装依赖

```bash
# 安装必要的ROS包
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-tf2-ros ros-noetic-moveit-msgs ros-noetic-moveit-core ros-noetic-moveit-ros-planning-interface ros-noetic-serial

# 安装Python依赖
pip install ultralytics numpy opencv-python
```

## 系统功能

### 机械臂控制命令

系统支持通过`/arm_command`话题发送文本命令控制机械臂：

1. 抓取命令: `pick [机械臂ID] [物体ID]`
   - 例如: `pick arm1 object_0`

2. 放置命令: `place [机械臂ID] [x坐标] [y坐标] [z坐标]`
   - 例如: `place arm1 0.2 0.3 0.1`

3. 回到初始位置: `home [机械臂ID]`
   - 例如: `home arm1`

### 发送命令示例

```bash
# 发送抓取命令
rostopic pub /arm_command std_msgs/String "data: 'pick arm1 object_0'"

# 发送放置命令
rostopic pub /arm_command std_msgs/String "data: 'place arm1 0.2 0.3 0.1'"

# 发送回到初始位置命令
rostopic pub /arm_command std_msgs/String "data: 'home arm1'"
```

### 系统参数配置

可通过launch文件参数调整系统配置：

- `use_stereo_cam`: 是否使用实际摄像头(默认: true)
- `stereo_cam_device`: 摄像头设备路径(默认: /dev/video0)
- `yolo_model_path`: YOLOv8模型路径
- `enable_visualization`: 是否启用可视化(默认: true)

### 自定义配置示例

```bash
roslaunch stereo_vision stereo_arm_control.launch use_stereo_cam:=false yolo_model_path:="/path/to/your/model.pt"
```

## 主要ROS话题

- `/stereo_camera/image_raw`: 双目相机原始图像
- `/stereo_vision/detection_image`: 检测结果可视化图像
- `/stereo_vision/depth_image`: 深度图像
- `/stereo_vision/detected_poses`: 检测到物体的三维位置
- `/arm_command`: 机械臂控制命令
- `/arm1/joint_command`: 机械臂1的关节控制命令
- `/arm1/vacuum_command`: 机械臂1的吸附控制命令
- `/Controller_motor_order`: 电机控制命令
- `/RelayOrder`: 继电器控制命令

### 服务

- `/plan_trajectory`: 轨迹规划服务

## 注意事项

- 请确保硬件连接正确
- 如需调整参数，可修改相应launch文件中的参数值
- 使用GUI界面时，可以通过"示例动作"区域中的按钮执行预设动作
- 双目摄像头视觉功能需要正确连接和配置摄像头

# 机械臂视觉感知与轨迹规划系统

本系统将双目摄像头视觉感知与机械臂轨迹规划集成到ROS框架中，实现了基于视觉的机械臂自动控制。

## 系统组件

系统由三个主要部分组成：

1. **双目摄像头视觉模块** (`stereo_vision`): 
   - 利用YOLOv8进行物体检测
   - 使用立体匹配算法计算深度信息
   - 将2D检测结果转换为3D位置坐标

2. **轨迹规划模块** (`arm_trajectory`): 
   - 使用鲸鱼优化算法(WOA)进行逆运动学求解
   - 基于立体视觉提供的目标点生成平滑轨迹
   - 处理多个机械臂的协同规划

3. **ROS通信与控制框架**:
   - 实现各模块间的消息通信
   - 提供可视化界面(RViz)
   - 支持命令行控制接口

## 安装依赖

```bash
# 安装必要的ROS包
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-tf2-ros

# 安装Python依赖
pip install ultralytics numpy opencv-python
```

## 编译与安装

```bash
# 进入工作空间
cd ~/arm/catkin/catkin_ws

# 编译
catkin_make

# 加载环境
source devel/setup.bash
```

## 使用方法

### 启动系统

```bash
# 启动集成系统
roslaunch stereo_vision stereo_arm_control.launch
```

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

## 系统参数配置

可通过launch文件参数调整系统配置：

- `use_stereo_cam`: 是否使用实际摄像头(默认: true)
- `stereo_cam_device`: 摄像头设备路径(默认: /dev/video0)
- `yolo_model_path`: YOLOv8模型路径
- `enable_visualization`: 是否启用可视化(默认: true)

### 自定义配置示例

```bash
roslaunch stereo_vision stereo_arm_control.launch use_stereo_cam:=false yolo_model_path:="/path/to/your/model.pt"
```

## 系统架构

![系统架构](./system_architecture.png)

### 主要ROS话题

- `/stereo_camera/image_raw`: 双目相机原始图像
- `/stereo_vision/detection_image`: 检测结果可视化图像
- `/stereo_vision/depth_image`: 深度图像
- `/stereo_vision/detected_poses`: 检测到物体的三维位置
- `/arm_command`: 机械臂控制命令
- `/arm1/trajectory_command`: 机械臂1的轨迹命令
- `/arm2/trajectory_command`: 机械臂2的轨迹命令

### 服务

- `/plan_arm_trajectory`: 规划机械臂轨迹服务 
# 机械臂立体视觉控制系统

一个集成了立体视觉和机械臂控制的ROS包，提供以下功能：
- 左右相机视图和深度图显示
- YOLO物体检测
- 物体距离测量
- 机械臂关节控制
- 机械臂末端坐标控制
- 集成的图形用户界面(GUI)

## 功能特点

- **相机视图**：在左相机、右相机和深度图之间切换
- **物体检测**：基于YOLO的物体检测，显示边界框
- **深度计算**：从立体相机图像计算深度图
- **3D位置**：估计检测到的物体的3D位置和距离
- **机械臂控制**：支持关节空间和笛卡尔空间的机械臂控制
- **集成GUI**：提供直观的用户界面进行操作

## 使用方法

### 启动系统

```bash
roslaunch stereo_vision stereo_vision.launch
```

### 启动参数

- `stereo_cam_device`: 相机设备 (默认: `/dev/video0`)
- `yolo_model_path`: YOLO模型路径 (默认: `$(env HOME)/arm/best.pt`)
- `image_width`: 相机图像宽度 (默认: `1280`)
- `image_height`: 相机图像高度 (默认: `480`)
- `frame_rate`: 相机帧率 (默认: `30`)
- `enable_detection`: 启用YOLO检测 (默认: `true`)
- `initial_view_mode`: 初始视图模式 (默认: `0`, 0=左相机, 1=右相机, 2=深度图)
- `with_trajectory_planner`: 启用轨迹规划 (默认: `true`)
- `with_arm_controller`: 启用机械臂控制器 (默认: `true`)
- `start_rviz`: 启动RViz可视化 (默认: `false`)

### 切换视图模式

使用服务切换不同的视图模式(左相机、右相机、深度图):

```bash
# 切换到左相机视图 (模式 0)
rosservice call /stereo_vision/switch_view "view_mode: 0"

# 切换到右相机视图 (模式 1)
rosservice call /stereo_vision/switch_view "view_mode: 1"

# 切换到深度图视图 (模式 2)
rosservice call /stereo_vision/switch_view "view_mode: 2"
```

## 主要话题

### 订阅的话题

- `/stereo_camera/image_raw`: 原始立体相机图像(并排格式)
- `/left_camera/image_raw`: 左相机图像(如果使用分离输入)
- `/right_camera/image_raw`: 右相机图像(如果使用分离输入)

### 发布的话题

- `/left_image`: 左相机图像
- `/right_image`: 右相机图像
- `/depth_image`: 深度图可视化
- `/detection_image`: 带检测结果的图像
- `/detected_poses`: 检测到的物体的3D位姿
- `/detections/image`: 带检测结果的图像(GUI兼容)
- `/detections/poses`: 检测到的物体的3D位姿(GUI兼容)

## 服务

- `/stereo_vision/switch_view`: 切换视图模式(0=左相机, 1=右相机, 2=深度图)
- `/arm_trajectory/plan_path`: 规划机械臂路径
- `/arm_trajectory/execute`: 执行规划的轨迹

## 依赖

- ROS
- OpenCV
- NumPy
- Ultralytics YOLO
- TF2
- Qt5 
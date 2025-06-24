# 机械臂视觉控制系统

这是一个基于ROS的机械臂控制系统，使用摄像头进行视觉感知、目标检测和路径规划。

## 系统组件

系统由以下主要组件组成：

1. **摄像头** - 支持高分辨率USB摄像头，包括宽幅摄像头
2. **图像处理** - 使用OpenCV和YOLO进行图像处理和目标检测
3. **机械臂控制** - 基于ROS的机械臂控制界面和轨迹规划

## 安装指南

### 前提条件

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- OpenCV 4.2+

### 安装依赖

```bash
# 安装基本依赖
sudo apt-get update
sudo apt-get install ros-noetic-image-view ros-noetic-tf2-ros

# 安装Python依赖
pip3 install numpy opencv-python torch ultralytics
```

### 编译工作空间

```bash
# 克隆仓库
git clone https://github.com/your-username/arm-vision-system.git
cd arm-vision-system

# 编译ROS工作空间
catkin_make
# 或使用catkin工具
catkin build

# 设置环境
source devel/setup.bash
```

## 使用指南

### 启动方式

系统提供了几种不同的启动方式：

```bash
# 启动完整系统
./start.sh

# 启动系统并启用YOLO目标检测
./start.sh --yolo

# 仅测试摄像头
./start.sh --test

# 直接模式（不使用ROS）
./start.sh --direct

# 指定摄像头和分辨率
./start.sh --camera /dev/video0 --resolution 1280x480
```

### 摄像头支持

系统自动支持宽幅摄像头，会自动检测图像宽高比，并处理左半部分图像。这对于宽幅图像尤其有用。

### 运行完整系统

使用以下命令启动完整系统，包括GUI界面和目标检测：

```bash
roslaunch run.launch enable_yolo:=true
```

可选参数：
- `enable_yolo` - 启用/禁用YOLO目标检测 (默认: false)
- `camera_device` - 摄像头设备路径 (默认: /dev/video0)
- `resolution_width` - 摄像头宽度 (默认: 1280)
- `resolution_height` - 摄像头高度 (默认: 480)
- `fps` - 帧率 (默认: 30)

### 系统配置

如需修改系统配置，可以编辑以下文件：

- `launch/run.launch` - 主启动文件
- `launch/camera_test.launch` - 摄像头测试启动文件
- `launch/direct_camera.launch` - 直接访问摄像头启动文件

## 故障排除

### 常见问题

1. **摄像头不工作**
   - 检查摄像头设备是否存在: `ls -la /dev/video*`
   - 确保摄像头权限正确: `sudo chmod a+rw /dev/video*`
   - 尝试直接测试模式: `./start.sh --test`

2. **图像显示问题**
   - 检查分辨率设置是否与摄像头匹配
   - 如果图像为黑屏，可能是摄像头曝光问题
   - 尝试不同的像素格式 (YUYV, MJPEG)

3. **目标检测不工作**
   - 确保已安装所需的Python库
   - 检查YOLO模型路径是否正确

4. **轨迹规划错误**
   - 检查机械臂配置参数
   - 确保已安装所有ROS依赖包

## 系统架构

系统采用模块化设计，主要包括以下ROS节点：

1. `camera_node` - 读取摄像头图像并发布到ROS话题
2. `yolo_detector` - 基于YOLO的目标检测
3. `trajectory_bridge` - 轨迹规划和执行
4. `arm_gui` - 用户界面

各节点之间通过ROS话题进行通信，主要话题包括：

- `/camera/image_raw` - 摄像头图像
- `/camera/detections` - 目标检测结果
- `/detection_image` - 包含检测结果的可视化图像

## 工具说明

系统提供了几个有用的工具脚本：

1. `camera_test.py` - 测试摄像头并显示参数信息
2. `camera_direct.py` - 不使用ROS直接访问摄像头
3. `camera_node.py` - ROS摄像头节点

## 许可证

本项目基于MIT许可证发布 - 详见 [LICENSE](LICENSE) 文件。

## 致谢

- 感谢所有贡献者和使用者
- 特别感谢ROS和OpenCV社区

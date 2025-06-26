# 机械臂视觉控制系统

这是一个基于ROS的机械臂控制系统，使用摄像头进行视觉感知、目标检测和路径规划。

## 系统组件

系统由以下主要组件组成：

1. **摄像头** - 支持高分辨率USB摄像头，包括宽幅摄像头
2. **图像处理** - 使用OpenCV和YOLO进行图像处理和目标检测
3. **机械臂控制** - 基于ROS的机械臂控制界面和轨迹规划

## 系统特性

### 末端执行器摄像头

系统支持在机械臂末端执行器（吸盘）上安装摄像头，提供移动视角。系统特性包括：

1. **移动视角** - 摄像头跟随机械臂移动，提供末端执行器视角的物体检测
2. **坐标系转换** - 自动在摄像头坐标系和世界坐标系之间进行转换
3. **动态显示** - 3D场景中物体位置会根据摄像头位置自动更新

这一特性使系统能够从吸取物体的视角检测和处理物体，提高了抓取精度。

## 安装指南

### 前提条件

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- OpenCV 4.2+

### YOLOv8安装说明

系统使用YOLOv8进行目标检测，按照以下步骤进行安装：

```bash
# 安装PyTorch和相关依赖
pip3 install torch torchvision

# 安装Ultralytics包（YOLOv8）
pip3 install ultralytics

# 验证安装
python3 -c "from ultralytics import YOLO; print('YOLOv8安装成功')"
```

如果需要特定版本的模型权重，可以：

```bash
# 下载模型权重到项目目录
mkdir -p models
wget -P models/ https://github.com/ultralytics/assets/releases/download/latest/yolov8n.pt  # nano版本
wget -P models/ https://github.com/ultralytics/assets/releases/download/latest/yolov8s.pt  # small版本
```

运行时需要在启动脚本中指定模型路径：

```bash
./start.sh --yolo --model models/yolov8n.pt
```

### 安装依赖

```bash
# 安装ROS基本依赖
sudo apt-get update
sudo apt-get install ros-noetic-image-view ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-serial

# 安装Qt5依赖
sudo apt-get install qtbase5-dev qt5-default qtchooser qttools5-dev-tools qttools5-dev libqt5core5a libqt5gui5 libqt5widgets5 libqt5opengl5 ros-noetic-rqt* ros-noetic-qt-gui* -y

# 安装Qt5 OpenGL开发库（用于QMatrix3x3和QMatrix4x4支持）
sudo apt-get install libqt5opengl5-dev -y

# 安装其他系统依赖
sudo apt-get install libopengl0 libglx0 libgl1-mesa-dev

# 安装Python依赖
pip3 install numpy opencv-python torch torchvision

# 安装YOLOv8
pip3 install ultralytics

# 安装消息和功能包依赖
sudo apt-get install ros-noetic-moveit-msgs ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-tf2-ros

# 安装通信和网络依赖
sudo apt-get install ros-noetic-image-transport
```

### 工作空间目录结构

系统包含以下ROS功能包：
- **arm_gui**: 机械臂控制GUI界面
- **arm_trajectory**: 轨迹规划和生成
- **liancheng_socket**: 网络通信接口（原名SimpleNetwork）
- **servo_wrist**: 舵机控制
- **stereo_vision**: 立体视觉处理

### 编译工作空间

```bash
# 克隆仓库
git clone https://github.com/jfy9606/rosarm.git
cd rosarm

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

系统支持双目OV4689摄像头，使用MJPEG格式，分辨率1280x480，30FPS。代码已针对此配置进行了优化。

系统自动支持宽幅摄像头，会自动检测图像宽高比，并处理左半部分图像。这对于宽幅图像尤其有用。

摄像头参数可以在以下文件中配置：
- `launch/demo.launch`
- `launch/camera_test.launch`
- `launch/direct_camera.launch`

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

1. **编译错误**
   - 确保所有依赖包都已安装
   - 检查工作空间中的包名是否与CMakeLists.txt中的包名一致
   - 如果出现liancheng_socket相关错误，确保SimpleNetwork目录已被重命名为liancheng_socket

2. **Qt库问题**
   - 如果出现 `libQt5Core.so.5: cannot open shared object file` 错误，需要执行以下修复：
   ```bash
   # 创建Qt库链接
   sudo strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
   
   # 如果上面的命令不解决问题，尝试下面的方法
   sudo apt-get install --reinstall libqt5core5a
   
   # 设置Qt库路径
   export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
   
   # 为了永久生效，可以将上面的命令添加到 ~/.bashrc 文件中
   echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **摄像头不工作**
   - 检查摄像头设备是否存在: `ls -la /dev/video*`
   - 确保摄像头权限正确: `sudo chmod a+rw /dev/video*`
   - 尝试直接测试模式: `./start.sh --test`

4. **图像显示问题**
   - 检查分辨率设置是否与摄像头匹配
   - 如果图像为黑屏，可能是摄像头曝光问题
   - 尝试不同的像素格式 (YUYV, MJPEG)

5. **目标检测不工作**
   - 确保已安装所需的Python库
   - 检查YOLO模型路径是否正确

6. **轨迹规划错误**
   - 检查机械臂配置参数
   - 确保已安装所有ROS依赖包

7. **Python导入错误**
   - 如果出现 `cannot import name 'WhaleOptimizer' from 'whales_optimizer'` 错误，需要检查：
   ```bash
   # 检查文件是否存在
   ls -la ~/catkin_ws/src/arm_trajectory/scripts/whales_optimizer.py
   
   # 确保Python版本兼容
   python3 --version
   
   # 确保没有语法错误，可以尝试手动导入
   cd ~/catkin_ws/src/arm_trajectory/scripts/
   python3 -c "from whales_optimizer import WhaleOptimizer, forward_kinematics_dh"
   
   # 检查文件权限
   chmod +x ~/catkin_ws/src/arm_trajectory/scripts/*.py
   ```

8. **Qt信号槽警告**
   - 如果出现 `QMetaObject::connectSlotsByName: No matching signal for on_XXXButton_clicked()` 警告，这通常是UI文件中定义的按钮没有对应的槽函数。您可以：
     - 在相应的类中添加缺失的槽函数
     - 或修改UI文件，移除未使用的按钮连接

9. **摄像头超时问题**
   - 如果出现 `VIDEOIO(V4L2:/dev/video0): select() timeout` 或 `摄像头读取失败` 警告：
   ```bash
   # 检查摄像头设备
   ls -la /dev/video*
   
   # 重置摄像头
   sudo modprobe -r uvcvideo
   sudo modprobe uvcvideo
   
   # 检查摄像头是否被其他进程占用
   sudo fuser -v /dev/video*
   
   # 如果摄像头被占用，终止相关进程
   sudo fuser -k /dev/video0
   
   # 尝试使用其他摄像头设备
   # 修改launch文件中的camera_device参数为其他可用设备
   ```

10. **GUI无法显示摄像头画面但guvcview正常**
    - 如果guvcview能正常显示摄像头画面，但ROS GUI无法显示，可能是视频格式或分辨率配置问题：
    ```bash
    # 查看摄像头支持的格式和分辨率
    v4l2-ctl --list-formats-ext -d /dev/video0
    
    # 检查并记录guvcview使用的成功配置
    guvcview -d /dev/video0 -g
    
    # 修改camera_node.py或相关launch文件中的摄像头参数，与guvcview成功的配置保持一致：
    # - 像素格式 (如MJPG, YUYV)
    # - 分辨率
    # - 帧率
    
    # 检查ROS图像传输是否工作
    rostopic list | grep image
    rostopic echo /camera/image_raw/header -n 1
    ```
    
    - 可能需要修改以下ROS相关配置：
      - 检查cv_bridge是否正确将OpenCV图像转换为ROS图像消息
      - 确保使用了正确的图像编码格式(如bgr8, mono8等)
      - 检查图像话题的发布和订阅是否正确
      - 尝试使用rqt_image_view工具检查图像话题是否有发布

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

## GUI界面功能

GUI界面提供了以下功能：

1. **关节控制**：通过滑块直接控制机械臂各关节
2. **视觉检测控制**：
   - 启用/禁用YOLO目标检测
   - 显示检测到的物体
   - 从检测表格中选择物体进行操作
3. **路径规划控制**：
   - 扫描场景中的物体
   - 规划从选定物体到指定放置区的路径
   - 执行规划的路径
   - 可视化工作空间
4. **末端执行器控制**：
   - 控制夹持器开合
   - 控制吸盘吸附
5. **示例动作**：预设的演示动作

## 启动方式

启动完整系统：

```bash
source devel/setup.bash
roslaunch arm_gui arm_gui.launch
```

仅启动GUI界面（不包含路径规划和视觉处理）：

```bash
source devel/setup.bash
roslaunch arm_gui arm_gui.launch with_path_planner:=false with_trajectory_planner:=false with_vision_arm_bridge:=false
```

## 使用方法

1. 启动系统后，GUI界面会显示机械臂控制面板
2. 在"视觉检测"区域勾选"启用YOLO目标检测"以开始检测物体
3. 点击"扫描物体"按钮扫描工作空间中的物体
4. 在检测表格中选择要操作的物体
5. 在"路径规划"区域选择放置区，然后点击"规划路径"
6. 点击"执行路径"开始执行抓取和放置任务
7. 点击"可视化工作空间"可以在RViz中查看工作空间布局

## 系统架构

系统由以下主要模块组成：

- **arm_gui**：图形用户界面
- **stereo_vision**：立体视觉和目标检测
- **arm_trajectory**：路径规划和轨迹生成
- **servo_wrist**：舵机控制
- **liancheng_socket**：通信网络（原名SimpleNetwork）

## 注意事项

- 确保摄像头设备可用
- 路径规划需要正确配置的DH参数和关节限制
- 视觉检测需要正确的相机标定参数

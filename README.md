# ROS 机械臂控制系统

本项目是一个基于ROS的机械臂控制系统，包含立体视觉、轨迹规划和机械臂控制功能。

## 环境配置指南

### 1. 安装必要的Python库

系统依赖以下Python库：

```bash
# 安装ultralytics用于YOLO目标检测
pip3 install ultralytics

# 安装opencv-contrib-python用于立体视觉和深度图处理
pip3 install opencv-contrib-python
```

### 2. 下载YOLO模型

系统使用YOLO11n模型进行目标检测，您可以从以下地址下载模型文件：

```bash
# 下载YOLO11n模型
wget https://ghproxy.cfd/https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt

# 将模型文件放置在当前目录或指定路径
# 如果使用其他路径，请在启动系统时通过yolo_model_path参数指定
```

### 3. 验证安装

执行以下命令验证环境配置是否正确：

```bash
# 验证ultralytics安装
python3 -c "import ultralytics; print(f'Ultralytics版本: {ultralytics.__version__}')"

# 验证WhaleOptimizer模块
python3 -c "import sys; sys.path.append('$(pwd)/src/arm_trajectory/scripts'); import whales_optimizer; print('WhaleOptimizer导入成功')"
```

### 4. Docker容器环境特殊配置

如果在Docker容器中运行，可能需要额外的权限：

```bash
# 如果需要超级用户权限
sudo -i
```

## 运行系统

在配置好环境后，使用以下命令启动系统：

```bash
# 启动ROS核心
roscore &

# 启动机械臂控制节点
roslaunch arm_control arm_bringup.launch

# 在新终端中启动立体视觉节点
roslaunch stereo_vision stereo_arm_control.launch

# 在新终端中启动轨迹规划节点
roslaunch arm_trajectory trajectory_planning.launch
```

## 故障排除

### 导入错误

如果遇到模块导入错误：

1. 检查PYTHONPATH环境变量是否正确设置：
   ```bash
   echo $PYTHONPATH
   ```

2. 确认ultralytics是否安装成功：
   ```bash
   pip3 list | grep ultralytics
   ```

3. 检查WhaleOptimizer文件是否存在于正确位置：
   ```bash
   ls -l $(pwd)/src/arm_trajectory/scripts/whales_optimizer.py
   ls -l /home/test/rosarm/devel/.private/arm_trajectory/lib/arm_trajectory/whales_optimizer.py
   ```

如果仍然无法导入模块，可以尝试使用绝对导入路径：

```python
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
```

### 机械臂未响应

如果机械臂未响应命令：

1. 检查ROS节点是否在运行：
   ```bash
   rosnode list
   ```

2. 检查主题是否正确发布：
   ```bash
   rostopic list
   rostopic echo /arm1/trajectory_command
   ```

3. 检查硬件连接和串口权限：
   ```bash
   ls -l /dev/ttyUSB*
   sudo chmod a+rw /dev/ttyUSB*
   ```

### "renderRobot: 关节数据为空"错误

如果GUI显示"renderRobot: 关节数据为空"错误或3D视图中机械臂不显示：

1. 检查关节状态话题是否存在：
   ```bash
   rostopic info /arm1/joint_states
   ```

2. 如果话题不存在，启动joint_state_publisher节点：
   ```bash
   # 使用test_joints.launch进行测试
   roslaunch test_joints.launch
   
   # 或在已有系统中启动关节发布节点
   rosrun liancheng_socket joint_state_publisher
   ```

3. 检查关节数据是否正确发布：
   ```bash
   rostopic echo /arm1/joint_states -n 1
   ```

4. 如果需要模拟关节数据进行测试：
   ```bash
   rosrun liancheng_socket test_joint_publisher
   ```

### "深度图不可用，切换到左图模式"错误

如果系统显示"深度图不可用，切换到左图模式"或"cv2.ximgproc not available, advanced depth processing will be disabled"错误：

1. 确认已安装opencv-contrib-python：
   ```bash
   # 检查是否安装了opencv-contrib-python
   pip3 list | grep opencv-contrib-python
   
   # 如果未安装，请安装
   pip3 install opencv-contrib-python
   
   # 验证ximgproc模块是否可用
   python3 -c "import cv2.ximgproc; print('ximgproc可用')"
   ```

2. 检查摄像头配置：
   ```bash
   # 检查摄像头是否支持双目模式
   v4l2-ctl --list-formats-ext -d /dev/video0
   
   # 确认launch文件中正确启用深度图处理
   # 在visual_servo.launch文件中，确保use_depth参数设为true
   ```

3. 重启摄像头节点：
   ```bash
   # 重启立体相机节点
   rosnode kill /stereo_camera_node
   roslaunch arm_trajectory visual_servo.launch
   ```

4. 尝试切换到深度图模式：
   ```bash
   # 发布视图模式切换消息（2表示深度图模式）
   rostopic pub -1 /stereo_camera/view_mode std_msgs/Int32 "data: 2"
   ```

## 系统组件

系统由以下主要组件组成：

1. **摄像头** - 支持高分辨率USB摄像头，包括宽幅摄像头
2. **图像处理** ：
   - 使用OpenCV和YOLO进行图像处理和目标检测
   - 支持深度图处理 - 使用opencv-contrib-python中的ximgproc模块进行立体匹配和深度图生成
3. **机械臂控制** - 基于ROS的机械臂控制界面和轨迹规划
4. **关节状态发布** - 负责将电机命令转换为关节状态并发布到/arm1/joint_states话题

### 机械臂关节配置

当前系统有四个可用关节，每个关节的功能如下：

1. **关节1** - 俯仰电机（第一级俯仰）
2. **关节2** - 直线进给电机（伸缩）
3. **关节3** - 绕轴旋转电机（旋转）
4. **关节4** - 俯仰电机（第二级俯仰）

注：系统设计有六个关节，但当前只有上述四个关节可用。另外两个关节处于未部署状态。

## 系统特性

### 末端执行器摄像头

系统支持在机械臂末端执行器（吸盘）上安装摄像头，提供移动视角。系统特性包括：

1. **移动视角** - 摄像头跟随机械臂移动，提供末端执行器视角的物体检测
2. **坐标系转换** - 自动在摄像头坐标系和世界坐标系之间进行转换
3. **动态显示** - 3D场景中物体位置会根据摄像头位置自动更新

这一特性使系统能够从吸取物体的视角检测和处理物体，提高了抓取精度。

### 3D机械臂可视化

系统提供了3D机械臂可视化功能，允许用户在GUI界面中直观地观察机械臂当前姿态：

1. **实时渲染** - 根据关节状态实时渲染机械臂3D模型
2. **交互控制** - 支持通过鼠标旋转和缩放视图
3. **物体选择** - 可在3D视图中选择和操作检测到的物体

## 安装指南

### 前提条件

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- OpenCV 4.2+

### 配置ROS软件源

```bash
# 添加ROS软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 添加密钥
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 更新软件包列表
sudo apt update
```

### 安装依赖

```bash
# 安装ROS基本依赖
sudo apt-get update
sudo apt-get install ros-noetic-image-view ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-serial ros-noetic-moveit-msgs ros-noetic-geometry-msgs ros-noetic-sensor-msgs ros-noetic-image-transport ros-noetic-cmake-modules

# 安装Qt5依赖
sudo apt-get install qtbase5-dev qt5-default qtchooser qttools5-dev-tools qttools5-dev libqt5core5a libqt5gui5 libqt5widgets5 libqt5opengl5 libqt5opengl5-dev ros-noetic-rqt* ros-noetic-qt-gui* -y

# 修复Qt5库问题（解决libQt5Core.so.5错误）
sudo strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# 安装系统依赖
sudo apt-get install libopengl0 libglx0 libgl1-mesa-dev freeglut3-dev -y

# 安装Python依赖
pip3 install numpy opencv-contrib-python torch torchvision ultralytics

# 验证YOLO安装
python3 -c "from ultralytics import YOLO; print('YOLOv8安装成功')"

# 下载YOLO模型权重（可选）
mkdir -p models
wget -P models/ https://github.com/ultralytics/assets/releases/download/latest/yolov8n.pt  # nano版本
```

### 安装配置Catkin工具

```bash
# 安装python-catkin-tools
sudo apt-get install python3-catkin-tools python3-osrf-pycommon -y

# 验证安装
catkin --version
```

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

# 测试机械臂3D渲染
roslaunch test_joints.launch
```

## 相机使用说明

### HBVCAM-4M2214HD-2 双目相机参数

本系统使用HBVCAM-4M2214HD-2 V11双目相机，其主要参数如下：

- **感光芯片**: OV4689(1/3) CMOS, 400万像素
- **基线距离**: 60mm（左镜头中线到右镜头中线距离）
- **视场角/焦距**: 72°有畸变/焦距3.6mm 或 80°无畸变/焦距3.0mm
- **输出格式**: MJPEG/YUY2
- **接口**: USB 2.0

### 支持的分辨率

该相机支持以下分辨率和帧率组合：

#### MJPEG格式（推荐）
- 1280x480 @ 30FPS（默认，左右双目合并）
- 1920x1080 @ 30FPS
- 2160x1080 @ 30FPS
- 2560x720 @ 30FPS
- 3840x1080 @ 30FPS
- 3840x1520 @ 10FPS

#### YUY2格式
- 1280x480 @ 3FPS
- 1920x1080 @ 3FPS
- 2160x1080 @ 3FPS
- 2560x720 @ 1FPS
- 3840x1080 @ 1FPS

### ROS话题

相机节点提供以下ROS话题：

- `/stereo_camera/left/image_raw` - 左相机图像
- `/stereo_camera/right/image_raw` - 右相机图像（双目模式下）
- `/stereo_camera/current_view` - 当前视图（左图/右图/深度图）
- `/stereo_camera/depth/image_raw` - 彩色深度图（如果启用深度计算）
- `/stereo_camera/depth/raw` - 原始深度数据（32位浮点，单位：像素视差）

### 启动方式

#### 标准分辨率 (1280x480)
```bash
roslaunch arm_trajectory stereo_camera.launch
```

#### 高分辨率 (3840x1080)
```bash
roslaunch arm_trajectory stereo_camera_highres.launch
```

#### 开发/测试模式（使用模拟相机）
当实际相机不可用或在开发环境中，可以使用模拟相机模式：

```bash
roslaunch arm_trajectory mock_camera.launch
```

#### 自定义分辨率
如需使用其他分辨率，可以修改launch文件中的相关参数：
```xml
<param name="camera_width" value="1280" />
<param name="camera_height" value="480" />
<param name="camera_format" value="MJPEG" />
<param name="frame_rate" value="30" />
```

### 图像处理说明

1. **图像分割**: 由于相机输出的是左右合并图像，我们的软件会自动将合并图像分割为左右两个独立图像。

2. **深度计算**: 利用左右图像进行视差计算，生成深度图。深度计算使用OpenCV的SGBM算法。

3. **焦距与基线**: 该相机基线距离为60mm，焦距为3.6mm，这些参数已在系统中配置，用于正确的深度计算。

### 参数配置

可通过ROS参数配置相机节点行为：

- `camera_index` (int): 相机设备索引，默认为0
- `frame_rate` (int): 目标帧率，默认为30
- `use_depth` (bool): 是否启用深度计算，默认为true
- `view_mode` (int): 默认视图模式，0=左图，1=右图，2=深度图
- `use_mock_camera` (bool): 是否使用模拟相机，默认为false
- `mock_camera_pattern` (string): 模拟相机图案类型，默认为'checkerboard'
- `camera_width` (int): 相机分辨率宽度，默认为1280
- `camera_height` (int): 相机分辨率高度，默认为480
- `camera_format` (string): 相机格式，默认为MJPEG

### 常见问题与排查

#### 1. 图像偏色或过暗/过亮

手动调节焦距（扭转镜头）可能会改变图像的曝光和色彩。如遇到这种情况，可以尝试以下方法：

```python
# 调整相机参数示例
camera.set(cv2.CAP_PROP_BRIGHTNESS, 128)  # 亮度 (0-255)
camera.set(cv2.CAP_PROP_CONTRAST, 128)    # 对比度 (0-255)
camera.set(cv2.CAP_PROP_SATURATION, 128)  # 饱和度 (0-255)
```

#### 2. 无法打开指定分辨率

如果启动时报错无法设置指定分辨率，请确认：

1. 检查相机是否支持该分辨率（参见上方列表）
2. 对于高分辨率，确保USB带宽足够
3. 使用`v4l2-ctl --list-formats-ext -d /dev/video0`检查实际支持的分辨率

#### 3. 相机无法打开

如果出现"无法打开相机设备"或类似错误：

- 确认相机已正确连接到USB端口
- 检查相机设备是否存在：`ls -l /dev/video*`
- 检查当前用户是否有相机访问权限：`sudo usermod -a -G video $USER`
- 尝试使用其他相机索引：修改launch文件中的`camera_index`参数
- 尝试使用v4l2测试相机：`v4l2-ctl --list-devices` 和 `v4l2-ctl -d /dev/videoX --list-formats-ext`

#### 4. 相机超时或读取失败

如果遇到"select() timeout"或"读取测试帧失败"等错误：

- 相机可能被其他程序占用，请关闭其他可能使用相机的应用
- 检查USB连接是否稳定，尝试更换USB端口
- 如果在虚拟机中运行，确保USB设备已正确传递给虚拟机
- 尝试减小帧率和分辨率
- 检查相机驱动是否兼容：`dmesg | grep -i video`

#### 5. 深度图不准确

双目相机的深度计算依赖于精确的相机标定。如果深度图不准确，可能需要：

1. 重新调整镜头焦距，确保两个镜头焦距一致
2. 在`stereo_camera_node.py`中微调视差计算参数

### 性能优化建议

1. 对于低性能设备，使用较低的分辨率（1280x480）
2. 如果不需要实时深度计算，可以设置`use_depth`参数为`false`
3. 高分辨率模式下，降低帧率可以减少CPU/GPU负载

### 维护与清理

1. 保持镜头清洁，使用专业镜头纸和气吹清理灰尘
2. 避免长时间暴露在强光下
3. 使用后将相机存放在干燥环境中

### 系统要求

- OpenCV: 4.2.0或更高版本
- opencv-contrib-python（用于高级深度处理）
- ROS: Noetic或更高版本
- Python: 3.6或更高版本

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

11. **3D视图中机械臂不显示**
    - 如果GUI中3D视图显示"renderRobot: 关节数据为空"：
    ```bash
    # 检查关节状态话题是否存在
    rostopic list | grep joint_states
    
    # 检查关节状态发布节点是否运行
    rosnode list | grep joint_publisher
    
    # 如果节点不存在，启动测试节点
    roslaunch test_joints.launch
    
    # 或单独启动关节发布节点
    rosrun liancheng_socket joint_state_publisher
    
    # 检查关节数据是否正确发布
    rostopic echo /arm1/joint_states -n 1
    ```

12. **节点无法启动（Cannot locate node）错误**
    - 如果遇到 `ERROR: cannot launch node of type [package_name/node_name]: Cannot locate node` 错误：
    ```bash
    # 检查节点可执行文件是否存在
    find ~/arm/rosarm/devel/lib -name "node_name"
    
    # 检查节点可执行文件是否有执行权限
    chmod +x ~/arm/rosarm/devel/lib/package_name/node_name
    
    # 检查CMakeLists.txt中是否正确定义了节点
    cat ~/arm/rosarm/src/package_name/CMakeLists.txt | grep add_executable
    
    # 重新编译工作空间
    cd ~/arm/rosarm && catkin build
    source devel/setup.bash
    ```

13. **模块导入错误（No module named 'package.msg'）**
    - 如果遇到 `ModuleNotFoundError: No module named 'package_name.msg'` 错误：
    ```bash
    # 检查消息文件是否存在
    find ~/arm/rosarm/src -name "*.msg"
    
    # 检查CMakeLists.txt中是否启用了消息生成
    # 确保add_message_files部分已取消注释并包含正确的.msg文件
    
    # 重新编译工作空间
    cd ~/arm/rosarm && catkin build
    source devel/setup.bash
    
    # 验证消息是否已生成
    find ~/arm/rosarm/devel -path "*package_name/msg*"
    ```

## 系统架构

系统采用模块化设计，主要包括以下ROS节点：

1. `camera_node` - 读取摄像头图像并发布到ROS话题
2. `yolo_detector` - 基于YOLO的目标检测
3. `trajectory_bridge` - 轨迹规划和执行
4. `arm_gui` - 用户界面
5. `joint_state_publisher` - 关节状态发布

各节点之间通过ROS话题进行通信，主要话题包括：

- `/camera/image_raw` - 摄像头图像
- `/camera/detections` - 检测结果
- `/stereo_camera/left/image_raw` - 左相机图像
- `/stereo_camera/right/image_raw` - 右相机图像
- `/stereo_camera/depth/image_raw` - 深度图像 
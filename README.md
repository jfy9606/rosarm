# SCServo 机械臂控制

一个用于控制机械臂的跨平台Python程序，支持使用Feetech-Servo-SDK控制SCServo（FT）舵机和DC直流电机。集成OpenCV视觉功能。

## 功能特点

- 使用官方Feetech-Servo-SDK控制多达6个SCServo舵机关节（目前实现4个）
- 支持2个DC大电机（YF俯仰和AImotor直线进给）
- 通过两个串口进行控制，可选择性连接舵机或电机
- 跨平台兼容（Windows，Linux和macOS）
- 支持多种控制模式：单关节，同步多关节，笛卡尔坐标等
- 集成OpenCV视觉处理功能，支持物体检测和跟踪
- 直观的Tkinter图形界面

## 系统要求

- Python 3.6+
- pySerial库
- OpenCV (opencv-python)
- PIL/Pillow库（用于图像处理）
- NumPy
- 兼容的SCServo（FT）舵机
- 两个串行端口（用于舵机和DC电机控制）
- 可选：USB摄像头（用于视觉功能）

## 安装

1. 克隆仓库或下载源码
2. 安装依赖库:

```bash
pip install -r requirements.txt
```

## 使用方法

### 启动图形用户界面

```bash
python main.py
```

或者直接运行:

```bash
python robot_arm_gui.py
```

### GUI界面使用说明

GUI界面分为三个选项卡：

1. **基本控制** - 连接设置、关节控制和电机控制
2. **笛卡尔控制** - 基于笛卡尔坐标的移动和演示
3. **视觉** - 摄像头视图和物体检测功能

#### 连接设备

1. 从下拉菜单中选择舵机和/或电机串口
2. 设置适当的波特率（舵机通常为1000000，电机通常为115200）
3. 点击"连接舵机"和/或"连接电机"按钮
4. 连接后，使用相应的"使能"按钮启用设备

#### 控制功能

- **舵机控制**：使用滑块调整各关节位置，点击"设置"应用更改
- **电机控制**：调整YF俯仰电机和AImotor进给电机的位置和速度
- **演示功能**：执行预设的动作演示（抓取-放置，方形轨迹）
- **视觉功能**：开启摄像头，启用视觉处理，调整HSV参数检测物体

## 在你的项目中使用

```python
from src import RobotArm

# 初始化机械臂
robot = RobotArm()

# 选择性连接到控制器
robot.connect_servo('/dev/ttyUSB0', 1000000)
robot.connect_motor('/dev/ttyUSB1', 115200)

# 使能舵机
robot.enable_torque(True)

# 设置运动速度
robot.set_speeds(servo_speed=500)
robot.motor.set_motor_speed(100, 100)  # YF俯仰速度, AImotor进给速度

# 移动单个关节
robot.set_joint_position('joint1', 2048, blocking=True)

# 同时移动多个关节
positions = {
    'joint1': 2048,
    'joint2': 2048,
    'joint3': 2048,
    'joint4': 2048
}
robot.set_joint_positions(positions, blocking=True)

# 控制DC电机
robot.set_pitch_position(1000)
robot.set_linear_position(5000)

# 结束后清理
robot.enable_torque(False)
robot.disconnect()
```

## 视觉功能

集成的OpenCV视觉系统支持:

- 连接USB摄像头并显示视频流
- 基于HSV颜色过滤进行物体检测
- 轮廓识别和中心点计算
- 对检测到的物体进行跟踪或抓取（可根据实际应用场景开发）

## 项目结构

- `src/feetech_servo_controller.py` - Feetech舵机控制类，使用Feetech-Servo-SDK
- `src/dcmotor.py` - DC电机控制类
- `src/robot_arm.py` - 机械臂主控制类，集成舵机和电机控制
- `src/__init__.py` - 使包可导入
- `robot_arm_gui.py` - 集成OpenCV视觉功能的Tkinter图形界面
- `main.py` - 程序入口
- `components/Feetech-Servo-SDK/` - Feetech官方舵机SDK

## 注意事项

- 本程序使用Feetech官方的SCServo SDK来控制舵机
- 支持选择性连接小臂舵机或大臂电机，可在设备不可用时仍能部分工作
- 对于不同型号的舵机，可能需要调整协议参数
- 使用前请确保已正确连接舵机和电源，并检查舵机工作电压

## 许可证

MIT License 
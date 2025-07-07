# SCServo Robot Arm Control

一个用于控制机械臂的跨平台Python程序，支持SCServo（FT）舵机和DC直流电机。

## 功能特点

- 支持控制多达6个SCServo舵机关节（目前实现4个）
- 支持2个DC大电机（俯仰和直线进给）
- 通过两个串口进行控制
- 跨平台兼容（Windows，Linux和macOS）
- 支持多种控制模式：单关节，同步多关节，笛卡尔坐标等

## 系统要求

- Python 3.6+
- pySerial库
- 兼容的SCServo（FT）舵机
- 两个串行端口（用于舵机和DC电机控制）

## 安装

1. 克隆仓库或下载源码
2. 安装依赖库:

```bash
pip install -r requirements.txt
```

## 使用方法

### 直接运行演示程序

```bash
python demo.py
```

程序会自动检测可用的串行端口，并让你选择用于舵机和DC电机的端口。

### 指定串行端口

```bash
python demo.py --servo-port COM3 --motor-port COM4
```

### 选择演示类型

```bash
python demo.py --demo basic  # 基本关节移动演示
python demo.py --demo dc     # DC电机演示
python demo.py --demo cartesian  # 笛卡尔坐标移动演示
python demo.py --demo pick   # 简单抓取放置演示
python demo.py --demo all    # 所有演示（默认）
```

## 在你的项目中使用

```python
from arm_control.src import RobotArm

# 初始化机械臂
robot = RobotArm()

# 连接到控制器
robot.connect("COM3", "COM4")

# 启用力矩
robot.enable_torque(True)

# 移动关节
robot.set_joint_position('joint1', 2048)  # 中间位置

# 同时控制多关节
positions = {
    'joint1': 2048,
    'joint2': 1500,
    'joint3': 2500,
    'joint4': 2048
}
robot.set_joint_positions(positions, blocking=True)

# 控制DC电机
robot.set_pitch_position(1000)  # 控制俯仰
robot.set_linear_position(5000)  # 控制直线进给

# 回到原位
robot.home()

# 断开连接
robot.disconnect()
```

## 系统架构

- `scservo.py`: SCServo通信类，处理舵机的命令和响应
- `dcmotor.py`: DC电机控制器，处理大电机的命令和响应
- `robot_arm.py`: 主要的机械臂控制类，集成舵机和电机控制
- `demo.py`: 演示程序

## 注意事项

- 确保在使用前正确连接硬件
- 舵机和DC电机使用不同的串口进行通信
- 在开始使用前，建议先校准机械臂

## 许可证

MIT License 
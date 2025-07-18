# SCServo 机械臂控制

一个用于控制机械臂的跨平台Python程序，支持使用Feetech-Servo-SDK控制SCServo（FT）舵机和DC直流电机。

## 功能特点

- 使用官方Feetech-Servo-SDK控制多达6个SCServo舵机关节（目前实现4个）
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

### 设置协议参数

```bash
# 对于 SCS 协议
python demo.py --protocol-end 1

# 对于 STS/SMS 协议
python demo.py --protocol-end 0
```

### 选择演示类型

```bash
python demo.py --demo basic  # 基本关节移动演示
python demo.py --demo dc     # DC电机演示
python demo.py --demo cartesian  # 笛卡尔坐标移动演示
python demo.py --demo pick   # 简单抓取放置演示
python demo.py --demo scan   # 扫描连接的舵机
python demo.py --demo all    # 所有演示（默认）
```

### 测试脚本

项目包含了几个专门用于测试电机通信的脚本，位于 `test` 文件夹中。这些测试脚本支持自动扫描可用串口和自适应波特率功能，可以帮助诊断电机通信问题：

#### 测试所有电机

```bash
# 自动扫描所有可用串口并测试
python -m test.motor_test --scan

# 指定串口进行测试
python -m test.motor_test --port COM3
```

#### 测试 YF 俯仰电机

```bash
# 自动扫描所有可用串口并尝试不同波特率
python -m test.test_yf_motor

# 指定串口和波特率
python -m test.test_yf_motor --port COM4 --baudrate 115200
```

#### 测试 AIMotor 进给电机

```bash
# 自动扫描所有可用串口并尝试不同波特率
python -m test.test_ai_motor

# 指定串口和波特率
python -m test.test_ai_motor --port COM4 --baudrate 115200
```

测试脚本会自动尝试与电机建立通信，并执行基本的位置控制命令。如果不指定串口，脚本会自动扫描所有可用串口；如果不指定波特率，脚本会尝试常用波特率（115200, 9600, 19200, 38400, 57600）。

## 在你的项目中使用

```python
from src import RobotArm

# 初始化机械臂
robot = RobotArm(protocol_end=0)  # STS/SMS=0, SCS=1

# 连接到控制器
robot.connect('/dev/ttyUSB0', '/dev/ttyUSB1')

# 使能舵机
robot.enable_torque(True)

# 设置运动速度
robot.set_speeds(servo_speed=500, pitch_speed=100, linear_speed=100)

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
robot.set_pitch_position(1000, blocking=True)
robot.set_linear_position(5000, blocking=True)

# 使用笛卡尔坐标控制
robot.move_cartesian(10, 15, 5)

# 扫描连接的舵机
servos = robot.scan_servos(1, 10)

# 结束后清理
robot.enable_torque(False)
robot.disconnect()
```

## 项目结构

- `src/feetech_servo_controller.py` - Feetech舵机控制类，使用Feetech-Servo-SDK
- `src/dcmotor.py` - DC电机控制类
- `src/robot_arm.py` - 机械臂主控制类，集成舵机和电机控制
- `src/__init__.py` - 使包可导入
- `demo.py` - 演示程序
- `components/Feetech-Servo-SDK/` - Feetech官方舵机SDK
- `test/motor_test.py` - 测试所有电机通信
- `test/test_yf_motor.py` - 专门测试YF俯仰电机
- `test/test_ai_motor.py` - 专门测试AIMotor进给电机
- `test/test_utils.py` - 测试工具模块，用于正确导入src模块

## 注意事项

- 本程序使用Feetech官方的SCServo SDK来控制舵机
- 对于不同型号的舵机，可能需要调整协议参数（使用--protocol-end参数）
- 使用前请确保已正确连接舵机和电源

## 许可证

MIT License 
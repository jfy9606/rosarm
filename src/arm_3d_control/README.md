# 机械臂3D位置控制

本ROS功能包提供了一个接口，允许通过输入三维坐标来控制机械臂末端移动到指定位置。

## 功能

- 接收目标3D坐标点（x, y, z）
- 计算逆运动学，得到各关节的旋转角度
- 控制舵机移动到指定位置

## 使用方法

### 编译

在ROS工作空间的根目录下编译：

```
catkin_make
source devel/setup.bash
```

### 启动

启动所有相关节点：

```
roslaunch arm_3d_control arm_3d_control.launch
```

### 发送位置命令

运行提供的示例客户端来发送3D位置：

```
rosrun arm_3d_control position_sender
```

然后按提示输入x、y、z坐标（单位：mm）。

### 使用ROS话题发送命令

也可以通过发布ROS消息来控制机械臂位置：

```
rostopic pub /arm_target_position geometry_msgs/Point "x: 100.0
y: 100.0
z: 150.0"
```

## 配置参数

机械臂的尺寸参数和舵机ID可以在以下文件中修改：

```
include/arm_3d_control/arm_parameters.h
```

主要参数：
- LINK1_LENGTH: 底座到第一个关节的距离
- LINK2_LENGTH: 第一个关节到第二个关节的距离
- LINK3_LENGTH: 第二个关节到末端执行器的距离
- 各舵机ID和控制参数 
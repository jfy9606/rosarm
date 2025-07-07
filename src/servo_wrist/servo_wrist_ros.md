# 腕关节数字舵机控制

#### 非ROS（舵机测试） 

```
cd ./src/SCServo_Linux/examples/SMS_STS/WritePos进入测试案例（已编译，可直接运行）
运行:sudo ./WritePos /dev/ttyUSB0
注:/dev/ttyUSB0根据设备实际串口指定
```

<font color = 'red'>端口查询方式：新开命令端， ls /dev/tty*</font>

>编译方式：
>
>使用cmake .命令生成Makefile,再使用make命令生成可执行文件WritePos



#### ROS下游控制 

目前实现了接收msg，并针对不同ID的舵机进行writePos的控制

```
//启动方法
rosrun servo_wrist wrist_ctrl /dev/ttyUSB0
注:/dev/ttyUSB0根据设备实际串口指定
```

>ROS编译方式：
>
>1、将servo_wrist文件夹放在workspace的src文件夹下，回到ws下catkin_make即可。
>
>2、如果不行可以新建ros pkg，再将pkg的<font color = 'red'>src文件夹、CmakeList、package</font>进行替换，再进行编译

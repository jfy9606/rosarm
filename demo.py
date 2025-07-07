#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Feetech SCServo 机械臂控制演示程序
一个使用 Feetech-Servo-SDK 控制带有 SCServo 舵机和 DC 电机的机械臂的演示
"""

import os
import sys
import time
import argparse
import platform
from src.robot_arm import RobotArm
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'components', 'Feetech-Servo-SDK'))
from scservo_sdk import COMM_SUCCESS


def detect_serial_ports():
    """
    检测系统上可用的串行端口
    
    Returns:
        可用串行端口列表
    """
    import serial.tools.list_ports
    
    ports = list(serial.tools.list_ports.comports())
    
    available_ports = []
    for p in ports:
        available_ports.append(p.device)
    
    return available_ports


def demo_basic_movement(robot, args):
    """
    演示机械臂的基本运动
    """
    print("校准机械臂...")
    robot.calibrate()
    time.sleep(1)
    
    print("设置加速度...")
    robot.set_all_servo_acc(5)  # 低加速度，平滑运动
    
    print("依次移动关节...")
    # 单独移动每个关节
    robot.set_speeds(servo_speed=500)  # 演示时使用较慢的速度
    
    print("移动关节 1...")
    robot.set_joint_position('joint1', 1500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint1', 2500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint1', 2048, blocking=True)  # 中间位置
    
    print("移动关节 2...")
    robot.set_joint_position('joint2', 1500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint2', 2500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint2', 2048, blocking=True)
    
    print("移动关节 3...")
    robot.set_joint_position('joint3', 1500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint3', 2500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint3', 2048, blocking=True)
    
    print("移动关节 4...")
    robot.set_joint_position('joint4', 1500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint4', 2500, blocking=True)
    time.sleep(0.5)
    robot.set_joint_position('joint4', 2048, blocking=True)
    
    print("同时移动所有关节...")
    positions = {
        'joint1': 1500,
        'joint2': 2500,
        'joint3': 1500,
        'joint4': 2500
    }
    robot.set_joint_positions(positions, blocking=True)
    time.sleep(1)
    
    positions = {
        'joint1': 2500,
        'joint2': 1500,
        'joint3': 2500,
        'joint4': 1500
    }
    robot.set_joint_positions(positions, blocking=True)
    time.sleep(1)
    
    # 返回原位
    print("返回原位...")
    robot.home(blocking=True)


def demo_dc_motors(robot, args):
    """
    演示 DC 电机运动（俯仰和线性进给）
    """
    print("测试俯仰电机...")
    robot.set_speeds(pitch_speed=100)
    
    print("俯仰向上...")
    robot.set_pitch_position(1000, blocking=True)
    time.sleep(1)
    
    print("俯仰向下...")
    robot.set_pitch_position(-1000, blocking=True)
    time.sleep(1)
    
    print("俯仰回中...")
    robot.set_pitch_position(0, blocking=True)
    time.sleep(1)
    
    print("测试线性进给电机...")
    robot.set_speeds(linear_speed=100)
    
    print("线性进给伸出...")
    robot.set_linear_position(5000, blocking=True)
    time.sleep(1)
    
    print("线性进给缩回...")
    robot.set_linear_position(0, blocking=True)
    time.sleep(1)


def demo_cartesian_movement(robot, args):
    """
    演示笛卡尔坐标系运动
    """
    print("演示笛卡尔坐标系运动...")
    robot.set_speeds(servo_speed=500, pitch_speed=100, linear_speed=100)
    
    # 在正方形轨迹上移动
    print("沿正方形轨迹移动...")
    robot.move_cartesian(10, 10, 10)
    time.sleep(1)
    
    robot.move_cartesian(10, 20, 10)
    time.sleep(1)
    
    robot.move_cartesian(-10, 20, 10)
    time.sleep(1)
    
    robot.move_cartesian(-10, 10, 10)
    time.sleep(1)
    
    robot.move_cartesian(10, 10, 10)
    time.sleep(1)
    
    # 在垂直轨迹上移动
    print("沿垂直轨迹移动...")
    robot.move_cartesian(0, 15, -10)
    time.sleep(1)
    
    robot.move_cartesian(0, 15, 10)
    time.sleep(1)
    
    robot.move_cartesian(0, 15, 0)
    time.sleep(1)
    
    # 返回原位
    print("返回原位...")
    robot.home(blocking=True)


def demo_pick_and_place(robot, args):
    """
    演示简单的抓取和放置操作
    """
    print("执行抓取和放置演示...")
    robot.set_speeds(servo_speed=500, pitch_speed=100, linear_speed=100)
    
    # 移动到抓取位置
    print("移动到抓取位置...")
    robot.set_joint_position('joint1', 1500, blocking=True)  # 底座旋转
    robot.set_joint_position('joint2', 1700, blocking=True)  # 肩部
    robot.set_joint_position('joint3', 2300, blocking=True)  # 肘部
    robot.set_joint_position('joint4', 2048, blocking=True)  # 手腕
    
    # 降低以抓取
    print("降低以抓取...")
    robot.set_pitch_position(-1000, blocking=True)
    
    # 模拟抓取（在实际机器人中，这将激活夹爪）
    print("抓取物体...")
    time.sleep(1)
    
    # 抓取后提升
    print("抓取后提升...")
    robot.set_pitch_position(0, blocking=True)
    
    # 移动到放置位置
    print("移动到放置位置...")
    robot.set_joint_position('joint1', 2500, blocking=True)  # 旋转底座
    
    # 降低以放置
    print("降低以放置...")
    robot.set_pitch_position(-800, blocking=True)
    
    # 模拟释放（在实际机器人中，这将释放夹爪）
    print("释放物体...")
    time.sleep(1)
    
    # 放置后提升
    print("放置后提升...")
    robot.set_pitch_position(0, blocking=True)
    
    # 返回原位
    print("返回原位...")
    robot.home(blocking=True)


def demo_scan_servos(robot, args):
    """
    扫描并显示连接的舵机
    """
    print("扫描连接的舵机...")
    found_servos = robot.scan_servos(1, 10)
    
    if not found_servos:
        print("未找到任何舵机")
    else:
        print(f"找到 {len(found_servos)} 个舵机:")
        for servo_id, model in found_servos:
            print(f"  ID: {servo_id}, 型号: {model}")
    
    return found_servos


def main():
    """
    演示程序的主函数
    """
    parser = argparse.ArgumentParser(description='Feetech SCServo 机械臂控制演示')
    parser.add_argument('--servo-port', dest='servo_port', help='舵机控制的串行端口')
    parser.add_argument('--motor-port', dest='motor_port', help='电机控制的串行端口')
    parser.add_argument('--servo-baudrate', dest='servo_baudrate', type=int, default=1000000,
                        help='舵机通信波特率')
    parser.add_argument('--motor-baudrate', dest='motor_baudrate', type=int, default=115200,
                        help='电机通信波特率')
    parser.add_argument('--protocol-end', dest='protocol_end', type=int, default=0,
                        help='舵机协议位结束（STS/SMS=0, SCS=1）')
    parser.add_argument('--demo', dest='demo', default='all',
                        choices=['all', 'basic', 'dc', 'cartesian', 'pick', 'scan'],
                        help='要运行的演示类型')
    
    args = parser.parse_args()
    
    # 如果未指定，则检测可用端口
    if not args.servo_port or not args.motor_port:
        print("检测可用串行端口...")
        available_ports = detect_serial_ports()
        
        if not available_ports:
            print("未检测到串行端口。请手动指定端口。")
            sys.exit(1)
        
        print("可用端口:")
        for i, port in enumerate(available_ports):
            print(f"  {i}: {port}")
        
        if not args.servo_port:
            servo_idx = int(input("选择舵机端口索引: "))
            args.servo_port = available_ports[servo_idx]
        
        if not args.motor_port:
            motor_idx = int(input("选择电机端口索引: "))
            args.motor_port = available_ports[motor_idx]
    
    print(f"使用舵机端口: {args.servo_port}，波特率: {args.servo_baudrate}")
    print(f"使用电机端口: {args.motor_port}，波特率: {args.motor_baudrate}")
    
    # 初始化机械臂
    robot = RobotArm(protocol_end=args.protocol_end)
    
    # 连接到控制器
    print("连接到控制器...")
    if not robot.connect(args.servo_port, args.motor_port, 
                        servo_baudrate=args.servo_baudrate, 
                        motor_baudrate=args.motor_baudrate,
                        protocol_end=args.protocol_end):
        print("连接到控制器失败。请检查连接并重试。")
        sys.exit(1)
    
    print("连接成功！")
    
    try:
        # 首先扫描舵机
        if args.demo == 'all' or args.demo == 'scan':
            demo_scan_servos(robot, args)
        
        # 使能所有舵机的力矩
        print("使能所有舵机的力矩...")
        robot.enable_torque(True)
        
        # 运行选定的演示
        if args.demo == 'all' or args.demo == 'basic':
            demo_basic_movement(robot, args)
        
        if args.demo == 'all' or args.demo == 'dc':
            demo_dc_motors(robot, args)
        
        if args.demo == 'all' or args.demo == 'cartesian':
            demo_cartesian_movement(robot, args)
        
        if args.demo == 'all' or args.demo == 'pick':
            demo_pick_and_place(robot, args)
        
        print("演示成功完成！")
        
    except KeyboardInterrupt:
        print("\n演示被用户中断。")
    except Exception as e:
        print(f"演示过程中出错: {e}")
    finally:
        # 清理
        print("停止所有电机...")
        robot.stop_all()
        
        print("禁用力矩...")
        robot.enable_torque(False)
        
        print("断开连接...")
        robot.disconnect()
        
        print("完成！")


if __name__ == "__main__":
    main() 
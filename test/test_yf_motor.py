#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import argparse
from src.dcmotor import DCMotorController

def main():
    """
    专门测试 YF 俯仰电机通信，使用 pos_pinch 方法
    """
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='测试YF俯仰电机通信')
    parser.add_argument('--port', '-p', type=str, default='COM3', help='串口名称，例如COM3、COM4等')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='串口波特率')
    args = parser.parse_args()
    
    # 创建电机控制器对象
    controller = DCMotorController(debug=True)
    
    # 连接到电机控制器
    print(f"尝试连接到串口 {args.port}...")
    if not controller.connect(args.port, args.baudrate):
        print(f"无法连接到串口 {args.port}")
        sys.exit(1)
    
    print(f"成功连接到串口 {args.port}")
    
    # 测试 YF 电机通信
    print("测试 YF 俯仰电机通信...")
    
    # 发送使能命令
    print("发送使能命令...")
    cmd_data = bytearray([
        0x3E,  # YF_START
        0x01,  # YF_STATION
        0x08,  # 命令字节
        0x81,  # 使能字节
        0x00, 0x00, 0x00, 0x00,  # 位置
        0x00,  # 速度
        0x00, 0x00  # 预留字节
    ])
    
    controller.serial.reset_input_buffer()
    controller.serial.write(cmd_data)
    time.sleep(0.5)
    
    if controller.serial.in_waiting:
        response = controller.serial.read(controller.serial.in_waiting)
        print(f"收到响应: {' '.join(f'{b:02X}' for b in response)}")
    else:
        print("未收到响应")
    
    # 使用 pos_pinch 方法发送位置命令
    print("\n使用 pos_pinch 方法设置位置...")
    
    # 测试位置 -25
    print("设置位置到 -25...")
    station_num = 1
    max_speed = 3000  # 30*100
    angle_control = -250000  # -25*10000
    
    controller.pos_pinch(station_num, max_speed, angle_control)
    time.sleep(5)
    
    # 测试位置 0
    print("设置位置回零点...")
    angle_control = 0
    controller.pos_pinch(station_num, max_speed, angle_control)
    time.sleep(5)
    
    # 关闭电机
    print("\n发送禁用命令...")
    cmd_data = bytearray([
        0x3E,  # YF_START
        0x01,  # YF_STATION
        0x08,  # 命令字节
        0x80,  # 禁用字节
        0x00, 0x00, 0x00, 0x00,  # 位置
        0x00,  # 速度
        0x00, 0x00  # 预留字节
    ])
    
    controller.serial.reset_input_buffer()
    controller.serial.write(cmd_data)
    time.sleep(0.5)
    
    if controller.serial.in_waiting:
        response = controller.serial.read(controller.serial.in_waiting)
        print(f"收到响应: {' '.join(f'{b:02X}' for b in response)}")
    else:
        print("未收到响应")
    
    # 断开连接
    controller.disconnect()
    print("测试完成")

if __name__ == "__main__":
    main() 
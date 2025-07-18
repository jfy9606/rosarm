#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import argparse
from src.dcmotor import DCMotorController

def main():
    """
    专门测试 AIMotor 进给电机通信
    """
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='测试AIMotor进给电机通信')
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
    
    # 测试 AIMotor 电机通信
    print("测试 AIMotor 进给电机通信...")
    
    # 设置控制模式为位置控制模式
    print("设置控制模式为位置控制模式...")
    controller.set_form(controller.AI_STATION, 0)
    time.sleep(0.5)
    
    # 使能 AIMotor 电机
    print("使能 AIMotor 电机...")
    controller.enable_motors(enable_pitch=False, enable_linear=True)
    time.sleep(0.5)
    
    # 设置相对位置
    print("\n设置位置到 -2000...")
    controller.set_linear_position(-2000, 200)
    time.sleep(5)
    
    # 设置回零点
    print("设置位置回 300...")
    controller.set_linear_position(300, 200)
    time.sleep(5)
    
    # 禁用 AIMotor 电机
    print("\n禁用 AIMotor 电机...")
    controller.enable_motors(enable_pitch=False, enable_linear=False)
    time.sleep(0.5)
    
    # 断开连接
    controller.disconnect()
    print("测试完成")

if __name__ == "__main__":
    main() 
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import argparse
import os

# 添加项目根目录到Python模块搜索路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# 导入src模块
from src.dcmotor import DCMotorController

def main():
    """
    测试与电机的通信，模仿SimpleNetwork的命令
    """
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='测试电机通信')
    parser.add_argument('--port', '-p', type=str, help='串口名称，例如COM3、COM4等。如果不指定，将自动扫描所有串口')
    parser.add_argument('--scan', '-s', action='store_true', help='扫描所有可用串口')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='串口波特率')
    args = parser.parse_args()
    
    # 创建电机控制器对象
    controller = DCMotorController(debug=True)
    
    # 如果指定了扫描选项，或者没有指定端口，则扫描所有可用串口
    if args.scan or not args.port:
        print("扫描可用串口...")
        available_ports = DCMotorController.scan_ports()
        
        if not available_ports:
            print("未找到可用串口")
            return
        
        print("找到以下串口:")
        for i, port in enumerate(available_ports):
            print(f"{i+1}. {port['device']} - {port['description']}")
        
        if args.port:
            # 如果指定了端口，检查它是否在可用端口列表中
            port_found = False
            for port_info in available_ports:
                if port_info['device'] == args.port:
                    port_found = True
                    break
            
            if not port_found:
                print(f"指定的端口 {args.port} 不在可用端口列表中")
                return
        else:
            # 如果没有指定端口，尝试连接每个可用端口
            connected = False
            for port_info in available_ports:
                port = port_info['device']
                print(f"\n尝试连接到 {port} - {port_info['description']}...")
                
                if controller.connect(port, args.baudrate):
                    print(f"成功连接到 {port}")
                    connected = True
                    break
                else:
                    print(f"无法连接到 {port}")
            
            if not connected:
                print("无法连接到任何串口")
                return
    else:
        # 使用指定的端口
        print(f"使用指定串口: {args.port}")
        
        # 连接到电机控制器
        print("尝试连接电机控制器...")
        if not controller.connect(args.port, args.baudrate):
            print(f"无法连接到电机控制器，请检查串口连接和名称: {args.port}")
            sys.exit(1)
        
        print("已连接到电机控制器")
    
    # 测试通信
    print("测试与电机的通信...")
    results = controller.test_communication()
    print(f"测试结果: {results}")
    
    if not (results['yf_pitch'] or results['ai_linear']):
        print("与电机的通信测试失败，退出测试")
        controller.disconnect()
        sys.exit(1)
    
    # 测试YF电机 (使用pos_pinch方法，完全按照SimpleNetwork示例)
    if results['yf_pitch']:
        print("\n测试YF电机命令...")
        
        # 使能YF电机
        print("使能YF电机...")
        controller.enable_motors(enable_pitch=True, enable_linear=False)
        time.sleep(1)
        
        # 使用pos_pinch方法设置位置 (模仿SimpleNetwork中case 11的调用方式)
        print("设置YF电机位置到 -25...")
        controller.pos_pinch(1, 30*100, -25*10000)  # station_num=1, maxSpeed=3000, angleControl=-250000
        time.sleep(5)
        
        print("设置YF电机位置回零点...")
        controller.pos_pinch(1, 30*100, 0)  # station_num=1, maxSpeed=3000, angleControl=0
        time.sleep(5)
        
        print("YF电机测试完成")
    
    # 测试AIMotor电机
    if results['ai_linear']:
        print("\n测试AIMotor电机命令...")
        
        # 先关闭AIMotor电机
        print("禁用AIMotor电机...")
        controller.enable_motors(enable_pitch=False, enable_linear=False)
        time.sleep(0.5)
        
        # 然后使能AIMotor电机
        print("使能AIMotor电机...")
        controller.enable_motors(enable_pitch=False, enable_linear=True)
        time.sleep(0.5)
        
        # 设置相对位置 (模仿SimpleNetwork中case 0的调用方式)
        print("设置AIMotor电机位置到 -2000...")
        controller.set_linear_position(-2000, 200)  # pos=-2000, speed=200
        time.sleep(3)
        
        # 设置回零点
        print("设置AIMotor电机位置回 300...")
        controller.set_linear_position(300, 200)  # pos=300, speed=200
        time.sleep(3)
        
        print("AIMotor电机测试完成")
    
    # 停止所有电机
    print("\n停止所有电机...")
    controller.stop_all()
    time.sleep(1)
    
    # 断开连接
    print("断开与电机控制器的连接")
    controller.disconnect()
    print("测试完成")

if __name__ == "__main__":
    main() 
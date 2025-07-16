#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
机械臂控制系统主程序
"""

import sys
import argparse
from PyQt5.QtWidgets import QApplication
from robot_gui import RobotArmGUI

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="机械臂控制系统")
    
    # 串口设置
    parser.add_argument("--servo-port", help="舵机串口，例如 'COM3'")
    parser.add_argument("--motor-port", help="电机串口，例如 'COM4'")
    parser.add_argument("--servo-baudrate", type=int, default=1000000, help="舵机波特率")
    parser.add_argument("--motor-baudrate", type=int, default=115200, help="电机波特率")
    
    # 协议设置
    parser.add_argument("--protocol-end", type=int, default=0, choices=[0, 1], 
                        help="舵机协议结束位 (STS/SMS=0, SCS=1)")
    
    # 调试设置
    parser.add_argument("--debug", action="store_true", help="启用调试输出")
    parser.add_argument("--test-motors", action="store_true", help="启动后立即测试电机")
    
    return parser.parse_args()

def main():
    """主函数"""
    # 解析命令行参数
    args = parse_args()
    
    # 创建应用
    app = QApplication(sys.argv)
    
    # 创建主窗口
    window = RobotArmGUI()
    
    # 设置调试模式
    if args.debug:
        print("Debug mode enabled")
        window.set_debug_mode(True)
    
    # 如果指定了端口，自动连接
    if args.servo_port and args.motor_port:
        window.auto_connect(args.servo_port, args.motor_port, 
                           args.servo_baudrate, args.motor_baudrate,
                           args.protocol_end)
        
        # 如果需要测试电机
        if args.test_motors:
            window.test_dc_motors()
    
    # 显示窗口
    window.show()
    
    # 运行应用
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 
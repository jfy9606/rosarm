#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
机械臂控制系统
图形用户界面程序
"""

import sys
import argparse
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication
from robot_gui import RobotArmGUI


def detect_serial_ports():
    """
    检测系统上可用的串行端口
    
    Returns:
        可用串行端口列表
    """
    ports = list(serial.tools.list_ports.comports())
    
    available_ports = []
    for p in ports:
        available_ports.append(p.device)
    
    return available_ports


def main():
    """
    程序主函数
    """
    parser = argparse.ArgumentParser(description='机械臂控制系统')
    parser.add_argument('--servo-port', dest='servo_port', help='舵机控制的串行端口')
    parser.add_argument('--motor-port', dest='motor_port', help='电机控制的串行端口')
    parser.add_argument('--servo-baudrate', dest='servo_baudrate', type=int, default=1000000,
                        help='舵机通信波特率')
    parser.add_argument('--motor-baudrate', dest='motor_baudrate', type=int, default=115200,
                        help='电机通信波特率')
    parser.add_argument('--protocol-end', dest='protocol_end', type=int, default=0,
                        help='舵机协议位结束（STS/SMS=0, SCS=1）')
    
    args = parser.parse_args()
    
    # 启动图形界面
    app = QApplication(sys.argv)
    window = RobotArmGUI()
    
    # 如果命令行指定了端口，就预先选择它们
    if args.servo_port and args.motor_port:
        # 将这些参数传递给GUI进行预选
        window.preselect_ports(args.servo_port, args.motor_port,
                              args.servo_baudrate, args.motor_baudrate,
                              args.protocol_end)
    
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 
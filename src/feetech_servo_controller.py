#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
使用 Feetech-Servo-SDK 的舵机控制器
"""

import os
import sys
import time
import platform

# 导入 Feetech-Servo-SDK
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'components', 'Feetech-Servo-SDK'))
from scservo_sdk import *

# SCServo 控制表地址
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_TIME         = 44
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56
ADDR_SCS_PRESENT_SPEED     = 58
ADDR_SCS_PRESENT_LOAD      = 60
ADDR_SCS_PRESENT_VOLTAGE   = 62
ADDR_SCS_PRESENT_TEMPERATURE = 63

class FeetechServoController:
    """
    使用 Feetech-Servo-SDK 控制 SCServo 舵机的类
    """
    
    def __init__(self, port=None, baudrate=1000000, protocol_end=0):
        """
        初始化 Feetech 舵机控制器
        
        Args:
            port: 串口名称，如果为 None 则需要稍后连接
            baudrate: 波特率，默认为 1000000
            protocol_end: 协议位结束（STS/SMS=0, SCS=1），默认为 0
        """
        self.port = port
        self.baudrate = baudrate
        self.protocol_end = protocol_end
        self.port_handler = None
        self.packet_handler = None
        
        if port:
            self.connect(port, baudrate, protocol_end)
    
    def connect(self, port, baudrate=1000000, protocol_end=0):
        """
        连接到舵机控制器
        
        Args:
            port: 串口名称
            baudrate: 波特率，默认为 1000000
            protocol_end: 协议位结束（STS/SMS=0, SCS=1），默认为 0
            
        Returns:
            bool: 如果连接成功则返回 True，否则返回 False
        """
        # 处理不同平台的串口命名规则
        if platform.system() == "Windows":
            if not port.startswith("COM"):
                port = f"COM{port}" if port.isdigit() else port
        else:  # Linux/Mac
            if not port.startswith("/dev/"):
                port = f"/dev/{port}" if not "/" in port else port
        
        # 初始化 PortHandler
        self.port_handler = PortHandler(port)
        
        # 初始化 PacketHandler
        self.packet_handler = PacketHandler(protocol_end)
        
        # 打开串口
        if not self.port_handler.openPort():
            print("Failed to open the port")
            return False
        
        # 设置波特率
        if not self.port_handler.setBaudRate(baudrate):
            print("Failed to change the baudrate")
            self.port_handler.closePort()
            return False
        
        print(f"Succeeded to open port {port} at baudrate {baudrate}")
        self.port = port
        self.baudrate = baudrate
        self.protocol_end = protocol_end
        return True
    
    def disconnect(self):
        """
        断开与舵机控制器的连接
        """
        if self.port_handler:
            self.port_handler.closePort()
            print("Port closed")
    
    def ping(self, servo_id):
        """
        Ping 舵机以检查是否响应
        
        Args:
            servo_id: 舵机 ID
            
        Returns:
            tuple: (model_number, result, error)
                model_number: 舵机型号号码
                result: 通信结果
                error: 错误代码
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return None, COMM_NOT_AVAILABLE, 0
        
        return self.packet_handler.ping(self.port_handler, servo_id)
    
    def read_position(self, servo_id):
        """
        读取舵机当前位置
        
        Args:
            servo_id: 舵机 ID
            
        Returns:
            int: 当前位置 (0-4095) 或如果出错则为 None
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return None
        
        # 设置一个更短的读取超时，防止在通信问题时长时间阻塞
        self.port_handler.setPacketTimeoutMillis(100)  # 100ms超时
        
        try:
            position, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_PRESENT_POSITION)
            
            if result != COMM_SUCCESS:
                print(f"Failed to read position: {self.packet_handler.getTxRxResult(result)}")
                return None
            
            if error != 0:
                print(f"Servo error: {self.packet_handler.getRxPacketError(error)}")
                return None
            
            return position
        except Exception as e:
            print(f"Exception during position read: {e}")
            return None
        finally:
            self.port_handler.setPacketTimeoutMillis(250)  # 设置回默认超时值
    
    def read_speed(self, servo_id):
        """
        读取舵机当前速度
        
        Args:
            servo_id: 舵机 ID
            
        Returns:
            int: 当前速度或如果出错则为 None
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return None
        
        speed, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, ADDR_SCS_PRESENT_SPEED)
        
        if result != COMM_SUCCESS:
            print(f"Failed to read speed: {self.packet_handler.getTxRxResult(result)}")
            return None
        
        if error != 0:
            print(f"Servo error: {self.packet_handler.getRxPacketError(error)}")
            return None
        
        # 转换为有符号值
        return SCS_TOHOST(speed, 15)  # 15位速度值（16位中的最高位为符号位）
    
    def read_load(self, servo_id):
        """
        读取舵机当前负载
        
        Args:
            servo_id: 舵机 ID
            
        Returns:
            int: 当前负载或如果出错则为 None
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return None
        
        load, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, ADDR_SCS_PRESENT_LOAD)
        
        if result != COMM_SUCCESS:
            print(f"Failed to read load: {self.packet_handler.getTxRxResult(result)}")
            return None
        
        if error != 0:
            print(f"Servo error: {self.packet_handler.getRxPacketError(error)}")
            return None
        
        # 转换为有符号值
        return SCS_TOHOST(load, 10)  # 10位负载值（11位中的最高位为符号位）
    
    def set_torque_enable(self, servo_id, enable):
        """
        使能或禁用舵机力矩
        
        Args:
            servo_id: 舵机 ID
            enable: True 表示使能，False 表示禁用
            
        Returns:
            bool: 如果成功则为 True，否则为 False
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return False
        
        value = 1 if enable else 0
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, servo_id, ADDR_SCS_TORQUE_ENABLE, value)
        
        if result != COMM_SUCCESS:
            print(f"Failed to set torque: {self.packet_handler.getTxRxResult(result)}")
            return False
        
        if error != 0:
            print(f"Servo error: {self.packet_handler.getRxPacketError(error)}")
            return False
        
        return True
    
    def write_position(self, servo_id, position, time=0):
        """
        设置舵机位置
        
        Args:
            servo_id: 舵机 ID
            position: 目标位置 (0-4095)
            time: 到达位置的时间（毫秒）(0 表示使用速度控制)
            
        Returns:
            bool: 如果成功则为 True，否则为 False
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return False
        
        # 限制位置在有效范围内
        position = max(0, min(4095, position))
        
        # 设置时间参数
        if time > 0:
            time = min(30000, time)  # 最大 30 秒
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_POSITION, position)
            if result == COMM_SUCCESS and error == 0:
                result, error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, servo_id, ADDR_SCS_GOAL_TIME, time)
        else:
            # 直接写入位置
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, ADDR_SCS_GOAL_POSITION, position)
        
        if result != COMM_SUCCESS:
            print(f"Failed to write position: {self.packet_handler.getTxRxResult(result)}")
            return False
        
        if error != 0:
            print(f"Servo error: {self.packet_handler.getRxPacketError(error)}")
            return False
        
        return True
    
    def write_speed(self, servo_id, speed):
        """
        设置舵机速度
        
        Args:
            servo_id: 舵机 ID
            speed: 速度值 (0-32767)
            
        Returns:
            bool: 如果成功则为 True，否则为 False
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return False
        
        # 限制速度在有效范围内
        speed = max(0, min(32767, speed))
        
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, ADDR_SCS_GOAL_SPEED, speed)
        
        if result != COMM_SUCCESS:
            print(f"Failed to write speed: {self.packet_handler.getTxRxResult(result)}")
            return False
        
        if error != 0:
            print(f"Servo error: {self.packet_handler.getRxPacketError(error)}")
            return False
        
        return True
    
    def write_acceleration(self, servo_id, acceleration):
        """
        设置舵机加速度
        
        Args:
            servo_id: 舵机 ID
            acceleration: 加速度值 (0-254)
            
        Returns:
            bool: 如果成功则为 True，否则为 False
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return False
        
        # 限制加速度在有效范围内
        acceleration = max(0, min(254, acceleration))
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, servo_id, ADDR_SCS_GOAL_ACC, acceleration)
        
        if result != COMM_SUCCESS:
            print(f"Failed to write acceleration: {self.packet_handler.getTxRxResult(result)}")
            return False
        
        if error != 0:
            print(f"Servo error: {self.packet_handler.getRxPacketError(error)}")
            return False
        
        return True
    
    def sync_write_position(self, positions, times=None):
        """
        同步写入多个舵机的位置
        
        Args:
            positions: 字典，格式为 {servo_id: position}
            times: 字典，格式为 {servo_id: time} 或单个时间值，用于所有舵机
            
        Returns:
            bool: 如果成功则为 True，否则为 False
        """
        if not self.port_handler or not self.packet_handler:
            print("Not connected to servo controller")
            return False
        
        if not positions:
            return False
        
        # 初始化 GroupSyncWrite 实例
        if times is not None and isinstance(times, dict) and len(times) > 0:
            # 位置和时间 - 需要4个字节的数据长度(位置2字节 + 时间2字节)
            groupSyncWrite = GroupSyncWrite(self.port_handler, 
                                          self.packet_handler,
                                          ADDR_SCS_GOAL_POSITION, 4)
            
            # 添加参数
            for servo_id, position in positions.items():
                # 限制位置在有效范围内
                position = max(0, min(4095, position))
                
                # 获取时间，如果不存在则默认为0
                time_value = 0
                if times:
                    if isinstance(times, dict):
                        time_value = times.get(servo_id, 0)
                    else:
                        time_value = times
                
                # 位置和时间参数
                param = [
                    SCS_LOBYTE(position), 
                    SCS_HIBYTE(position),
                    SCS_LOBYTE(time_value),
                    SCS_HIBYTE(time_value)
                ]
                
                result = groupSyncWrite.addParam(servo_id, param)
                if result != True:
                    print(f"Failed to add parameter for Servo ID: {servo_id}")
                    return False
                    
        else:
            # 只有位置 - 只需要2个字节的数据长度
            groupSyncWrite = GroupSyncWrite(self.port_handler, 
                                          self.packet_handler,
                                          ADDR_SCS_GOAL_POSITION, 2)
            
            # 添加参数
            for servo_id, position in positions.items():
                # 限制位置在有效范围内
                position = max(0, min(4095, position))
                
                # 只有位置参数
                param = [SCS_LOBYTE(position), SCS_HIBYTE(position)]
                
                result = groupSyncWrite.addParam(servo_id, param)
                if result != True:
                    print(f"Failed to add parameter for Servo ID: {servo_id}")
                    return False
        
        # 同步写入
        result = groupSyncWrite.txPacket()
        
        # 清除同步写入参数
        groupSyncWrite.clearParam()
        
        if result != COMM_SUCCESS:
            print(f"Failed to sync write: {self.packet_handler.getTxRxResult(result)}")
            return False
        
        return True 
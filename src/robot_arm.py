#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
机械臂控制类，集成 Feetech 舵机控制和 DC 电机控制
- 小臂电机：FT系列舵机，用于控制关节
- 大臂电机：两个不同型号的DC电机
  - AImotor：大臂的进给电机（线性移动）
  - YF：大臂的俯仰电机（俯仰运动）
"""

from .feetech_servo_controller import FeetechServoController
from .dcmotor import DCMotorController
import time
import math
import sys
import os

# 导入 Feetech-Servo-SDK 中的 COMM_SUCCESS
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'components', 'Feetech-Servo-SDK'))
from scservo_sdk import COMM_SUCCESS


class RobotArm:
    """
    机械臂控制类，集成 Feetech 舵机控制和 DC 电机控制
    - 小臂电机：FT系列舵机，用于控制关节
    - 大臂电机：
      - AImotor型号：大臂的进给电机（线性移动）
      - YF型号：大臂的俯仰电机（俯仰运动）
    
    支持仅连接小臂舵机或大臂电机的功能
    """
    
    # 默认关节 ID
    DEFAULT_JOINT_IDS = {
        'joint1': 1,  # 底座旋转
        'joint2': 2,  # 肩部
        'joint3': 3,  # 肘部
        'joint4': 4,  # 腕部俯仰
        'joint5': 5,  # 腕部旋转（当前设置中未使用）
        'joint6': 6   # 夹爪（当前设置中未使用）
    }
    
    # 关节限位（舵机位置单位：0-4095）
    DEFAULT_JOINT_LIMITS = {
        'joint1': (0, 4095),
        'joint2': (500, 3500),
        'joint3': (500, 3500),
        'joint4': (500, 3500),
        'joint5': (500, 3500),
        'joint6': (500, 3500)
    }
    
    # DC 电机限位
    DEFAULT_PITCH_LIMITS = (-10000, 10000)  # YF型号俯仰电机限位
    DEFAULT_LINEAR_LIMITS = (0, 20000)      # AImotor型号进给电机限位
    
    def __init__(self, servo_port=None, motor_port=None, joint_ids=None, joint_limits=None, protocol_end=0):
        """
        初始化机械臂控制器
        
        Args:
            servo_port: FT系列舵机控制串口
            motor_port: DC 电机控制串口 (AImotor进给电机和YF俯仰电机)
            joint_ids: 关节名称到舵机 ID 的映射字典
            joint_limits: 关节名称到位置限位的映射字典
            protocol_end: 舵机协议位结束（STS/SMS=0, SCS=1），默认为 0
        """
        # 初始化舵机控制器
        self.servo = FeetechServoController(port=servo_port, protocol_end=protocol_end) if servo_port else FeetechServoController(protocol_end=protocol_end)
        
        # 初始化 DC 电机控制器
        self.motor = DCMotorController(port=motor_port) if motor_port else DCMotorController()
        
        # 设置关节 ID 和限位
        self.joint_ids = joint_ids if joint_ids else self.DEFAULT_JOINT_IDS
        self.joint_limits = joint_limits if joint_limits else self.DEFAULT_JOINT_LIMITS
        
        # 设置 DC 电机限位
        self.pitch_limits = self.DEFAULT_PITCH_LIMITS  # YF型号俯仰电机限位
        self.linear_limits = self.DEFAULT_LINEAR_LIMITS  # AImotor型号进给电机限位
        
        # 当前关节位置
        self.current_positions = {}
        
        # 当前 DC 电机位置
        self.current_pitch = 0  # YF俯仰电机位置
        self.current_linear = 0  # AImotor进给电机位置
        
        # 运动速度
        self.servo_speed = 0  # 时间（毫秒，0 = 最大速度）
        self.pitch_speed = 100  # YF俯仰电机速度
        self.linear_speed = 100  # AImotor进给电机速度
        
        # 加速度参数 - 添加缺失的属性
        self.servo_acceleration = 5  # 默认加速度值
        
        # 连接状态
        self.servo_connected = False
        self.motor_connected = False
        self.connected = False
        
    def connect_servo(self, servo_port, servo_baudrate=1000000, protocol_end=0):
        """
        仅连接舵机控制器
        
        Args:
            servo_port: 舵机串口名称
            servo_baudrate: 舵机波特率
            protocol_end: 舵机协议结束位 (STS/SMS=0, SCS=1)
            
        Returns:
            bool: 如果连接成功则返回 True，否则返回 False
        """
        # 连接到舵机控制器
        servo_success = self.servo.connect(servo_port, servo_baudrate, protocol_end)
        
        if servo_success:
            self.servo_connected = True
            print(f"Connected to servo controller on {servo_port}")
            
            # 初始化舵机
            self._init_servos()
            
            # 至少有一个控制器连接成功，设置connected为True
            self.connected = True
            
            return True
        else:
            print(f"Failed to connect to servo controller on {servo_port}")
            self.servo_connected = False
            return False
    
    def connect_motor(self, motor_port, motor_baudrate=115200):
        """
        仅连接DC电机控制器
        
        Args:
            motor_port: 电机串口名称
            motor_baudrate: 电机波特率
            
        Returns:
            bool: 如果连接成功则返回 True，否则返回 False
        """
        # 连接到电机控制器
        motor_success = self.motor.connect(motor_port, motor_baudrate)
        
        if motor_success:
            self.motor_connected = True
            print(f"Connected to motor controller on {motor_port}")
            
            # 至少有一个控制器连接成功，设置connected为True
            self.connected = True
            
            return True
        else:
            print(f"Failed to connect to motor controller on {motor_port}")
            self.motor_connected = False
            return False
    
    def connect(self, servo_port=None, motor_port=None, servo_baudrate=1000000, motor_baudrate=115200, protocol_end=0):
        """
        连接到机械臂控制器
        
        Args:
            servo_port: 舵机串口名称，如果为None则不连接舵机
            motor_port: 电机串口名称，如果为None则不连接电机
            servo_baudrate: 舵机波特率
            motor_baudrate: 电机波特率
            protocol_end: 舵机协议结束位 (STS/SMS=0, SCS=1)
            
        Returns:
            bool: 如果至少有一个控制器连接成功则返回 True，否则返回 False
        """
        servo_success = False
        motor_success = False
        
        # 连接到舵机控制器（如果提供了端口）
        if servo_port:
            servo_success = self.connect_servo(servo_port, servo_baudrate, protocol_end)
        
        # 连接到电机控制器（如果提供了端口）
        if motor_port:
            motor_success = self.connect_motor(motor_port, motor_baudrate)
        
        # 如果至少有一个连接成功，则认为连接成功
        if servo_success or motor_success:
            self.connected = True
            return True
        else:
            self.connected = False
            return False
            
    def _init_servos(self):
        """
        初始化舵机，设置初始速度和加速度
        """
        if not self.servo_connected:
            print("Servo controller not connected")
            return False
            
        try:
            # 检查所有舵机ID是否存在
            for servo_id in self.joint_ids.values():
                if not self.servo.ping(servo_id):
                    print(f"Warning: Servo ID {servo_id} not found")
            
            # 设置所有舵机的初始速度和加速度
            for joint, servo_id in self.joint_ids.items():
                # 设置速度
                self.servo.write_speed(servo_id, self.servo_speed)
                
                # 设置加速度
                self.servo.write_acceleration(servo_id, self.servo_acceleration)
                
                print(f"Initialized servo for joint {joint} (ID: {servo_id})")
                
            print("All servos initialized successfully")
            return True
            
        except Exception as e:
            print(f"Error initializing servos: {e}")
            return False
    
    def disconnect(self):
        """
        断开与两个控制器的连接
        """
        if self.servo_connected:
            self.servo.disconnect()
            self.servo_connected = False
            
        if self.motor_connected:
            self.motor.disconnect()
            self.motor_connected = False
            
        self.connected = False
    
    def enable_torque(self, enable=True):
        """
        使能或禁用所有舵机的力矩
        
        Args:
            enable: True 表示使能，False 表示禁用
            
        Returns:
            bool: 如果所有舵机都成功则返回 True，否则返回 False
        """
        if not self.servo_connected:
            print("Servo controller not connected")
            return False
            
        success = True
        for joint, servo_id in self.joint_ids.items():
            if not self.servo.set_torque_enable(servo_id, enable):
                success = False
        
        return success
    
    def _clamp_joint_position(self, joint, position):
        """
        将关节位置限制在其限位范围内
        
        Args:
            joint: 关节名称
            position: 目标位置
            
        Returns:
            int: 限制后的位置值
        """
        if joint in self.joint_limits:
            min_pos, max_pos = self.joint_limits[joint]
            return max(min_pos, min(max_pos, position))
        return position
    
    def _clamp_motor_position(self, motor_type, position):
        """
        将 DC 电机位置限制在其限位范围内
        
        Args:
            motor_type: 'pitch' 或 'linear'
            position: 目标位置
            
        Returns:
            int: 限制后的位置值
        """
        if motor_type == 'pitch':
            min_pos, max_pos = self.pitch_limits
        else:  # linear
            min_pos, max_pos = self.linear_limits
            
        return max(min_pos, min(max_pos, position))
    
    def read_joint_positions(self):
        """
        读取所有舵机的当前位置
        
        Returns:
            dict: 关节位置字典，如果出错则返回 None
        """
        positions = {}
        for joint, servo_id in self.joint_ids.items():
            position = self.servo.read_position(servo_id)
            if position is not None:
                positions[joint] = position
        
        if positions:
            self.current_positions = positions
            return positions
        return None
    
    def read_motor_status(self):
        """
        读取DC电机状态
        
        Returns:
            dict: 电机状态字典，如果未连接则返回空字典
        """
        status = {}
        
        if not self.motor_connected:
            return status
            
        try:
            # 获取电机状态
            motor_status = self.motor.get_status()
            if motor_status:
                status.update(motor_status)
            
            # 获取位置
            pitch_pos, linear_pos = self.motor.get_motor_positions()
            if pitch_pos is not None:
                status['pitch_position'] = pitch_pos
                self.current_pitch = pitch_pos
                
            if linear_pos is not None:
                status['linear_position'] = linear_pos
                self.current_linear = linear_pos
                
            return status
        except Exception as e:
            print(f"Error reading motor status: {e}")
            return {}
    
    def set_joint_position(self, joint, position, blocking=False, timeout=5.0):
        """
        设置单个关节位置
        
        Args:
            joint: 关节名称或ID
            position: 目标位置
            blocking: 是否阻塞等待完成
            timeout: 最大等待时间（秒）
            
        Returns:
            bool: 如果命令发送成功则返回 True
        """
        if not self.servo_connected:
            print("Servo controller not connected")
            return False
        
        # 检查是否是已知关节
        servo_id = None
        if isinstance(joint, str):
            if joint in self.joint_ids:
                servo_id = self.joint_ids[joint]
            else:
                print(f"Unknown joint: {joint}")
                return False
        else:
            # 假设joint是ID
            servo_id = joint
        
        # 限制位置范围
        if isinstance(joint, str) and joint in self.joint_limits:
            position = self._clamp_joint_position(joint, position)
        
        # 设置位置
        if not self.servo.write_position(servo_id, position):
            print(f"Failed to set position for joint {joint}")
            return False
        
        # 更新当前位置
        if isinstance(joint, str):
            self.current_positions[joint] = position
        
        # 如果需要阻塞等待
        if blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 读取当前位置
                current_pos = self.servo.read_position(servo_id)
                if current_pos is None:
                    print(f"Failed to read position for joint {joint}")
                    return False
                
                # 检查是否到达目标位置
                if abs(current_pos - position) <= 10:  # 允许10个单位的误差
                    return True
                
                time.sleep(0.1)
        
            print(f"Timeout waiting for joint {joint} to reach position {position}")
            return False
            
        return True 

    def check_servo_voltage(self, servo_id):
        """
        检查舵机电压是否在正常范围内
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            tuple: (is_ok, voltage)
                is_ok: 如果电压正常则为True
                voltage: 电压值(单位:V)
        """
        if not self.servo_connected or not self.servo or not self.servo.port_handler:
            return False, 0
        
        try:
            # 读取电压寄存器(地址62)
            voltage_raw, result, error = self.servo.packet_handler.read1ByteTxRx(
                self.servo.port_handler, servo_id, 62)  # ADDR_SCS_PRESENT_VOLTAGE
            
            if result != COMM_SUCCESS or error != 0:
                return False, 0
            
            # 转换为实际电压(单位:V)
            voltage = voltage_raw / 10.0
            
            # 检查电压是否在安全范围内(通常6V-12V，但这里我们宽松一些)
            is_ok = (voltage >= 5.5) and (voltage <= 12.5)
            
            return is_ok, voltage
            
        except Exception as e:
            print(f"检查电压时出错: {e}")
            return False, 0 
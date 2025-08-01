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
import numpy as np
import cv2  # 添加OpenCV库用于图像处理

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
        
        # 机械臂DH参数（用于运动学计算）
        self.dh_params = {
            # 默认值，实际使用时应根据机械臂尺寸调整
            'a': [0, 0.25, 0.25, 0],  # 连杆长度(m)
            'd': [0.1, 0, 0, 0],      # 连杆偏移(m)
            'alpha': [np.pi/2, 0, 0, -np.pi/2]  # 连杆扭转角度(rad)
        }
        
        # 笛卡尔坐标系中末端执行器的位置和姿态
        self.end_effector_pose = {
            'position': np.array([0.0, 0.0, 0.0]),  # [x, y, z] in meters
            'orientation': np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] in radians
        }
        
        # 连接的舵机IDs列表
        self.connected_servos = set()
        
        # 未连接舵机的默认位置值
        self.default_servo_positions = {}
        
        # 添加坐标系转换矩阵
        # 大臂电机坐标系到机械臂基座坐标系的转换矩阵
        self.T_base_to_dcmotor = np.eye(4)
        # 默认设置大臂电机位于基座正上方10cm处
        self.T_base_to_dcmotor[2, 3] = 0.10
        
        # 双目相机状态
        self.cameras_initialized = False
        self.left_camera = None
        self.right_camera = None
    
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

    def detect_connected_servos(self):
        """
        检测哪些舵机实际连接并可通信
        
        Returns:
            set: 已连接舵机ID的集合
        """
        if not self.servo_connected:
            print("舵机控制器未连接")
            return set()
            
        connected = set()
        for joint, servo_id in self.joint_ids.items():
            try:
                # 尝试ping舵机
                model_number, comm_result, error = self.servo.ping(servo_id)
                if model_number > 0 and comm_result == 0:  # 通信成功且返回了型号
                    connected.add(servo_id)
                    print(f"舵机 {joint} (ID: {servo_id}) 已连接")
                else:
                    print(f"舵机 {joint} (ID: {servo_id}) 未连接或通信错误")
            except Exception as e:
                print(f"检测舵机 {joint} (ID: {servo_id}) 时发生错误: {e}")
                
        self.connected_servos = connected
        return connected
    
    def calibrate_servos(self):
        """
        检测未连接的舵机并设置校准参数
        
        Returns:
            dict: 校准参数
        """
        # 首先检测已连接舵机
        self.detect_connected_servos()
        
        # 为未连接舵机设置默认位置
        for joint, servo_id in self.joint_ids.items():
            if servo_id not in self.connected_servos:
                # 使用中点位置作为默认值
                if joint in self.joint_limits:
                    min_pos, max_pos = self.joint_limits[joint]
                    default_pos = (min_pos + max_pos) // 2
                    self.default_servo_positions[servo_id] = default_pos
                    print(f"舵机 {joint} (ID: {servo_id}) 未连接，使用默认位置: {default_pos}")
                    
        return self.default_servo_positions
    
    def dh_transform(self, a, d, alpha, theta):
        """
        计算单个DH变换矩阵
        
        Args:
            a: 连杆长度
            d: 连杆偏移
            alpha: 连杆扭转角度
            theta: 关节角度
            
        Returns:
            numpy.ndarray: 4x4变换矩阵
        """
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        T = np.array([
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    def forward_kinematics(self, joint_angles=None):
        """
        计算正向运动学，获取机械臂末端执行器的位置和姿态
        
        Args:
            joint_angles: 关节角度列表(弧度)，如果为None则使用当前关节角度
            
        Returns:
            dict: 末端执行器的位置和姿态
                'position': [x, y, z] 位置(m)
                'orientation': [roll, pitch, yaw] 姿态(rad)
                'matrix': 4x4变换矩阵
        """
        # 如果未提供关节角度，则读取当前关节角度
        if joint_angles is None:
            joint_angles = self.get_current_joint_angles()
            
        if joint_angles is None or len(joint_angles) < 4:
            print("缺少关节角度信息")
            return None
        
        # 获取DH参数
        a = self.dh_params['a']
        d = self.dh_params['d']
        alpha = self.dh_params['alpha']
        
        # 计算总变换矩阵
        T = np.eye(4)
        for i in range(len(joint_angles)):
            Ti = self.dh_transform(a[i], d[i], alpha[i], joint_angles[i])
            T = np.matmul(T, Ti)
        
        # 提取位置和姿态
        position = T[0:3, 3]
        
        # 从旋转矩阵提取欧拉角(roll, pitch, yaw)
        sy = np.sqrt(T[0, 0] * T[0, 0] + T[1, 0] * T[1, 0])
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(T[2, 1], T[2, 2])
            pitch = np.arctan2(-T[2, 0], sy)
            yaw = np.arctan2(T[1, 0], T[0, 0])
        else:
            roll = np.arctan2(-T[1, 2], T[1, 1])
            pitch = np.arctan2(-T[2, 0], sy)
            yaw = 0
        
        orientation = np.array([roll, pitch, yaw])
        
        # 更新末端执行器位姿
        self.end_effector_pose = {
            'position': position,
            'orientation': orientation,
            'matrix': T
        }
        
        return self.end_effector_pose
    
    def get_current_joint_angles(self):
        """
        获取当前关节角度(弧度)
        
        Returns:
            list: 关节角度列表(弧度)
        """
        # 读取当前舵机位置
        positions = self.read_joint_positions()
        if not positions:
            print("无法读取关节位置")
            return None
        
        # 将舵机位置(0-4095)转换为关节角度(弧度)
        angles = []
        for joint in ['joint1', 'joint2', 'joint3', 'joint4']:
            if joint in positions:
                # 舵机位置到角度的转换
                pos = positions[joint]
                min_pos, max_pos = self.joint_limits[joint]
                angle = (pos - min_pos) / (max_pos - min_pos) * np.pi - np.pi/2
                angles.append(angle)
            elif joint in self.joint_ids:
                # 使用未连接舵机的默认位置
                servo_id = self.joint_ids[joint]
                if servo_id in self.default_servo_positions:
                    pos = self.default_servo_positions[servo_id]
                    min_pos, max_pos = self.joint_limits[joint]
                    angle = (pos - min_pos) / (max_pos - min_pos) * np.pi - np.pi/2
                    angles.append(angle)
                else:
                    # 没有默认位置，使用中点
                    min_pos, max_pos = self.joint_limits[joint]
                    pos = (min_pos + max_pos) // 2
                    angle = (pos - min_pos) / (max_pos - min_pos) * np.pi - np.pi/2
                    angles.append(angle)
        
        return angles
    
    def inverse_kinematics(self, target_position, target_orientation=None, max_iterations=200, tolerance=0.001, adaptive_tolerance=True):
        """
        计算逆向运动学，求解到达指定位置和姿态所需的关节角度
        采用迭代法（雅可比矩阵）
        
        Args:
            target_position: 目标位置 [x, y, z]
            target_orientation: 目标姿态 [roll, pitch, yaw]，如果为None则仅考虑位置
            max_iterations: 最大迭代次数
            tolerance: 位置误差容忍度(m)
            adaptive_tolerance: 是否使用自适应容忍度（当迭代未收敛但误差较小时仍返回近似解）
            
        Returns:
            tuple: (是否成功, 关节角度列表)
        """
        # 获取当前关节角度作为初始猜测值
        joint_angles = self.get_current_joint_angles()
        if not joint_angles:
            print("无法获取当前关节角度作为初始值")
            return False, None
        
        # 转换为numpy数组
        joint_angles = np.array(joint_angles)
        target_position = np.array(target_position)
        
        # 创建目标位姿矩阵
        target_pose = np.eye(4)
        target_pose[0:3, 3] = target_position
        
        if target_orientation is not None:
            # 如果提供了姿态，则设置旋转矩阵部分
            roll, pitch, yaw = target_orientation
            
            # 计算旋转矩阵
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)]
            ])
            
            Ry = np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]
            ])
            
            Rz = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1]
            ])
            
            R = np.matmul(np.matmul(Rz, Ry), Rx)
            target_pose[0:3, 0:3] = R
        
        # 迭代求解
        for iteration in range(max_iterations):
            # 计算当前位姿
            current_pose = self.forward_kinematics(joint_angles)
            if current_pose is None:
                return False, None
            
            current_position = current_pose['position']
            
            # 计算位置误差
            position_error = target_position - current_position
            error_magnitude = np.linalg.norm(position_error)
            
            # 如果误差小于容忍度，则认为找到解
            if error_magnitude < tolerance:
                # 将关节角度转换回舵机位置值
                servo_positions = {}
                for i, joint in enumerate(['joint1', 'joint2', 'joint3', 'joint4']):
                    if i < len(joint_angles):
                        angle = joint_angles[i]
                        min_pos, max_pos = self.joint_limits[joint]
                        # 将角度(-pi/2到pi/2)转换为舵机位置(min_pos到max_pos)
                        pos = int((angle + np.pi/2) / np.pi * (max_pos - min_pos) + min_pos)
                        servo_positions[joint] = pos
                
                return True, servo_positions
            
            # 计算雅可比矩阵（数值微分法）
            J = np.zeros((3, len(joint_angles)))
            delta = 0.01  # 微小变化量
            
            for i in range(len(joint_angles)):
                # 增加一个微小变化
                joint_angles_plus = joint_angles.copy()
                joint_angles_plus[i] += delta
                
                # 计算新位置
                pose_plus = self.forward_kinematics(joint_angles_plus)
                if pose_plus is None:
                    continue
                
                position_plus = pose_plus['position']
                
                # 计算雅可比矩阵的一列
                J[:, i] = (position_plus - current_position) / delta
            
            # 计算关节角度的更新量（使用雅可比矩阵的伪逆）
            try:
                J_inv = np.linalg.pinv(J)
                delta_theta = np.matmul(J_inv, position_error)
                
                # 更新关节角度
                joint_angles += delta_theta * 0.5  # 添加阻尼因子以提高稳定性
                
                # 限制关节角度在有效范围内
                for i in range(len(joint_angles)):
                    joint_angles[i] = max(-np.pi/2, min(np.pi/2, joint_angles[i]))
                    
            except np.linalg.LinAlgError:
                print("雅可比矩阵求逆失败")
                return False, None
        
        print(f"逆运动学迭代未收敛，最终误差: {error_magnitude:.4f}m")
        
        # 如果启用了自适应容忍度，且误差在可接受范围内（小于0.2m），仍然返回近似解
        if adaptive_tolerance and error_magnitude < 0.2:
            print(f"使用自适应容忍度，返回近似解（误差: {error_magnitude:.4f}m）")
            # 将关节角度转换回舵机位置值
            servo_positions = {}
            for i, joint in enumerate(['joint1', 'joint2', 'joint3', 'joint4']):
                if i < len(joint_angles):
                    angle = joint_angles[i]
                    min_pos, max_pos = self.joint_limits[joint]
                    # 将角度(-pi/2到pi/2)转换为舵机位置(min_pos到max_pos)
                    pos = int((angle + np.pi/2) / np.pi * (max_pos - min_pos) + min_pos)
                    servo_positions[joint] = pos
            
            return True, servo_positions
        
        return False, None
    
    def is_moving(self, servo_id):
        """
        检查舵机是否正在移动
        
        Args:
            servo_id: 舵机 ID
            
        Returns:
            bool: 如果舵机正在移动则返回 True，否则返回 False
        """
        if not self.servo_connected:
            return False
        
        # 读取舵机当前速度
        speed = self.servo.read_speed(servo_id)
        
        # 如果读取失败，假设舵机不在移动
        if speed is None:
            return False
        
        # 如果速度不为0，认为舵机正在移动
        return abs(speed) > 0
    
    def move_to_cartesian_position(self, x, y, z, roll=None, pitch=None, yaw=None, blocking=False, timeout=10.0, strict_error_check=False):
        """
        移动末端执行器到指定的笛卡尔坐标位置和姿态
        
        Args:
            x, y, z: 目标位置坐标(m)
            roll, pitch, yaw: 目标姿态欧拉角(rad)，如果为None则仅控制位置
            blocking: 是否阻塞等待动作完成
            timeout: 最大等待时间(秒)
            strict_error_check: 是否严格检查误差（如果为False，即使存在误差也会继续执行）
            
        Returns:
            bool: 如果规划成功并开始执行则返回True
        """
        if not self.connected:
            print("控制器未连接")
            return False
        
        # 创建目标位置数组
        target_position = np.array([x, y, z])
        
        # 创建目标姿态数组(如果有提供)
        target_orientation = None
        if roll is not None and pitch is not None and yaw is not None:
            target_orientation = np.array([roll, pitch, yaw])
        
        # 计算逆运动学（启用自适应容忍度）
        success, joint_positions = self.inverse_kinematics(target_position, target_orientation, adaptive_tolerance=True)
        
        if not success:
            print("无法求解逆运动学，无法到达目标位置")
            return False
        
        # 设置关节位置
        for joint, position in joint_positions.items():
            if joint in self.joint_ids:
                servo_id = self.joint_ids[joint]
                if not self.set_joint_position(joint, position):
                    print(f"设置关节 {joint} 位置失败")
                    return False
        
        # 如果需要阻塞等待
        if blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 计算当前末端位置
                current_pose = self.forward_kinematics()
                if current_pose is None:
                    time.sleep(0.1)
                    continue
                
                current_position = current_pose['position']
                
                # 计算位置误差
                error = np.linalg.norm(target_position - current_position)
                
                # 如果误差小于阈值，认为到达目标
                if error < 0.01:  # 1cm误差
                    return True
                
                # 检查所有舵机是否已停止移动
                all_servos_stopped = True
                for joint in self.joint_ids:
                    if self.is_moving(self.joint_ids[joint]):
                        all_servos_stopped = False
                        break
                
                # 如果所有舵机已停止移动且不需要严格检查误差，则认为动作已完成
                if all_servos_stopped and not strict_error_check:
                    print(f"机械臂已停止移动，最终误差: {error:.4f}m")
                    return True
                
                time.sleep(0.1)
            
            print("等待机械臂到达目标位置超时")
            # 如果不需要严格检查误差，即使超时也返回成功
            if not strict_error_check:
                return True
            return False
        
        return True
    
    def get_current_cartesian_position(self):
        """
        获取当前末端执行器的笛卡尔坐标位置和姿态
        
        Returns:
            dict: 包含位置和姿态的字典，如果计算失败则返回None
        """
        return self.forward_kinematics() 

    def set_motor_position(self, motor_type, position, speed=None, blocking=False, timeout=5.0):
        """
        设置DC电机位置
        
        Args:
            motor_type: 'pitch' 或 'linear'
            position: 目标位置
            speed: 速度，如果为None则使用默认速度
            blocking: 是否阻塞等待完成
            timeout: 最大等待时间（秒）
            
        Returns:
            bool: 如果命令发送成功则返回 True
        """
        if not self.motor_connected:
            print("电机控制器未连接")
            return False
        
        # 限制位置范围
        position = self._clamp_motor_position(motor_type, position)
        
        try:
            if motor_type == 'pitch':
                # 设置YF俯仰电机位置
                speed = speed if speed is not None else self.pitch_speed
                if not self.motor.set_pitch_position(position, speed):
                    print(f"设置俯仰电机位置失败: {position}")
                    return False
                self.current_pitch = position
            elif motor_type == 'linear':
                # 设置AImotor进给电机位置
                speed = speed if speed is not None else self.linear_speed
                if not self.motor.set_linear_position(position, speed):
                    print(f"设置进给电机位置失败: {position}")
                    return False
                self.current_linear = position
            else:
                print(f"未知电机类型: {motor_type}")
                return False
            
            # 如果需要阻塞等待
            if blocking:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    # 读取电机状态
                    status = self.read_motor_status()
                    if not status:
                        time.sleep(0.1)
                        continue
                    
                    # 获取当前位置
                    if motor_type == 'pitch' and 'pitch_position' in status:
                        current_pos = status['pitch_position']
                    elif motor_type == 'linear' and 'linear_position' in status:
                        current_pos = status['linear_position']
                    else:
                        time.sleep(0.1)
                        continue
                    
                    # 检查是否到达目标位置
                    if abs(current_pos - position) <= 10:  # 允许10个单位的误差
                        return True
                    
                    time.sleep(0.1)
                    
                print(f"等待电机 {motor_type} 到达位置 {position} 超时")
                return False
                
            return True
            
        except Exception as e:
            print(f"设置电机 {motor_type} 位置时出错: {e}")
            return False
    
    def move_to_cartesian_with_motors(self, x, y, z, pitch_pos=None, linear_pos=None, roll=None, pitch=None, yaw=None, blocking=False, timeout=10.0):
        """
        综合控制大臂电机和小臂舵机，移动末端执行器到指定的笛卡尔坐标位置
        
        Args:
            x, y, z: 目标位置坐标(m)
            pitch_pos: 大臂俯仰电机位置，如果为None则保持当前位置
            linear_pos: 大臂进给电机位置，如果为None则保持当前位置
            roll, pitch, yaw: 目标姿态欧拉角(rad)，如果为None则仅控制位置
            blocking: 是否阻塞等待动作完成
            timeout: 最大等待时间(秒)
            
        Returns:
            bool: 如果规划成功并开始执行则返回True
        """
        # 先设置大臂电机位置（如果提供）
        motors_success = True
        if self.motor_connected:
            # 设置大臂俯仰电机位置
            if pitch_pos is not None:
                motor_success = self.set_motor_position('pitch', pitch_pos, blocking=False)
                motors_success = motors_success and motor_success
            
            # 设置大臂进给电机位置
            if linear_pos is not None:
                motor_success = self.set_motor_position('linear', linear_pos, blocking=False)
                motors_success = motors_success and motor_success
        
        # 然后设置舵机位置以到达笛卡尔目标
        servos_success = True
        if self.servo_connected:
            # 笛卡尔坐标考虑大臂电机的位置
            # 计算大臂电机坐标系中的目标位置
            target_in_base = np.array([x, y, z, 1])
            
            # 将目标转换为小臂舵机坐标系中的目标
            # 注意：这里我们假设小臂舵机的坐标系原点在大臂电机位置
            servos_success = self.move_to_cartesian_position(x, y, z, roll, pitch, yaw, blocking=False, strict_error_check=False)
        
        # 如果需要阻塞等待
        if blocking and (motors_success or servos_success):
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 检查大臂电机是否到位
                motors_done = True
                if self.motor_connected:
                    status = self.read_motor_status()
                    if not status:
                        motors_done = False
                    else:
                        if pitch_pos is not None and 'pitch_position' in status:
                            if abs(status['pitch_position'] - pitch_pos) > 10:
                                motors_done = False
                        
                        if linear_pos is not None and 'linear_position' in status:
                            if abs(status['linear_position'] - linear_pos) > 10:
                                motors_done = False
                
                # 检查小臂舵机是否到位
                servos_done = True
                if self.servo_connected:
                    # 获取当前末端执行器位置
                    current_pose = self.get_current_cartesian_position()
                    if current_pose is None:
                        servos_done = False
                    else:
                        current_pos = current_pose['position']
                        # 计算位置误差
                        error = np.linalg.norm(np.array([x, y, z]) - current_pos)
                        if error > 0.01:  # 1cm误差
                            servos_done = False
                
                if motors_done and servos_done:
                    return True
                
                time.sleep(0.1)
            
            print("等待机械臂到达目标位置超时")
            return False
        
        return motors_success or servos_success
        
    def coordinate_to_motors(self, x, y, z):
        """
        将笛卡尔坐标转换为大臂电机的位置值
        
        Args:
            x, y, z: 目标位置坐标(m)
            
        Returns:
            tuple: (pitch_pos, linear_pos) 大臂俯仰电机和进给电机的位置值
        """
        # 简化模型：y 主要由底座旋转舵机控制
        # x, z 则由大臂电机和小臂舵机共同控制
        
        # 计算到原点的距离（在xz平面）
        distance_xz = np.sqrt(x**2 + z**2)
        
        # 简单映射: 距离越大，进给电机位置越大
        # 这里假设进给电机位置与距离成正比
        linear_scale = 5000  # 比例系数，需要根据实际机械臂调整
        linear_pos = int(distance_xz * linear_scale)
        
        # 限制在有效范围内
        linear_pos = max(self.linear_limits[0], min(self.linear_limits[1], linear_pos))
        
        # 计算俯仰角度（相对于z轴）
        if z != 0:
            pitch_angle = np.arctan2(x, z)  # 弧度
        else:
            pitch_angle = np.pi/2 if x > 0 else -np.pi/2
            
        # 将角度转换为俯仰电机位置
        # 这里假设俯仰电机位置与角度成正比
        pitch_scale = 1000  # 比例系数，需要根据实际机械臂调整
        pitch_pos = int(pitch_angle * pitch_scale)
        
        # 限制在有效范围内
        pitch_pos = max(self.pitch_limits[0], min(self.pitch_limits[1], pitch_pos))
        
        return pitch_pos, linear_pos
        
    #--------------------------------------------------------------------------
    # 视觉系统集成部分
    #--------------------------------------------------------------------------
    
    def init_stereo_cameras(self, left_camera_id=0, right_camera_id=1):
        """
        初始化双目相机
        
        Args:
            left_camera_id: 左相机ID
            right_camera_id: 右相机ID
            
        Returns:
            bool: 如果成功初始化双目相机则返回True
        """
        try:
            # 检查摄像头索引是否有效
            if left_camera_id < 0 or right_camera_id < 0:
                print(f"无效的摄像头索引: 左 {left_camera_id}, 右 {right_camera_id}")
                return False
                
            # 初始化左右相机
            print(f"尝试初始化左相机 (索引: {left_camera_id})...")
            self.left_camera = cv2.VideoCapture(left_camera_id)
            print(f"尝试初始化右相机 (索引: {right_camera_id})...")
            self.right_camera = cv2.VideoCapture(right_camera_id)
            
            # 检查相机是否成功打开
            if not self.left_camera.isOpened() or not self.right_camera.isOpened():
                print(f"无法打开双目相机: 左相机状态: {self.left_camera.isOpened()}, 右相机状态: {self.right_camera.isOpened()}")
                return False
                
            # 设置相机分辨率
            self.left_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.left_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.right_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.right_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # 相机标定参数（这些参数需要通过相机标定获得）
            # 实际使用时请替换为实际标定得到的参数
            self.camera_matrix_left = np.array([
                [500, 0, 320],
                [0, 500, 240],
                [0, 0, 1]
            ])
            self.camera_matrix_right = np.array([
                [500, 0, 320],
                [0, 500, 240],
                [0, 0, 1]
            ])
            self.dist_coeffs_left = np.zeros((5, 1))
            self.dist_coeffs_right = np.zeros((5, 1))
            
            # 双目相机参数
            self.baseline = 0.1  # 基线距离(m)
            self.focal_length = 500  # 焦距(像素)
            
            # 标记相机已初始化
            self.cameras_initialized = True
            
            print("双目相机初始化成功")
            return True
            
        except Exception as e:
            print(f"初始化双目相机时发生错误: {e}")
            self.cameras_initialized = False
            return False
    
    def capture_stereo_images(self):
        """
        同时捕获左右相机的图像
        
        Returns:
            tuple: (left_frame, right_frame) 或 (None, None)表示失败
        """
        if not hasattr(self, 'cameras_initialized') or not self.cameras_initialized:
            print("双目相机未初始化")
            return None, None
            
        try:
            # 捕获左右相机图像
            ret_left, left_frame = self.left_camera.read()
            ret_right, right_frame = self.right_camera.read()
            
            if not ret_left or not ret_right:
                print("无法捕获双目相机图像")
                return None, None
                
            return left_frame, right_frame
            
        except Exception as e:
            print(f"捕获双目图像时发生错误: {e}")
            return None, None
    
    def detect_object(self, frame, object_name="target"):
        """
        在图像中检测指定物体
        
        Args:
            frame: 输入图像帧
            object_name: 目标物体名称
            
        Returns:
            tuple: (x, y, w, h) 物体在图像中的边界框，或 None表示未检测到
        """
        try:
            # 这里使用简单的颜色检测作为示例
            # 实际应用中应替换为更复杂的物体检测算法(如YOLO, SSD等)
            
            # 转换为HSV颜色空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # 根据物体名称选择颜色阈值
            # 支持颜色名称和序号
            if object_name.lower() == "red" or object_name == "1":
                # 红色物体的HSV阈值
                lower_color = np.array([0, 100, 100])
                upper_color = np.array([10, 255, 255])
            elif object_name.lower() == "blue" or object_name == "2":
                # 蓝色物体的HSV阈值
                lower_color = np.array([100, 100, 100])
                upper_color = np.array([140, 255, 255])
            elif object_name.lower() == "green" or object_name == "3":
                # 绿色物体的HSV阈值
                lower_color = np.array([40, 100, 100])
                upper_color = np.array([80, 255, 255])
            elif object_name == "4":
                # 黄色物体的HSV阈值
                lower_color = np.array([20, 100, 100])
                upper_color = np.array([30, 255, 255])
            elif object_name == "5":
                # 紫色物体的HSV阈值
                lower_color = np.array([125, 100, 100])
                upper_color = np.array([155, 255, 255])
            elif object_name == "6":
                # 青色物体的HSV阈值
                lower_color = np.array([85, 100, 100])
                upper_color = np.array([95, 255, 255])
            else:
                # 默认为红色
                lower_color = np.array([0, 100, 100])
                upper_color = np.array([10, 255, 255])
            
            # 创建掩码
            mask = cv2.inRange(hsv, lower_color, upper_color)
            
            # 形态学操作去除噪点
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # 寻找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 选择最大的轮廓
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                # 如果面积太小，认为是噪声
                if area < 100:
                    return None
                    
                # 获取边界框
                x, y, w, h = cv2.boundingRect(largest_contour)
                return (x, y, w, h)
            
            return None
            
        except Exception as e:
            print(f"物体检测错误: {e}")
            return None
    
    def calculate_distance_with_yolo(self, left_frame, right_frame, yolo_bbox):
        """
        使用YOLO检测框和双目立体视觉计算物体到相机的距离
        
        Args:
            left_frame: 左相机图像
            right_frame: 右相机图像
            yolo_bbox: YOLO检测框 [x1, y1, x2, y2]
            
        Returns:
            tuple: (x, y, z, bbox) 物体的3D坐标(m)和边界框，或 None表示失败
        """
        try:
            # 从YOLO检测框中提取左图像中的边界框
            x1, y1, x2, y2 = map(int, yolo_bbox)
            left_bbox = (x1, y1, x2-x1, y2-y1)  # 转换为(x, y, w, h)格式
            
            # 在右图像中寻找对应的物体
            # 这里使用简化方法：在右图像中搜索与左图像中相似的区域
            # 实际应用中应使用更复杂的立体匹配算法
            
            # 获取左图像中物体的中心点
            left_x = (x1 + x2) // 2
            left_y = (y1 + y2) // 2
            
            # 在右图像中搜索范围（假设视差不会太大）
            search_range = 100  # 搜索范围（像素）
            search_x1 = max(0, left_x - search_range)
            search_x2 = min(right_frame.shape[1], left_x + search_range)
            
            # 提取左图像中的物体区域
            object_roi = left_frame[y1:y2, x1:x2]
            if object_roi.size == 0:
                print("YOLO检测框无效")
                return None
                
            # 在右图像中搜索最匹配的位置
            best_match_x = None
            best_match_score = float('inf')
            
            # 简化的模板匹配（实际应用中可使用更复杂的方法）
            for x in range(search_x1, search_x2 - (x2-x1)):
                roi = right_frame[y1:y2, x:x+(x2-x1)]
                if roi.shape != object_roi.shape:
                    continue
                    
                # 计算差异（简单的像素差的平方和）
                diff = np.sum((roi.astype(np.float32) - object_roi.astype(np.float32)) ** 2)
                if diff < best_match_score:
                    best_match_score = diff
                    best_match_x = x
            
            if best_match_x is None:
                print("在右图像中未找到匹配区域")
                return None
                
            # 计算右图像中物体的中心点
            right_x = best_match_x + (x2-x1) // 2
            right_y = left_y  # 假设y坐标相同
            
            # 计算视差
            disparity = left_x - right_x
            
            # 避免除零错误
            if disparity <= 0:
                print("视差为零或负值，无法计算距离")
                return None
                
            # 计算距离(m)
            z = (self.baseline * self.focal_length) / disparity
            
            # 计算3D坐标
            x = (left_x - 320) * z / self.focal_length
            y = (left_y - 240) * z / self.focal_length
            
            print(f"使用YOLO检测框计算的物体位置: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
            return (x, y, z, left_bbox)
            
        except Exception as e:
            print(f"YOLO距离计算错误: {e}")
            import traceback
            print(traceback.format_exc())
            return None
    
    def calculate_distance(self, left_frame, right_frame, object_name="target"):
        """
        使用双目立体视觉计算物体到相机的距离
        
        Args:
            left_frame: 左相机图像
            right_frame: 右相机图像
            object_name: 要检测的物体名称
            
        Returns:
            tuple: (x, y, z, bbox) 物体的3D坐标(m)和边界框，或 None表示失败
        """
        try:
            # 在左右图像中检测物体
            left_bbox = self.detect_object(left_frame, object_name)
            right_bbox = self.detect_object(right_frame, object_name)
            
            if left_bbox is None or right_bbox is None:
                print(f"在双目图像中未检测到物体: {object_name}")
                return None
                
            # 获取物体中心点
            left_x = left_bbox[0] + left_bbox[2] // 2
            left_y = left_bbox[1] + left_bbox[3] // 2
            right_x = right_bbox[0] + right_bbox[2] // 2
            right_y = right_bbox[1] + right_bbox[3] // 2
            
            # 计算视差
            disparity = left_x - right_x
            
            # 避免除零错误
            if disparity <= 0:
                print("视差为零或负值，无法计算距离")
                return None
                
            # 计算距离(m)
            z = (self.baseline * self.focal_length) / disparity
            
            # 计算3D坐标
            x = (left_x - 320) * z / self.focal_length
            y = (left_y - 240) * z / self.focal_length
            
            print(f"检测到物体 {object_name} 位置: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
            return (x, y, z, left_bbox)
            
        except Exception as e:
            print(f"距离计算错误: {e}")
            return None
    
    def grab_object_with_vision(self, object_name="target", grab_distance=0.05, blocking=True, timeout=20.0, yolo_bbox=None):
        """
        使用视觉系统自动抓取物体
        
        Args:
            object_name: 要抓取的物体名称
            grab_distance: 抓取距离(m)，最终抓取位置会比检测到的物体位置稍微靠前
            blocking: 是否阻塞等待动作完成
            timeout: 最大等待时间(秒)
            yolo_bbox: YOLO检测框 [x1, y1, x2, y2]，如果提供则优先使用
            
        Returns:
            bool: 如果抓取成功则返回True
        """
        if not hasattr(self, 'cameras_initialized') or not self.cameras_initialized:
            print("双目相机未初始化")
            return False
            
        try:
            # 捕获双目图像
            left_frame, right_frame = self.capture_stereo_images()
            if left_frame is None or right_frame is None:
                print("无法获取双目图像")
                return False
                
            # 计算物体3D位置
            if yolo_bbox is not None:
                # 使用YOLO检测框
                print(f"使用YOLO检测框: {yolo_bbox}")
                result = self.calculate_distance_with_yolo(left_frame, right_frame, yolo_bbox)
            else:
                # 使用传统颜色检测
                result = self.calculate_distance(left_frame, right_frame, object_name)
                
            if result is None:
                print(f"无法定位物体: {object_name}")
                return False
                
            x, y, z, bbox = result
            
            # 计算抓取位置（稍微靠前一点）
            grab_z = z - grab_distance
            
            # 将相机坐标系中的坐标转换为机械臂坐标系中的坐标
            # 注意：这里需要根据实际相机安装位置和机械臂坐标系进行调整
            # 这里假设相机坐标系与机械臂末端坐标系重合
            arm_x = x
            arm_y = y
            arm_z = grab_z
            
            # 获取大臂电机位置
            pitch_pos, linear_pos = self.coordinate_to_motors(arm_x, arm_y, arm_z)
            
            print(f"移动到抓取位置: X={arm_x:.3f}m, Y={arm_y:.3f}m, Z={arm_z:.3f}m")
            print(f"大臂电机位置: 俯仰={pitch_pos}, 进给={linear_pos}")
            
            # 移动到预抓取位置（先到物体上方）
            pre_grab_z = arm_z + 0.1  # 比目标高10cm
            success = self.move_to_cartesian_with_motors(
                arm_x, arm_y, pre_grab_z, 
                pitch_pos=pitch_pos, 
                linear_pos=linear_pos,
                blocking=True,
                timeout=timeout
            )
            
            if not success:
                print("移动到预抓取位置失败")
                return False
                
            # 打开夹爪
            self.open_gripper()
            time.sleep(0.5)
            
            # 移动到抓取位置
            success = self.move_to_cartesian_with_motors(
                arm_x, arm_y, arm_z,
                blocking=True,
                timeout=timeout / 2
            )
            
            if not success:
                print("移动到抓取位置失败")
                return False
                
            # 关闭夹爪抓取物体
            self.close_gripper()
            time.sleep(1.0)
            
            # 提起物体（回到预抓取位置）
            success = self.move_to_cartesian_with_motors(
                arm_x, arm_y, pre_grab_z,
                blocking=blocking,
                timeout=timeout / 2
            )
            
            if not success and blocking:
                print("提起物体失败")
                return False
                
            print(f"成功抓取物体: {object_name}")
            return True
            
        except Exception as e:
            print(f"视觉抓取过程中发生错误: {e}")
            return False
    
    def open_gripper(self):
        """
        打开夹爪
        
        Returns:
            bool: 成功返回True
        """
        if not self.servo_connected:
            print("舵机控制器未连接")
            return False
            
        try:
            # 假设joint6是夹爪
            if 'joint6' in self.joint_ids:
                gripper_id = self.joint_ids['joint6']
                min_pos, max_pos = self.joint_limits['joint6']
                # 设置为最小位置（打开）
                return self.set_joint_position('joint6', min_pos)
            return False
        except Exception as e:
            print(f"打开夹爪时发生错误: {e}")
            return False
    
    def close_gripper(self):
        """
        关闭夹爪
        
        Returns:
            bool: 成功返回True
        """
        if not self.servo_connected:
            print("舵机控制器未连接")
            return False
            
        try:
            # 假设joint6是夹爪
            if 'joint6' in self.joint_ids:
                gripper_id = self.joint_ids['joint6']
                min_pos, max_pos = self.joint_limits['joint6']
                # 设置为关闭位置（位于最大位置和最小位置之间）
                close_pos = min_pos + (max_pos - min_pos) // 2
                return self.set_joint_position('joint6', close_pos)
            return False
        except Exception as e:
            print(f"关闭夹爪时发生错误: {e}")
            return False
    
    def release_cameras(self):
        """
        释放相机资源
        
        Returns:
            bool: 成功返回True
        """
        try:
            if hasattr(self, 'left_camera') and self.left_camera is not None:
                self.left_camera.release()
                
            if hasattr(self, 'right_camera') and self.right_camera is not None:
                self.right_camera.release()
                
            self.cameras_initialized = False
            print("已释放相机资源")
            return True
        except Exception as e:
            print(f"释放相机资源时发生错误: {e}")
            return False
    
    def scan_servos(self, start_id=1, end_id=10):
        """
        扫描指定ID范围内的舵机
        
        Args:
            start_id: 起始ID
            end_id: 结束ID
            
        Returns:
            list: 包含(servo_id, model_number)元组的列表
        """
        if not self.servo_connected:
            print("舵机控制器未连接")
            return []
            
        try:
            servos = []
            for servo_id in range(start_id, end_id + 1):
                model_number, result, error = self.servo.ping(servo_id)
                if result == COMM_SUCCESS and error == 0:
                    servos.append((servo_id, model_number))
                    print(f"找到舵机 ID: {servo_id}, 型号: {model_number}")
                    
            return servos
        except Exception as e:
            print(f"扫描舵机时发生错误: {e}")
            return []
    
    def set_speeds(self, servo_speed=None, pitch_speed=None, linear_speed=None):
        """
        设置舵机和电机的速度
        
        Args:
            servo_speed: 舵机速度（时间，毫秒，0=最大速度）
            pitch_speed: 俯仰电机速度
            linear_speed: 进给电机速度
            
        Returns:
            bool: 成功返回True
        """
        success = True
        
        # 设置舵机速度
        if servo_speed is not None:
            self.servo_speed = servo_speed
            if self.servo_connected:
                # 为所有已连接的舵机设置速度
                for joint_name, servo_id in self.joint_ids.items():
                    try:
                        self.servo.write_speed(servo_id, servo_speed)
                    except Exception as e:
                        print(f"设置舵机 {joint_name} (ID: {servo_id}) 速度时出错: {e}")
                        success = False
        
        # 设置电机速度
        if (pitch_speed is not None or linear_speed is not None) and self.motor_connected:
            try:
                self.motor.set_motor_speed(pitch_speed, linear_speed)
            except Exception as e:
                print(f"设置电机速度时出错: {e}")
                success = False
        
        return success
    
    def set_all_servo_acc(self, acceleration):
        """
        设置所有舵机的加速度
        
        Args:
            acceleration: 加速度值 (0-254)
            
        Returns:
            bool: 成功返回True
        """
        if not self.servo_connected:
            print("舵机控制器未连接")
            return False
            
        success = True
        self.servo_acceleration = acceleration
        
        # 为所有已连接的舵机设置加速度
        for joint_name, servo_id in self.joint_ids.items():
            try:
                self.servo.write_acceleration(servo_id, acceleration)
            except Exception as e:
                print(f"设置舵机 {joint_name} (ID: {servo_id}) 加速度时出错: {e}")
                success = False
                
        return success
    
    def stop_all(self):
        """
        停止所有电机和舵机
        
        Returns:
            bool: 成功返回True
        """
        success = True
        
        # 停止舵机
        if self.servo_connected:
            try:
                # 禁用舵机力矩
                for joint_name, servo_id in self.joint_ids.items():
                    try:
                        self.servo.set_torque_enable(servo_id, False)
                    except Exception as e:
                        print(f"禁用舵机 {joint_name} (ID: {servo_id}) 力矩时出错: {e}")
                        success = False
            except Exception as e:
                print(f"停止舵机时出错: {e}")
                success = False
        
        # 停止电机
        if self.motor_connected:
            try:
                self.motor.stop_all()
            except Exception as e:
                print(f"停止电机时出错: {e}")
                success = False
                
        return success
        
    def home(self, blocking=False, timeout=10.0):
        """
        将机械臂移动到初始位置（归位）
        
        Args:
            blocking: 是否阻塞等待动作完成
            timeout: 最大等待时间(秒)
            
        Returns:
            bool: 如果成功返回True
        """
        if not self.connected:
            print("控制器未连接")
            return False
            
        success = True
        
        try:
            # 设置舵机初始位置
            if self.servo_connected:
                # 定义初始位置（中间位置）
                home_positions = {
                    'joint1': 2048,  # 底座旋转
                    'joint2': 2048,  # 肩部
                    'joint3': 2048,  # 肘部
                    'joint4': 2048   # 腕部俯仰
                }
                
                # 设置所有关节位置
                for joint, position in home_positions.items():
                    if joint in self.joint_ids:
                        if not self.set_joint_position(joint, position, blocking=False):
                            print(f"设置关节 {joint} 初始位置失败")
                            success = False
            
            # 设置电机初始位置
            if self.motor_connected:
                # 设置俯仰电机位置（水平位置）
                if not self.set_pitch_position(0, blocking=False):
                    print("设置俯仰电机初始位置失败")
                    success = False
                    
                # 设置进给电机位置（初始位置）
                if not self.set_linear_position(0, blocking=False):
                    print("设置进给电机初始位置失败")
                    success = False
            
            # 如果需要阻塞等待
            if blocking and success:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    # 检查所有舵机是否到达目标位置
                    all_reached = True
                    
                    if self.servo_connected:
                        for joint, target_pos in home_positions.items():
                            if joint in self.joint_ids:
                                servo_id = self.joint_ids[joint]
                                current_pos = self.servo.read_position(servo_id)
                                
                                if current_pos is None or abs(current_pos - target_pos) > 10:
                                    all_reached = False
                                    break
                    
                    # 检查电机是否到达目标位置
                    if self.motor_connected:
                        # 检查俯仰电机
                        pitch_pos = self.motor.get_pitch_position()
                        if pitch_pos is None or abs(pitch_pos) > 100:
                            all_reached = False
                        
                        # 检查进给电机
                        linear_pos = self.motor.get_linear_position()
                        if linear_pos is None or abs(linear_pos) > 100:
                            all_reached = False
                    
                    if all_reached:
                        print("所有关节和电机已到达初始位置")
                        return True
                        
                    time.sleep(0.1)
                
                print("等待到达初始位置超时")
                return False
            
            return success
            
        except Exception as e:
            print(f"归位过程中出错: {e}")
            return False
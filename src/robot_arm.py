#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
机械臂控制类，集成 Feetech 舵机控制和 DC 电机控制
"""

from .feetech_servo_controller import FeetechServoController
from .dcmotor import DCMotorController
import time
import math


class RobotArm:
    """
    机械臂控制类，集成 Feetech 舵机控制和 DC 电机控制
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
    DEFAULT_PITCH_LIMITS = (-10000, 10000)
    DEFAULT_LINEAR_LIMITS = (0, 20000)
    
    def __init__(self, servo_port=None, motor_port=None, joint_ids=None, joint_limits=None, protocol_end=0):
        """
        初始化机械臂控制器
        
        Args:
            servo_port: 舵机控制串口
            motor_port: DC 电机控制串口
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
        self.pitch_limits = self.DEFAULT_PITCH_LIMITS
        self.linear_limits = self.DEFAULT_LINEAR_LIMITS
        
        # 当前关节位置
        self.current_positions = {}
        
        # 当前 DC 电机位置
        self.current_pitch = 0
        self.current_linear = 0
        
        # 运动速度
        self.servo_speed = 0  # 时间（毫秒，0 = 最大速度）
        self.pitch_speed = 100
        self.linear_speed = 100
    
    def connect(self, servo_port, motor_port, servo_baudrate=1000000, motor_baudrate=115200, protocol_end=0):
        """
        连接到舵机和电机控制器
        
        Args:
            servo_port: 舵机控制串口
            motor_port: DC 电机控制串口
            servo_baudrate: 舵机通信波特率
            motor_baudrate: 电机通信波特率
            protocol_end: 舵机协议位结束（STS/SMS=0, SCS=1），默认为 0
            
        Returns:
            bool: 如果两个连接都成功则返回 True，否则返回 False
        """
        servo_connected = self.servo.connect(servo_port, baudrate=servo_baudrate, protocol_end=protocol_end)
        motor_connected = self.motor.connect(motor_port, baudrate=motor_baudrate)
        
        return servo_connected and motor_connected
    
    def disconnect(self):
        """
        断开与两个控制器的连接
        """
        self.servo.disconnect()
        self.motor.disconnect()
    
    def enable_torque(self, enable=True):
        """
        使能或禁用所有舵机的力矩
        
        Args:
            enable: True 表示使能，False 表示禁用
            
        Returns:
            bool: 如果所有舵机都成功则返回 True，否则返回 False
        """
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
        读取两个 DC 电机的状态
        
        Returns:
            dict: 电机状态字典，如果出错则返回 None
        """
        status = self.motor.get_status()
        if status:
            self.current_pitch = status['pitch_position']
            self.current_linear = status['linear_position']
        
        return status
    
    def set_joint_position(self, joint, position, blocking=False, timeout=5.0):
        """
        设置特定关节的位置
        
        Args:
            joint: 关节名称
            position: 目标位置
            blocking: 如果为 True，等待运动完成
            timeout: 如果 blocking 为 True，最大等待时间（秒）
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        if joint not in self.joint_ids:
            print(f"未知关节: {joint}")
            return False
        
        servo_id = self.joint_ids[joint]
        position = self._clamp_joint_position(joint, position)
        
        success = self.servo.write_position(servo_id, position, self.servo_speed)
        
        if success and blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_pos = self.servo.read_position(servo_id)
                if current_pos is None:
                    return False
                
                # 检查位置是否足够接近
                if abs(current_pos - position) < 10:
                    break
                
                time.sleep(0.05)
        
        if success:
            self.current_positions[joint] = position
            
        return success
    
    def set_joint_positions(self, positions, blocking=False, timeout=5.0):
        """
        设置多个关节的位置
        
        Args:
            positions: {joint: position} 格式的字典
            blocking: 如果为 True，等待所有运动完成
            timeout: 如果 blocking 为 True，最大等待时间（秒）
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        # 将关节名称转换为舵机 ID，并限制位置
        servo_positions = {}
        times = {} if self.servo_speed > 0 else None
        
        for joint, position in positions.items():
            if joint in self.joint_ids:
                servo_id = self.joint_ids[joint]
                clamped_position = self._clamp_joint_position(joint, position)
                servo_positions[servo_id] = clamped_position
                if times is not None:
                    times[servo_id] = self.servo_speed
            else:
                print(f"未知关节: {joint}")
        
        if not servo_positions:
            return False
        
        # 使用同步写入同时设置位置
        success = self.servo.sync_write_position(servo_positions, times)
        
        if success and blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                all_reached = True
                
                for joint, position in positions.items():
                    if joint in self.joint_ids:
                        servo_id = self.joint_ids[joint]
                        current_pos = self.servo.read_position(servo_id)
                        
                        if current_pos is None:
                            return False
                        
                        # 检查位置是否足够接近
                        if abs(current_pos - servo_positions[servo_id]) >= 10:
                            all_reached = False
                            break
                
                if all_reached:
                    break
                    
                time.sleep(0.05)
        
        if success:
            for joint, position in positions.items():
                if joint in self.joint_ids:
                    self.current_positions[joint] = self._clamp_joint_position(joint, position)
        
        return success
    
    def set_pitch_position(self, position, blocking=False, timeout=5.0):
        """
        设置 pitch DC 电机位置
        
        Args:
            position: 目标位置
            blocking: 如果为 True，等待运动完成
            timeout: 如果 blocking 为 True，最大等待时间（秒）
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        position = self._clamp_motor_position('pitch', position)
        
        success = self.motor.set_pitch_position(position, self.pitch_speed)
        
        if success and blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.motor.get_status()
                
                if status is None:
                    return False
                
                if not status['pitch_moving'] and abs(status['pitch_position'] - position) < 10:
                    break
                    
                time.sleep(0.05)
        
        if success:
            self.current_pitch = position
            
        return success
    
    def set_linear_position(self, position, blocking=False, timeout=5.0):
        """
        设置线性进给 DC 电机位置
        
        Args:
            position: 目标位置
            blocking: 如果为 True，等待运动完成
            timeout: 如果 blocking 为 True，最大等待时间（秒）
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        position = self._clamp_motor_position('linear', position)
        
        success = self.motor.set_linear_position(position, self.linear_speed)
        
        if success and blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.motor.get_status()
                
                if status is None:
                    return False
                
                if not status['linear_moving'] and abs(status['linear_position'] - position) < 10:
                    break
                    
                time.sleep(0.05)
        
        if success:
            self.current_linear = position
            
        return success
    
    def set_speeds(self, servo_speed=None, pitch_speed=None, linear_speed=None):
        """
        设置不同电机的速度
        
        Args:
            servo_speed: 舵机运动的时间（毫秒，0 = 最大速度）
            pitch_speed: pitch 电机的速度 (0-255)
            linear_speed: 线性电机的速度 (0-255)
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        success = True
        
        if servo_speed is not None:
            self.servo_speed = max(0, servo_speed)
            
        if pitch_speed is not None or linear_speed is not None:
            success = self.motor.set_motor_speed(pitch_speed, linear_speed)
            
            if success:
                if pitch_speed is not None:
                    self.pitch_speed = pitch_speed
                if linear_speed is not None:
                    self.linear_speed = linear_speed
        
        return success
    
    def set_servo_acc(self, servo_id, acceleration):
        """
        设置特定舵机的加速度
        
        Args:
            servo_id: 舵机 ID
            acceleration: 加速度值 (0-254)
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        return self.servo.write_acceleration(servo_id, acceleration)
    
    def set_all_servo_acc(self, acceleration):
        """
        设置所有舵机的加速度
        
        Args:
            acceleration: 加速度值 (0-254)
            
        Returns:
            bool: 如果所有设置都成功则返回 True，否则返回 False
        """
        success = True
        for joint, servo_id in self.joint_ids.items():
            if not self.set_servo_acc(servo_id, acceleration):
                success = False
        return success
    
    def stop_all(self):
        """
        立即停止所有电机
        
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        dc_success = self.motor.stop_all()
        
        # 对于舵机，我们禁用力矩，这会有效地停止它们
        servo_success = self.enable_torque(False)
        
        return dc_success and servo_success
    
    def home(self, blocking=True, timeout=10.0):
        """
        将机械臂移动到原位
        
        Args:
            blocking: 如果为 True，等待运动完成
            timeout: 如果 blocking 为 True，最大等待时间（秒）
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        # 定义所有关节的原位位置
        home_positions = {
            'joint1': 2048,  # 中间位置
            'joint2': 2048,
            'joint3': 2048,
            'joint4': 2048
        }
        
        # 将所有关节设置为原位
        joints_success = self.set_joint_positions(home_positions, blocking=blocking, timeout=timeout)
        
        # 将 DC 电机归位
        dc_success = self.motor.home_motors()
        
        if dc_success and blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.motor.get_status()
                
                if status is None:
                    return False
                
                if not status['pitch_moving'] and not status['linear_moving']:
                    self.current_pitch = status['pitch_position']
                    self.current_linear = status['linear_position']
                    break
                    
                time.sleep(0.05)
        
        return joints_success and dc_success
    
    def calibrate(self):
        """
        执行校准程序
        
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        # 首先禁用力矩
        self.enable_torque(False)
        
        # 然后重新使能力矩
        servo_success = self.enable_torque(True)
        
        # 将电机归位
        return servo_success and self.home()
    
    def move_cartesian(self, x, y, z, speed=None):
        """
        将末端执行器移动到笛卡尔位置（简化实现）
        这是一个非常简化的实现，将坐标直接映射到电机
        对于真实实现，需要逆运动学
        
        Args:
            x: X 坐标（映射到 joint1）
            y: Y 坐标（映射到线性进给）
            z: Z 坐标（映射到俯仰）
            speed: 运动速度
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        if speed is not None:
            self.set_speeds(servo_speed=speed, pitch_speed=speed, linear_speed=speed)
        
        # 将 x 映射到 joint1（底座旋转）
        # 这是一个简化映射，在实际使用中需要适当的逆运动学
        joint1_pos = int(x * 10 + 2048)  # 简单缩放，2048 是中心
        
        # 将 y 映射到线性进给
        linear_pos = int(y * 100)
        
        # 将 z 映射到俯仰
        pitch_pos = int(z * 100)
        
        # 执行运动
        j1_success = self.set_joint_position('joint1', joint1_pos)
        linear_success = self.set_linear_position(linear_pos)
        pitch_success = self.set_pitch_position(pitch_pos)
        
        return j1_success and linear_success and pitch_success
    
    def scan_servos(self, start_id=1, end_id=10):
        """
        扫描并检测连接的舵机
        
        Args:
            start_id: 开始扫描的 ID
            end_id: 结束扫描的 ID
            
        Returns:
            list: 找到的舵机 ID 和型号列表
        """
        found_servos = []
        
        print(f"扫描舵机 ID {start_id} 到 {end_id}...")
        for i in range(start_id, end_id + 1):
            model_number, result, error = self.servo.ping(i)
            if result == COMM_SUCCESS:
                print(f"[ID:{i:03d}] 成功找到舵机，型号：{model_number}")
                found_servos.append((i, model_number))
            else:
                if error != 0:
                    print(f"[ID:{i:03d}] 通信错误: {self.servo.packet_handler.getRxPacketError(error)}")
        
        return found_servos 
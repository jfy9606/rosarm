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
        self.connected = False
    
    def connect(self, servo_port, motor_port, servo_baudrate=1000000, motor_baudrate=115200, protocol_end=0):
        """
        连接到机械臂控制器
        
        Args:
            servo_port: 舵机串口名称
            motor_port: 电机串口名称
            servo_baudrate: 舵机波特率
            motor_baudrate: 电机波特率
            protocol_end: 舵机协议结束位 (STS/SMS=0, SCS=1)
            
        Returns:
            bool: 如果连接成功则返回 True，否则返回 False
        """
        # 连接到舵机控制器
        servo_success = self.servo.connect(servo_port, servo_baudrate, protocol_end)
        
        # 连接到电机控制器
        motor_success = self.motor.connect(motor_port, motor_baudrate)
        
        # 如果两者都成功，则设置为已连接
        if servo_success and motor_success:
            self.connected = True
            print(f"Connected to robot arm controllers: servos on {servo_port}, motors on {motor_port}")
            
            # 初始化舵机和电机
            self._init_servos()
            
            return True
        else:
            # 如果有一个失败，断开另一个
            if servo_success and not motor_success:
                print("Failed to connect to motor controller, disconnecting servo controller")
                self.servo.disconnect()
            elif motor_success and not servo_success:
                print("Failed to connect to servo controller, disconnecting motor controller")
                self.motor.disconnect()
            
            self.connected = False
            return False
            
    def _init_servos(self):
        """
        初始化舵机，设置初始速度和加速度
        """
        try:
            # 检查所有舵机ID是否存在
            for servo_id in self.joint_ids.values():
                if not self.servo.ping(servo_id):
                    print(f"Warning: Servo ID {servo_id} not found")
            
            # 设置所有舵机的初始速度和加速度
            for joint, servo_id in self.joint_ids.items():
                # 设置速度
                self.servo.set_servo_speed(servo_id, self.servo_speed)
                
                # 设置加速度
                self.servo.set_servo_acceleration(servo_id, self.servo_acceleration)
                
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
    
    def set_pitch_position(self, position, speed=None, blocking=False, timeout=5.0):
        """
        设置YF俯仰电机位置
        
        Args:
            position: 目标位置
            speed: 可选的速度参数（0-255）
            blocking: 是否阻塞等待完成
            timeout: 阻塞等待的超时时间（秒）
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        # 限制位置在范围内
        position = self._clamp_motor_position('pitch', position)
        
        # 设置速度（如果提供）
        if speed is not None:
            self.pitch_speed = speed
            self.motor.set_motor_speed(pitch_speed=speed)
        
        # 尝试多次发送命令，增加成功率
        success = False
        retries = 3
        
        for attempt in range(retries):
            success = self.motor.set_pitch_position(position, self.pitch_speed)
            if success:
                break
            time.sleep(0.1)  # 短暂延迟后重试
        
        if not success:
            print(f"Failed to set pitch position after {retries} attempts")
            return False
        
        # 如果需要阻塞等待完成
        if blocking and success:
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 获取当前位置
                pitch_pos, _ = self.motor.get_motor_positions()
                
                # 如果获取位置失败，继续尝试
                if pitch_pos is None:
                    time.sleep(0.1)
                    continue
                
                # 检查是否到达目标位置附近
                if abs(pitch_pos - position) < 10:  # 允许误差范围
                    return True
                
                time.sleep(0.1)
            
            # 超时
            print(f"Timeout waiting for pitch motor to reach position {position}")
            return False
        
        return success
    
    def set_linear_position(self, position, speed=None, blocking=False, timeout=5.0):
        """
        设置AImotor进给电机位置
        
        Args:
            position: 目标位置
            speed: 可选的速度参数（0-255）
            blocking: 是否阻塞等待完成
            timeout: 阻塞等待的超时时间（秒）
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        # 限制位置在范围内
        position = self._clamp_motor_position('linear', position)
        
        # 设置速度（如果提供）
        if speed is not None:
            self.linear_speed = speed
            self.motor.set_motor_speed(linear_speed=speed)
        
        # 尝试多次发送命令，增加成功率
        success = False
        retries = 3
        
        for attempt in range(retries):
            success = self.motor.set_linear_position(position, self.linear_speed)
            if success:
                break
            time.sleep(0.1)  # 短暂延迟后重试
        
        if not success:
            print(f"Failed to set linear position after {retries} attempts")
            return False
        
        # 如果需要阻塞等待完成
        if blocking and success:
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 获取当前位置
                _, linear_pos = self.motor.get_motor_positions()
                
                # 如果获取位置失败，继续尝试
                if linear_pos is None:
                    time.sleep(0.1)
                    continue
                
                # 检查是否到达目标位置附近
                if abs(linear_pos - position) < 10:  # 允许误差范围
                    return True
                
                time.sleep(0.1)
            
            # 超时
            print(f"Timeout waiting for linear motor to reach position {position}")
            return False
        
        return success
    
    def set_speeds(self, servo_speed=None, pitch_speed=None, linear_speed=None):
        """
        设置不同电机的速度
        
        Args:
            servo_speed: FT系列舵机运动的时间（毫秒，0 = 最大速度）
            pitch_speed: YF型号俯仰电机的速度 (0-255)
            linear_speed: AImotor型号线性电机的速度 (0-255)
            
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
        将机械臂移动到初始位置
        - 舵机回到中间位置
        - DC 电机回到零位置
        
        Args:
            blocking: 如果为 True，等待运动完成
            timeout: 如果 blocking 为 True，最大等待时间（秒）
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        if not self.connected:
            return False
            
        # 先使能电机
        try:
            # 使能大臂DC电机
            self.motor.enable_motors(True, True)
        except Exception as e:
            print(f"Error enabling DC motors: {e}")
            return False
            
        # 先移动 DC 电机回零位置
        pitch_success = self.set_pitch_position(0, blocking=blocking, timeout=timeout)
        linear_success = self.set_linear_position(0, blocking=blocking, timeout=timeout)
        
        # 然后移动舵机到中间位置
        joint_positions = {
            'joint1': 2048,  # 底座中间位置
            'joint2': 2048,  # 肩部中间位置
            'joint3': 2048,  # 肘部中间位置
            'joint4': 2048   # 腕部中间位置
        }
        servo_success = self.set_joint_positions(joint_positions, blocking=blocking, timeout=timeout)
        
        return pitch_success and linear_success and servo_success
        
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
            x: X 坐标（映射到 joint1 FT系列舵机）
            y: Y 坐标（映射到 AImotor型号线性进给电机）
            z: Z 坐标（映射到 YF型号俯仰电机）
            speed: 运动速度
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        if speed is not None:
            self.set_speeds(servo_speed=speed, pitch_speed=speed, linear_speed=speed)
        
        # 将 x 映射到 joint1（底座旋转）
        # 这是一个简化映射，在实际使用中需要适当的逆运动学
        joint1_pos = int(x * 10 + 2048)  # 简单缩放，2048 是中心
        
        # 将 y 映射到 AImotor型号线性进给电机
        linear_pos = int(y * 100)
        
        # 将 z 映射到 YF型号俯仰电机
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
        if not self.servo or not self.servo.port_handler:
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
    
    def set_joint_positions_with_retry(self, positions, blocking=False, timeout=5.0, max_retries=3):
        """
        设置多个关节的位置，带有重试机制
        
        Args:
            positions: {joint: position} 格式的字典
            blocking: 如果为 True，等待所有运动完成
            timeout: 如果 blocking 为 True，最大等待时间（秒）
            max_retries: 最大重试次数
            
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        # 检查所有舵机的电压
        all_voltages_ok = True
        for joint, servo_id in self.joint_ids.items():
            if joint in positions:
                is_ok, voltage = self.check_servo_voltage(servo_id)
                if not is_ok:
                    print(f"警告: 舵机ID {servo_id} 电压异常: {voltage}V")
                    all_voltages_ok = False
        
        if not all_voltages_ok:
            print("由于电压问题，可能会出现运动错误")
        
        # 尝试最多max_retries次
        for attempt in range(max_retries):
            try:
                success = self.set_joint_positions(positions, blocking, timeout)
                if success:
                    return True
                    
                # 如果失败但还有重试机会，先暂停一会
                if attempt < max_retries - 1:
                    print(f"设置位置失败，暂停后重试 ({attempt+1}/{max_retries})")
                    time.sleep(0.2 * (attempt + 1))  # 递增等待时间
                    
            except Exception as e:
                print(f"移动舵机时出错: {e}")
                if attempt < max_retries - 1:
                    time.sleep(0.2 * (attempt + 1))
                else:
                    return False
        
        return False
        
    def set_joint_positions_sequential(self, positions, blocking=True):
        """
        按顺序设置多个关节的位置，而不是同时设置
        这可以减少瞬时功率需求，避免电压下降
        
        Args:
            positions: {joint: position} 格式的字典
            blocking: 如果为 True，等待每个关节运动完成
            
        Returns:
            bool: 如果所有关节都成功则返回 True，否则返回 False
        """
        all_success = True
        
        # 按顺序设置每个关节
        for joint, position in positions.items():
            success = self.set_joint_position(joint, position, blocking=blocking)
            if not success:
                all_success = False
                print(f"设置关节 {joint} 位置失败")
            
            # 即使是非阻塞模式，也稍微暂停一下，以避免同时启动
            time.sleep(0.05)
        
        return all_success 

    def test_dc_motors(self):
        """
        测试DC电机通信和基本功能
        
        Returns:
            dict: 测试结果
        """
        results = {
            "communication": False,
            "pitch_movement": False,
            "linear_movement": False,
            "status_reading": False,
            "speed_setting": False,
            "errors": []
        }
        
        try:
            # 测试通信
            status = self.motor.get_status()
            if status is not None:
                results["communication"] = True
                results["status_reading"] = True
                
                # 记录初始位置
                initial_pitch = status['pitch_position']
                initial_linear = status['linear_position']
                
                # 测试电机使能
                self.motor.enable_motors(True, True)
                
                # 测试速度设置
                speed_result = self.motor.set_motor_speed(50, 50)
                results["speed_setting"] = speed_result
                
                # 测试YF俯仰电机移动
                pitch_target = initial_pitch + 100
                pitch_result = self.motor.set_pitch_position(pitch_target)
                results["pitch_movement"] = pitch_result
                
                # 测试AImotor进给电机移动
                linear_target = initial_linear + 100
                linear_result = self.motor.set_linear_position(linear_target)
                results["linear_movement"] = linear_result
                
                # 等待一会儿
                time.sleep(1.0)
                
                # 读取新状态
                new_status = self.motor.get_status()
                
                # 停止所有电机
                self.motor.stop_all()
                
                if new_status:
                    # 检查是否移动了
                    if abs(new_status['pitch_position'] - initial_pitch) > 10:
                        results["pitch_movement"] = True
                    if abs(new_status['linear_position'] - initial_linear) > 10:
                        results["linear_movement"] = True
        except Exception as e:
            error_msg = str(e)
            results["errors"].append(error_msg)
            print(f"Error testing DC motors: {error_msg}")
        
        return results
        
    def home_motors(self):
        """
        将大臂 DC 电机归零
        
        Returns:
            bool: 如果成功则返回 True，否则返回 False
        """
        if not self.connected:
            return False
            
        try:
            return self.motor.home_motors()
        except Exception as e:
            print(f"Error homing motors: {e}")
            return False

    def get_motor_positions(self):
        """
        获取电机当前位置
        
        Returns:
            tuple: (pitch_position, linear_position) 或者如果失败则为 (None, None)
        """
        if not self.connected:
            print("Not connected to robot arm")
            return None, None
        
        try:
            return self.motor.get_motor_positions()
        except Exception as e:
            print(f"Error getting motor positions: {e}")
            return None, None 
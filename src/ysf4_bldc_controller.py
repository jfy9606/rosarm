import serial
import time
import struct
import logging
import threading
from typing import Optional, Dict, Any, Tuple

# 设置日志
logging.basicConfig(level=logging.DEBUG, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('YSF4BLDCController')

class YSF4BLDCController:
    """
    YSF4 HAL MOTOR-650 FOC v5.2.0 57BLDC HALLSensor 专用控制器
    
    支持功能:
    - FOC (Field Oriented Control) 控制算法
    - 霍尔传感器位置反馈
    - 过载/过流/过热保护
    - 可配置保护参数
    - 实时状态监控
    """
    
    # YSF4 协议常量
    FRAME_HEADER = 0x55  # 帧头
    FRAME_TAIL = 0xAA    # 帧尾
    
    # 命令码定义
    CMD_ENABLE = 0x01           # 使能电机
    CMD_DISABLE = 0x02          # 禁用电机
    CMD_SET_SPEED = 0x03        # 设置速度
    CMD_SET_POSITION = 0x04     # 设置位置
    CMD_SET_CURRENT = 0x05      # 设置电流
    CMD_READ_STATUS = 0x10      # 读取状态
    CMD_READ_POSITION = 0x11    # 读取位置
    CMD_READ_SPEED = 0x12       # 读取速度
    CMD_READ_CURRENT = 0x13     # 读取电流
    CMD_READ_TEMPERATURE = 0x14 # 读取温度
    CMD_READ_VOLTAGE = 0x15     # 读取电压
    CMD_READ_HALLS = 0x16       # 读取霍尔传感器
    CMD_SET_PROTECTION = 0x20   # 设置保护参数
    CMD_CLEAR_ERROR = 0x21      # 清除错误
    CMD_SAVE_CONFIG = 0x22      # 保存配置
    
    # 状态位定义
    STATUS_ENABLED = 0x01       # 使能状态
    STATUS_MOVING = 0x02        # 运动状态
    STATUS_POSITION_REACHED = 0x04  # 位置到达
    STATUS_OVERCURRENT = 0x08   # 过流保护
    STATUS_OVERHEAT = 0x10      # 过热保护
    STATUS_OVERVOLTAGE = 0x20   # 过压保护
    STATUS_UNDERVOLTAGE = 0x40  # 欠压保护
    STATUS_HALL_ERROR = 0x80    # 霍尔传感器错误
    
    # 默认保护参数 (针对YSF4-650电机优化)
    DEFAULT_MAX_CURRENT = 15.0     # 最大电流 15A (YSF4-650额定电流约12A)
    DEFAULT_MAX_TEMPERATURE = 80   # 最大温度 80°C
    DEFAULT_MAX_VOLTAGE = 30.0     # 最大电压 30V
    DEFAULT_MIN_VOLTAGE = 18.0     # 最小电压 18V
    DEFAULT_MAX_SPEED = 3000       # 最大转速 3000 RPM
    
    def __init__(self, motor_id: int = 1, port: Optional[str] = None, 
                 baudrate: int = 115200, timeout: float = 0.5, debug: bool = False):
        """
        初始化YSF4 BLDC电机控制器
        
        Args:
            motor_id: 电机ID (1-255)
            port: 串口名称
            baudrate: 波特率
            timeout: 超时时间
            debug: 调试模式
        """
        self.motor_id = motor_id
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.debug = debug
        self.serial = None
        self.connected = False
        
        # 电机状态
        self.enabled = False
        self.position = 0.0  # 位置 (度)
        self.speed = 0.0     # 速度 (RPM)
        self.current = 0.0   # 电流 (A)
        self.temperature = 0.0  # 温度 (°C)
        self.voltage = 0.0   # 电压 (V)
        self.hall_states = [False, False, False]  # 霍尔传感器状态
        self.status_flags = 0
        
        # 保护参数
        self.max_current = self.DEFAULT_MAX_CURRENT
        self.max_temperature = self.DEFAULT_MAX_TEMPERATURE
        self.max_voltage = self.DEFAULT_MAX_VOLTAGE
        self.min_voltage = self.DEFAULT_MIN_VOLTAGE
        self.max_speed = self.DEFAULT_MAX_SPEED
        
        # FOC参数
        self.foc_enabled = True
        self.hall_enabled = True
        self.pole_pairs = 4  # YSF4-650电机极对数
        
        # 通信控制
        self.comm_lock = threading.Lock()
        self.last_comm_time = 0
        self.min_comm_interval = 0.01  # 最小通信间隔 10ms
        
        # 监控线程
        self.monitor_thread = None
        self.monitor_running = False
        self.monitor_interval = 0.1  # 监控间隔 100ms
        
        if debug:
            logger.setLevel(logging.DEBUG)
        
        if port:
            self.connect(port, baudrate, timeout)
    
    def connect(self, port: str, baudrate: int = 115200, timeout: float = 0.5) -> bool:
        """
        连接到电机控制器
        
        Args:
            port: 串口名称
            baudrate: 波特率
            timeout: 超时时间
            
        Returns:
            bool: 连接成功返回True
        """
        try:
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            
            if self.serial and self.serial.is_open:
                self.serial.close()
                
            logger.info(f"连接YSF4电机控制器 - 端口: {port}, 波特率: {baudrate}")
            
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # 清空缓冲区
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            time.sleep(0.5)  # 等待电机控制器稳定
            
            # 测试通信
            if self.test_communication():
                self.connected = True
                logger.info(f"YSF4电机 ID={self.motor_id} 连接成功")
                
                # 读取初始状态
                self.read_all_status()
                
                # 设置保护参数
                self.configure_protection()
                
                # 启动监控线程
                self.start_monitoring()
                
                return True
            else:
                logger.error("YSF4电机通信测试失败")
                return False
                
        except Exception as e:
            logger.error(f"连接YSF4电机失败: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> bool:
        """断开连接"""
        try:
            # 停止监控
            self.stop_monitoring()
            
            # 禁用电机
            if self.connected:
                self.disable()
            
            if self.serial and self.serial.is_open:
                self.serial.close()
                
            self.connected = False
            logger.info("YSF4电机已断开连接")
            return True
            
        except Exception as e:
            logger.error(f"断开YSF4电机连接失败: {e}")
            return False
    
    def _calculate_checksum(self, data: bytes) -> int:
        """计算校验和"""
        return sum(data) & 0xFF
    
    def _send_command(self, cmd: int, data: bytes = b'') -> Optional[bytes]:
        """
        发送命令到电机
        
        Args:
            cmd: 命令码
            data: 数据
            
        Returns:
            bytes: 响应数据，失败返回None
        """
        if not self.connected or not self.serial:
            logger.error("YSF4电机未连接")
            return None
        
        with self.comm_lock:
            try:
                # 限制通信频率
                current_time = time.time()
                time_diff = current_time - self.last_comm_time
                if time_diff < self.min_comm_interval:
                    time.sleep(self.min_comm_interval - time_diff)
                
                # 构建命令帧
                frame = bytearray()
                frame.append(self.FRAME_HEADER)  # 帧头
                frame.append(self.motor_id)      # 电机ID
                frame.append(cmd)                # 命令码
                frame.append(len(data))          # 数据长度
                frame.extend(data)               # 数据
                
                # 计算校验和
                checksum = self._calculate_checksum(frame[1:])
                frame.append(checksum)
                frame.append(self.FRAME_TAIL)    # 帧尾
                
                if self.debug:
                    logger.debug(f"发送命令: {' '.join(f'{b:02X}' for b in frame)}")
                
                # 发送命令
                self.serial.write(frame)
                self.last_comm_time = time.time()
                
                # 读取响应
                time.sleep(0.01)  # 等待响应
                
                if self.serial.in_waiting > 0:
                    response = self.serial.read(self.serial.in_waiting)
                    
                    if self.debug:
                        logger.debug(f"接收响应: {' '.join(f'{b:02X}' for b in response)}")
                    
                    # 验证响应
                    if len(response) >= 6:
                        if (response[0] == self.FRAME_HEADER and 
                            response[1] == self.motor_id and
                            response[-1] == self.FRAME_TAIL):
                            
                            # 验证校验和
                            calc_checksum = self._calculate_checksum(response[1:-2])
                            recv_checksum = response[-2]
                            
                            if calc_checksum == recv_checksum:
                                data_len = response[3]
                                return response[4:4+data_len]
                            else:
                                logger.warning(f"校验和错误: 计算={calc_checksum:02X}, 接收={recv_checksum:02X}")
                        else:
                            logger.warning("响应帧格式错误")
                    else:
                        logger.warning(f"响应长度不足: {len(response)}")
                
                return None
                
            except Exception as e:
                logger.error(f"发送命令失败: {e}")
                return None
    
    def test_communication(self) -> bool:
        """测试通信"""
        try:
            response = self._send_command(self.CMD_READ_STATUS)
            return response is not None
        except Exception as e:
            logger.error(f"通信测试失败: {e}")
            return False
    
    def enable(self) -> bool:
        """使能电机"""
        try:
            response = self._send_command(self.CMD_ENABLE)
            if response is not None:
                self.enabled = True
                logger.info(f"YSF4电机 ID={self.motor_id} 已使能")
                return True
            return False
        except Exception as e:
            logger.error(f"使能电机失败: {e}")
            return False
    
    def disable(self) -> bool:
        """禁用电机"""
        try:
            response = self._send_command(self.CMD_DISABLE)
            if response is not None:
                self.enabled = False
                logger.info(f"YSF4电机 ID={self.motor_id} 已禁用")
                return True
            return False
        except Exception as e:
            logger.error(f"禁用电机失败: {e}")
            return False
    
    def set_speed(self, speed: float) -> bool:
        """
        设置电机速度
        
        Args:
            speed: 目标速度 (RPM)
            
        Returns:
            bool: 设置成功返回True
        """
        try:
            # 限制速度范围
            speed = max(-self.max_speed, min(self.max_speed, speed))
            
            # 转换为整数 (0.1 RPM精度)
            speed_int = int(speed * 10)
            
            # 构建数据
            data = struct.pack('<h', speed_int)
            
            response = self._send_command(self.CMD_SET_SPEED, data)
            if response is not None:
                logger.info(f"设置YSF4电机速度: {speed} RPM")
                return True
            return False
            
        except Exception as e:
            logger.error(f"设置速度失败: {e}")
            return False
    
    def set_position(self, position: float, speed: Optional[float] = None) -> bool:
        """
        设置电机位置
        
        Args:
            position: 目标位置 (度)
            speed: 运动速度 (RPM, 可选)
            
        Returns:
            bool: 设置成功返回True
        """
        try:
            # 转换为整数 (0.01度精度)
            pos_int = int(position * 100)
            
            if speed is not None:
                speed = max(1, min(self.max_speed, abs(speed)))
                speed_int = int(speed * 10)
                data = struct.pack('<lh', pos_int, speed_int)
            else:
                data = struct.pack('<l', pos_int)
            
            response = self._send_command(self.CMD_SET_POSITION, data)
            if response is not None:
                logger.info(f"设置YSF4电机位置: {position}°" + 
                           (f", 速度: {speed} RPM" if speed else ""))
                return True
            return False
            
        except Exception as e:
            logger.error(f"设置位置失败: {e}")
            return False
    
    def set_current(self, current: float) -> bool:
        """
        设置电机电流
        
        Args:
            current: 目标电流 (A)
            
        Returns:
            bool: 设置成功返回True
        """
        try:
            # 限制电流范围
            current = max(-self.max_current, min(self.max_current, current))
            
            # 转换为整数 (0.01A精度)
            current_int = int(current * 100)
            
            data = struct.pack('<h', current_int)
            
            response = self._send_command(self.CMD_SET_CURRENT, data)
            if response is not None:
                logger.info(f"设置YSF4电机电流: {current} A")
                return True
            return False
            
        except Exception as e:
            logger.error(f"设置电流失败: {e}")
            return False
    
    def read_position(self) -> Optional[float]:
        """读取电机位置"""
        try:
            response = self._send_command(self.CMD_READ_POSITION)
            if response and len(response) >= 4:
                pos_int = struct.unpack('<l', response[:4])[0]
                self.position = pos_int / 100.0  # 转换为度
                return self.position
            return None
        except Exception as e:
            logger.error(f"读取位置失败: {e}")
            return None
    
    def read_speed(self) -> Optional[float]:
        """读取电机速度"""
        try:
            response = self._send_command(self.CMD_READ_SPEED)
            if response and len(response) >= 2:
                speed_int = struct.unpack('<h', response[:2])[0]
                self.speed = speed_int / 10.0  # 转换为RPM
                return self.speed
            return None
        except Exception as e:
            logger.error(f"读取速度失败: {e}")
            return None
    
    def read_current(self) -> Optional[float]:
        """读取电机电流"""
        try:
            response = self._send_command(self.CMD_READ_CURRENT)
            if response and len(response) >= 2:
                current_int = struct.unpack('<h', response[:2])[0]
                self.current = current_int / 100.0  # 转换为A
                return self.current
            return None
        except Exception as e:
            logger.error(f"读取电流失败: {e}")
            return None
    
    def read_temperature(self) -> Optional[float]:
        """读取电机温度"""
        try:
            response = self._send_command(self.CMD_READ_TEMPERATURE)
            if response and len(response) >= 2:
                temp_int = struct.unpack('<h', response[:2])[0]
                self.temperature = temp_int / 10.0  # 转换为°C
                return self.temperature
            return None
        except Exception as e:
            logger.error(f"读取温度失败: {e}")
            return None
    
    def read_voltage(self) -> Optional[float]:
        """读取电机电压"""
        try:
            response = self._send_command(self.CMD_READ_VOLTAGE)
            if response and len(response) >= 2:
                voltage_int = struct.unpack('<h', response[:2])[0]
                self.voltage = voltage_int / 100.0  # 转换为V
                return self.voltage
            return None
        except Exception as e:
            logger.error(f"读取电压失败: {e}")
            return None
    
    def read_hall_sensors(self) -> Optional[Tuple[bool, bool, bool]]:
        """读取霍尔传感器状态"""
        try:
            response = self._send_command(self.CMD_READ_HALLS)
            if response and len(response) >= 1:
                hall_byte = response[0]
                self.hall_states = [
                    bool(hall_byte & 0x01),
                    bool(hall_byte & 0x02),
                    bool(hall_byte & 0x04)
                ]
                return tuple(self.hall_states)
            return None
        except Exception as e:
            logger.error(f"读取霍尔传感器失败: {e}")
            return None
    
    def read_status(self) -> Optional[int]:
        """读取电机状态"""
        try:
            response = self._send_command(self.CMD_READ_STATUS)
            if response and len(response) >= 1:
                self.status_flags = response[0]
                
                # 更新状态标志
                self.enabled = bool(self.status_flags & self.STATUS_ENABLED)
                
                # 检查错误状态
                if self.status_flags & self.STATUS_OVERCURRENT:
                    logger.warning("YSF4电机过流保护激活")
                if self.status_flags & self.STATUS_OVERHEAT:
                    logger.warning("YSF4电机过热保护激活")
                if self.status_flags & self.STATUS_OVERVOLTAGE:
                    logger.warning("YSF4电机过压保护激活")
                if self.status_flags & self.STATUS_UNDERVOLTAGE:
                    logger.warning("YSF4电机欠压保护激活")
                if self.status_flags & self.STATUS_HALL_ERROR:
                    logger.warning("YSF4电机霍尔传感器错误")
                
                return self.status_flags
            return None
        except Exception as e:
            logger.error(f"读取状态失败: {e}")
            return None
    
    def read_all_status(self) -> Dict[str, Any]:
        """读取所有状态信息"""
        status = {}
        
        try:
            # 读取各项状态
            status['position'] = self.read_position()
            status['speed'] = self.read_speed()
            status['current'] = self.read_current()
            status['temperature'] = self.read_temperature()
            status['voltage'] = self.read_voltage()
            status['hall_sensors'] = self.read_hall_sensors()
            status['status_flags'] = self.read_status()
            status['enabled'] = self.enabled
            
            # 检查保护状态
            status['protection'] = {
                'overcurrent': bool(self.status_flags & self.STATUS_OVERCURRENT),
                'overheat': bool(self.status_flags & self.STATUS_OVERHEAT),
                'overvoltage': bool(self.status_flags & self.STATUS_OVERVOLTAGE),
                'undervoltage': bool(self.status_flags & self.STATUS_UNDERVOLTAGE),
                'hall_error': bool(self.status_flags & self.STATUS_HALL_ERROR)
            }
            
        except Exception as e:
            logger.error(f"读取全部状态失败: {e}")
        
        return status
    
    def configure_protection(self) -> bool:
        """配置保护参数"""
        try:
            # 构建保护参数数据
            data = bytearray()
            
            # 最大电流 (0.01A精度)
            data.extend(struct.pack('<H', int(self.max_current * 100)))
            
            # 最大温度 (0.1°C精度)
            data.extend(struct.pack('<H', int(self.max_temperature * 10)))
            
            # 最大电压 (0.01V精度)
            data.extend(struct.pack('<H', int(self.max_voltage * 100)))
            
            # 最小电压 (0.01V精度)
            data.extend(struct.pack('<H', int(self.min_voltage * 100)))
            
            # 最大速度 (0.1 RPM精度)
            data.extend(struct.pack('<H', int(self.max_speed * 10)))
            
            response = self._send_command(self.CMD_SET_PROTECTION, data)
            if response is not None:
                logger.info(f"YSF4电机保护参数已配置 - 最大电流: {self.max_current}A, "
                           f"最大温度: {self.max_temperature}°C, "
                           f"电压范围: {self.min_voltage}-{self.max_voltage}V, "
                           f"最大速度: {self.max_speed} RPM")
                return True
            return False
            
        except Exception as e:
            logger.error(f"配置保护参数失败: {e}")
            return False
    
    def clear_errors(self) -> bool:
        """清除错误状态"""
        try:
            response = self._send_command(self.CMD_CLEAR_ERROR)
            if response is not None:
                logger.info("YSF4电机错误状态已清除")
                return True
            return False
        except Exception as e:
            logger.error(f"清除错误失败: {e}")
            return False
    
    def save_configuration(self) -> bool:
        """保存配置到非易失性存储器"""
        try:
            response = self._send_command(self.CMD_SAVE_CONFIG)
            if response is not None:
                logger.info("YSF4电机配置已保存")
                return True
            return False
        except Exception as e:
            logger.error(f"保存配置失败: {e}")
            return False
    
    def set_protection_limits(self, max_current: Optional[float] = None,
                            max_temperature: Optional[int] = None,
                            max_voltage: Optional[float] = None,
                            min_voltage: Optional[float] = None,
                            max_speed: Optional[int] = None) -> bool:
        """
        设置保护限制参数
        
        Args:
            max_current: 最大电流 (A)
            max_temperature: 最大温度 (°C)
            max_voltage: 最大电压 (V)
            min_voltage: 最小电压 (V)
            max_speed: 最大速度 (RPM)
            
        Returns:
            bool: 设置成功返回True
        """
        if max_current is not None:
            self.max_current = max_current
        if max_temperature is not None:
            self.max_temperature = max_temperature
        if max_voltage is not None:
            self.max_voltage = max_voltage
        if min_voltage is not None:
            self.min_voltage = min_voltage
        if max_speed is not None:
            self.max_speed = max_speed
        
        return self.configure_protection()
    
    def start_monitoring(self):
        """启动状态监控线程"""
        if self.monitor_thread is not None and self.monitor_thread.is_alive():
            return
        
        self.monitor_running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        logger.info("YSF4电机状态监控已启动")
    
    def stop_monitoring(self):
        """停止状态监控线程"""
        self.monitor_running = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)
        logger.info("YSF4电机状态监控已停止")
    
    def _monitor_loop(self):
        """监控循环"""
        while self.monitor_running and self.connected:
            try:
                # 读取关键状态
                self.read_status()
                self.read_current()
                self.read_temperature()
                
                # 检查保护状态
                if self.status_flags & (self.STATUS_OVERCURRENT | 
                                      self.STATUS_OVERHEAT | 
                                      self.STATUS_OVERVOLTAGE | 
                                      self.STATUS_UNDERVOLTAGE):
                    logger.error("YSF4电机保护激活，自动禁用电机")
                    self.disable()
                
                time.sleep(self.monitor_interval)
                
            except Exception as e:
                logger.error(f"监控线程错误: {e}")
                time.sleep(1.0)
    
    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.connected and self.serial and self.serial.is_open
    
    def get_motor_info(self) -> Dict[str, Any]:
        """获取电机信息"""
        return {
            'model': 'YSF4_HAL_MOTOR-650',
            'version': 'FOC_v5.2.0_57BLDC_HALLSensor',
            'motor_id': self.motor_id,
            'pole_pairs': self.pole_pairs,
            'foc_enabled': self.foc_enabled,
            'hall_enabled': self.hall_enabled,
            'protection_limits': {
                'max_current': self.max_current,
                'max_temperature': self.max_temperature,
                'max_voltage': self.max_voltage,
                'min_voltage': self.min_voltage,
                'max_speed': self.max_speed
            }
        }
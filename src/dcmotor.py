import serial
import platform
import time
import struct
import sys
import logging

# 设置日志
logging.basicConfig(level=logging.DEBUG, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('DCMotorController')

class DCMotorController:
    """
    Controller class for the two large DC motors:
    - YF型号: 大臂的俯仰电机（pitch motor）
    - AImotor型号: 大臂的进给电机（linear feed motor）
    通过串口通信控制
    """
    
    # Command codes - 基于YF和AIMotor系列的通信协议
    CMD_SET_PITCH = 0x01    # YF俯仰电机命令
    CMD_SET_LINEAR = 0x02   # AImotor进给电机命令
    CMD_STOP_ALL = 0x03     # 停止所有电机
    CMD_GET_STATUS = 0x04   # 获取状态
    CMD_SET_SPEED = 0x05    # 设置速度
    CMD_SET_ACCEL = 0x06    # 设置加速度
    CMD_HOME = 0x07         # 回零点
    
    # YF系列特定命令
    YF_HEADER = 0xEB        # YF协议帧头
    YF_ADDR = 0x90          # 默认地址
    
    # AIMotor系列特定命令
    AI_HEADER = 0xAA        # AIMotor协议帧头
    AI_ADDR = 0x01          # 默认地址

    # 基于SimpleNetwork实现的通信协议
    YF_MODBUS_ADDR = 0x3E   # YF Modbus地址
    YF_FUNC_WRITE = 0x01    # YF写入命令
    YF_CMD_MOVE = 0x08      # YF位置控制命令
    YF_CMD_ENABLE = 0x81    # YF使能命令
    YF_CMD_DISABLE = 0x80   # YF禁用命令

    # AIMotor ModBus命令
    AI_MODBUS_ADDR = 0x03   # AIMotor默认地址
    AI_WRITE_REG = 0x06     # 写寄存器
    AI_READ_REG = 0x03      # 读寄存器
    
    def __init__(self, port=None, baudrate=115200, timeout=0.5):
        """
        Initialize DC motor controller
        
        Args:
            port: Serial port name. If None, needs to be connected later
            baudrate: Baud rate for serial communication
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.debug = False
        
        # 电机参数
        self.pitch_position = 0  # YF电机位置
        self.linear_position = 0  # AIMotor电机位置
        self.pitch_speed = 100    # YF电机速度 (0-255)
        self.linear_speed = 100   # AIMotor电机速度 (0-255)
        
        # 使能状态
        self.pitch_enabled = False
        self.linear_enabled = False

        # 上次通信时间戳
        self.last_comm_time = 0
        
        if port:
            self.connect(port, baudrate, timeout)
    
    def set_debug(self, debug=True):
        """设置调试模式"""
        self.debug = debug
        if debug:
            logger.setLevel(logging.DEBUG)
        else:
            logger.setLevel(logging.INFO)
    
    def connect(self, port, baudrate=115200, timeout=0.5):
        """
        Connect to the motor controller
        
        Args:
            port: Serial port name
            baudrate: Baud rate for serial communication
            timeout: Serial timeout in seconds
            
        Returns:
            bool: True if connected successfully, False otherwise
        """
        try:
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            
            # 关闭之前的连接
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            # 创建新的串口连接
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
            
            self.connected = True
            logger.info(f"Connected to motor controller on {port} at {baudrate} baud")
            
            # 初始化电机
            time.sleep(0.5)  # 等待电机控制器稳定
            self._send_stop_command()  # 初始化时停止所有电机
            
            return True
        except Exception as e:
            logger.error(f"Failed to connect to motor controller: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """
        Disconnect from the motor controller
        
        Returns:
            bool: True if disconnected successfully, False otherwise
        """
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.connected = False
            logger.info("Disconnected from motor controller")
            return True
        except Exception as e:
            logger.error(f"Failed to disconnect from motor controller: {e}")
            return False
    
    def is_connected(self):
        """
        Check if connected to the motor controller
            
        Returns:
            bool: True if connected, False otherwise
        """
        return self.connected and self.serial and self.serial.is_open
    
    def set_pitch_position(self, position, speed=None):
        """
        Set the position of the YF pitch motor
        
        Args:
            position: Target position
            speed: Optional speed parameter (0-255)
            
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
        
        if speed is not None:
            self.pitch_speed = min(255, max(0, speed))
        
        try:
            # YF电机使用Modbus协议
            # 帧格式: 帧头(1B) + 功能码(1B) + 命令长度(1B) + 命令(1B) + 数据(4B) + 校验和(2B)
            data = [
                self.YF_MODBUS_ADDR,  # 地址
                self.YF_FUNC_WRITE,   # 功能码
                0x08,                 # 命令长度
                self.YF_CMD_MOVE,     # 命令
                (position >> 24) & 0xFF,  # 位置 (高字节)
                (position >> 16) & 0xFF,
                (position >> 8) & 0xFF,
                position & 0xFF,      # 位置 (低字节)
                self.pitch_speed & 0xFF,  # 速度
                0x00,                 # 预留
                0x00                  # 预留
            ]
            
            # 计算CRC校验
            crc = self._calculate_crc16(data, len(data))
            data.append(crc & 0xFF)
            data.append((crc >> 8) & 0xFF)
            
            command = bytearray(data)
            
            if self.debug:
                logger.debug(f"Sending YF pitch command: {' '.join(f'{b:02X}' for b in command)}")
            
            # 避免串口过度占用，添加延时
            self._delay_if_needed()
            
            # 发送命令
            self.serial.write(command)
            
            # 等待响应
            time.sleep(0.1)
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received response: {' '.join(f'{b:02X}' for b in response)}")
                
                # 检查响应是否有效
                if len(response) >= 3 and response[0] == self.YF_MODBUS_ADDR:
                    self.pitch_position = position
            return True
            
            logger.warning("No valid response from YF motor")
            return False
            
        except Exception as e:
            logger.error(f"Error sending command to YF pitch motor: {e}")
            return False
    
    def set_linear_position(self, position, speed=None):
        """
        Set the position of the AIMotor linear motor
        
        Args:
            position: Target position
            speed: Optional speed parameter (0-255)
            
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
        
        if speed is not None:
            self.linear_speed = min(255, max(0, speed))
        
        try:
            # AIMotor使用标准Modbus协议
            # 帧格式: 地址(1B) + 命令(1B) + 寄存器地址高(1B) + 寄存器地址低(1B) + 数据长度(2B) + 数据(nB) + CRC(2B)
            
            # 设置绝对位置模式
            mode_cmd = [
                self.AI_MODBUS_ADDR,  # 地址
                self.AI_WRITE_REG,    # 功能码: 写寄存器
                0x11,                 # 寄存器地址高: 位置模式寄存器
                0x04,                 # 寄存器地址低
                0x00,                 # 数据高字节
                0x01                  # 数据低字节: 绝对位置模式
            ]
            
            crc = self._calculate_crc16(mode_cmd, len(mode_cmd))
            mode_cmd.append(crc & 0xFF)
            mode_cmd.append((crc >> 8) & 0xFF)
            
            # 避免串口过度占用，添加延时
            self._delay_if_needed()
            
            # 发送模式设置命令
            self.serial.write(bytearray(mode_cmd))
            time.sleep(0.05)
            
            # 设置速度
            speed_cmd = [
                self.AI_MODBUS_ADDR,  # 地址
                self.AI_WRITE_REG,    # 功能码: 写寄存器
                0x06,                 # 寄存器地址高: 速度寄存器
                0x03,                 # 寄存器地址低
                (self.linear_speed >> 8) & 0xFF,  # 速度高字节
                self.linear_speed & 0xFF         # 速度低字节
            ]
            
            crc = self._calculate_crc16(speed_cmd, len(speed_cmd))
            speed_cmd.append(crc & 0xFF)
            speed_cmd.append((crc >> 8) & 0xFF)
            
            # 发送速度设置命令
            self._delay_if_needed()
            self.serial.write(bytearray(speed_cmd))
            time.sleep(0.05)
            
            # 设置位置
            pos_cmd = [
                self.AI_MODBUS_ADDR,  # 地址
                self.AI_WRITE_REG,    # 功能码: 写寄存器
                0x05,                 # 寄存器地址高: 位置寄存器
                0x00,                 # 寄存器地址低
                (position >> 8) & 0xFF,  # 位置高字节
                position & 0xFF          # 位置低字节
            ]
            
            crc = self._calculate_crc16(pos_cmd, len(pos_cmd))
            pos_cmd.append(crc & 0xFF)
            pos_cmd.append((crc >> 8) & 0xFF)
            
            # 发送位置设置命令
            self._delay_if_needed()
            self.serial.write(bytearray(pos_cmd))
            
            # 等待响应
            time.sleep(0.1)
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received response: {' '.join(f'{b:02X}' for b in response)}")
                
                # 记录新的位置
                self.linear_position = position
                return True
            
            logger.warning("No response from AIMotor")
            return False
            
        except Exception as e:
            logger.error(f"Error sending command to AIMotor linear motor: {e}")
            return False
    
    def _calculate_crc16(self, data, length):
        """计算Modbus CRC-16校验码"""
        crc = 0xFFFF
        
        for i in range(length):
            crc ^= data[i] & 0xFF
            
            for _ in range(8):
                if (crc & 0x0001) != 0:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
                    
        return crc
    
    def _delay_if_needed(self):
        """添加必要的延时以避免串口通信过于频繁"""
        current_time = time.time()
        if current_time - self.last_comm_time < 0.02:  # 确保命令间隔至少20ms
            time.sleep(0.02 - (current_time - self.last_comm_time))
        self.last_comm_time = time.time()
    
    def _send_stop_command(self):
        """
        Send stop command to all motors
            
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
        
        success = True
        
        try:
            # 发送YF电机停止命令
            yf_cmd = [
                self.YF_MODBUS_ADDR,  # 地址
                self.YF_FUNC_WRITE,   # 功能码
                0x08,                 # 命令长度
                self.YF_CMD_DISABLE,  # 停止命令
                0x00, 0x00, 0x00, 0x00,  # 位置 (不重要)
                0x00,                 # 速度 (不重要)
                0x00,                 # 预留
                0x00                  # 预留
            ]
            
            crc = self._calculate_crc16(yf_cmd, len(yf_cmd))
            yf_cmd.append(crc & 0xFF)
            yf_cmd.append((crc >> 8) & 0xFF)
            
            # 发送YF停止命令
            self.serial.write(bytearray(yf_cmd))
            time.sleep(0.05)
            
            # 发送AIMotor电机停止命令 - 通过设置速度为0
            ai_cmd = [
                self.AI_MODBUS_ADDR,  # 地址
                self.AI_WRITE_REG,    # 功能码: 写寄存器
                0x06,                 # 寄存器地址高: 速度寄存器
                0x03,                 # 寄存器地址低
                0x00, 0x00           # 速度为0
            ]
            
            crc = self._calculate_crc16(ai_cmd, len(ai_cmd))
            ai_cmd.append(crc & 0xFF)
            ai_cmd.append((crc >> 8) & 0xFF)
            
            # 发送AIMotor停止命令
            self.serial.write(bytearray(ai_cmd))
            time.sleep(0.05)
            
            # 清空接收缓冲区
            if self.serial.in_waiting:
                self.serial.read(self.serial.in_waiting)
            
            logger.info("Stop command sent to all motors")
            
            # 更新使能状态
            self.pitch_enabled = False
            self.linear_enabled = False
            
            return success
            
        except Exception as e:
            logger.error(f"Error sending stop command: {e}")
            return False
        
    def stop_all(self):
        """
        Stop all motors
        
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        return self._send_stop_command()
    
    def set_motor_speed(self, pitch_speed=None, linear_speed=None):
        """
        Set the speed of the motors
        
        Args:
            pitch_speed: Speed for YF pitch motor (0-255)
            linear_speed: Speed for AIMotor linear motor (0-255)
            
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
        
        success = True
        
        # 设置YF电机速度
        if pitch_speed is not None:
            self.pitch_speed = min(255, max(0, pitch_speed))
            logger.info(f"Set YF pitch speed to {self.pitch_speed}")
        
        # 设置AIMotor电机速度
        if linear_speed is not None:
            self.linear_speed = min(255, max(0, linear_speed))
            try:
                # 避免串口过度占用，添加延时
                self._delay_if_needed()
                
                # 发送AIMotor电机速度命令
                ai_cmd = [
                    self.AI_MODBUS_ADDR,  # 地址
                    self.AI_WRITE_REG,    # 功能码: 写寄存器
                    0x06,                 # 寄存器地址高: 速度寄存器
                    0x03,                 # 寄存器地址低
                    (self.linear_speed >> 8) & 0xFF,  # 速度高字节
                    self.linear_speed & 0xFF         # 速度低字节
                ]
                
                crc = self._calculate_crc16(ai_cmd, len(ai_cmd))
                ai_cmd.append(crc & 0xFF)
                ai_cmd.append((crc >> 8) & 0xFF)
                
                if self.debug:
                    logger.debug(f"Setting AIMotor linear speed to {self.linear_speed}: {' '.join(f'{b:02X}' for b in ai_cmd)}")
                
                self.serial.write(bytearray(ai_cmd))
                time.sleep(0.05)
                
                # 读取响应
                if self.serial.in_waiting:
                    response = self.serial.read(self.serial.in_waiting)
                    if self.debug:
                        logger.debug(f"Received response: {' '.join(f'{b:02X}' for b in response)}")
                
                logger.info(f"Set AIMotor linear speed to {self.linear_speed}")
                
            except Exception as e:
                logger.error(f"Error setting AIMotor linear speed: {e}")
                success = False
        
        return success
    
    def enable_motors(self, enable_pitch=True, enable_linear=True):
        """
        使能或禁用电机
        
        Args:
            enable_pitch: 是否使能YF俯仰电机
            enable_linear: 是否使能AImotor线性电机
        
        Returns:
            bool: 如果操作成功则返回 True
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
        
        success = True
        
        # 使能/禁用YF俯仰电机
        if enable_pitch != self.pitch_enabled:
            try:
                yf_cmd = [
                    self.YF_MODBUS_ADDR,  # 地址
                    self.YF_FUNC_WRITE,   # 功能码
                    0x08,                 # 命令长度
                    self.YF_CMD_ENABLE if enable_pitch else self.YF_CMD_DISABLE,  # 使能/禁用命令
                    0x00, 0x00, 0x00, 0x00,  # 位置 (不重要)
                    0x00,                 # 速度 (不重要)
                    0x00,                 # 预留
                    0x00                  # 预留
                ]
                
                crc = self._calculate_crc16(yf_cmd, len(yf_cmd))
                yf_cmd.append(crc & 0xFF)
                yf_cmd.append((crc >> 8) & 0xFF)
                
                # 避免串口过度占用，添加延时
                self._delay_if_needed()
                
                # 发送命令
                self.serial.write(bytearray(yf_cmd))
                time.sleep(0.05)
                
                # 读取响应
                if self.serial.in_waiting:
                    response = self.serial.read(self.serial.in_waiting)
                    if self.debug:
                        logger.debug(f"Received response: {' '.join(f'{b:02X}' for b in response)}")
                
                self.pitch_enabled = enable_pitch
                logger.info(f"YF pitch motor {'enabled' if enable_pitch else 'disabled'}")
            except Exception as e:
                logger.error(f"Error {'enabling' if enable_pitch else 'disabling'} YF pitch motor: {e}")
                success = False
        
        # 使能/禁用AImotor线性电机
        if enable_linear != self.linear_enabled:
            try:
                # AIMotor通过设置寄存器来使能/禁用
                ai_cmd = [
                    self.AI_MODBUS_ADDR,  # 地址
                    self.AI_WRITE_REG,    # 功能码: 写寄存器
                    0x03,                 # 寄存器地址高: 使能寄存器
                    0x03,                 # 寄存器地址低
                    0x00,                 # 数据高字节
                    0x01 if enable_linear else 0x00  # 数据低字节: 使能/禁用
                ]
                
                crc = self._calculate_crc16(ai_cmd, len(ai_cmd))
                ai_cmd.append(crc & 0xFF)
                ai_cmd.append((crc >> 8) & 0xFF)
                
                # 避免串口过度占用，添加延时
                self._delay_if_needed()
                
                # 发送命令
                self.serial.write(bytearray(ai_cmd))
                time.sleep(0.05)
                
                # 读取响应
                if self.serial.in_waiting:
                    response = self.serial.read(self.serial.in_waiting)
                    if self.debug:
                        logger.debug(f"Received response: {' '.join(f'{b:02X}' for b in response)}")
                
                self.linear_enabled = enable_linear
                logger.info(f"AImotor linear motor {'enabled' if enable_linear else 'disabled'}")
            except Exception as e:
                logger.error(f"Error {'enabling' if enable_linear else 'disabling'} AImotor linear motor: {e}")
                success = False
        
        return success
    
    def get_status(self):
        """
        Get the status of the motors
        
        Returns:
            dict: Motor status information or None if failed
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return None
        
        # 获取YF和AImotor电机位置
        pitch_pos, linear_pos = self.get_motor_positions()
        
        status = {
            'pitch_position': pitch_pos if pitch_pos is not None else self.pitch_position,
            'linear_position': linear_pos if linear_pos is not None else self.linear_position,
            'pitch_speed': self.pitch_speed,
            'linear_speed': self.linear_speed,
            'pitch_enabled': self.pitch_enabled,
            'linear_enabled': self.linear_enabled,
            'connected': self.is_connected()
        }
        
        return status
    
    def get_motor_positions(self):
        """
        Get the current position of the motors
        
        Returns:
            tuple: (pitch_position, linear_position) or (None, None) if failed
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return None, None
        
        pitch_pos = None
        linear_pos = None
        
        # 获取YF电机位置
        try:
            yf_cmd = [
                self.YF_MODBUS_ADDR,  # 地址
                0x02,                 # 读取指令
                0x00, 0x00, 0x00, 0x00  # 预留
            ]
            
            crc = self._calculate_crc16(yf_cmd, len(yf_cmd))
            yf_cmd.append(crc & 0xFF)
            yf_cmd.append((crc >> 8) & 0xFF)
            
            # 避免串口过度占用，添加延时
            self._delay_if_needed()
            
            # 清空接收缓冲区
            self.serial.reset_input_buffer()
            
            # 发送命令
            self.serial.write(bytearray(yf_cmd))
            time.sleep(0.1)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting >= 8:  # 预期响应长度
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received YF response: {' '.join(f'{b:02X}' for b in response)}")
                
                # 解析位置数据
                if len(response) >= 8 and response[0] == self.YF_MODBUS_ADDR:
                    pitch_pos = (response[3] << 24) | (response[4] << 16) | (response[5] << 8) | response[6]
                    if pitch_pos > 0x7FFFFFFF:  # 处理负数
                        pitch_pos = pitch_pos - 0x100000000
                    self.pitch_position = pitch_pos
            else:
                logger.warning("No or incomplete response from YF motor")
        
        except Exception as e:
            logger.error(f"Error getting YF pitch position: {e}")
        
        # 获取AIMotor电机位置
        try:
            ai_cmd = [
                self.AI_MODBUS_ADDR,  # 地址
                self.AI_READ_REG,     # 功能码: 读寄存器
                0x0B,                 # 寄存器地址高: 位置反馈寄存器
                0x07,                 # 寄存器地址低
                0x00,                 # 读取数量高字节
                0x02                  # 读取数量低字节: 2个寄存器
            ]
            
            crc = self._calculate_crc16(ai_cmd, len(ai_cmd))
            ai_cmd.append(crc & 0xFF)
            ai_cmd.append((crc >> 8) & 0xFF)
            
            # 避免串口过度占用，添加延时
            self._delay_if_needed()
            
            # 清空接收缓冲区
            self.serial.reset_input_buffer()
            
            # 发送命令
            self.serial.write(bytearray(ai_cmd))
            time.sleep(0.1)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting >= 7:  # 预期响应长度
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received AIMotor response: {' '.join(f'{b:02X}' for b in response)}")
                
                # 解析位置数据
                if len(response) >= 7 and response[0] == self.AI_MODBUS_ADDR and response[1] == self.AI_READ_REG:
                    linear_pos = (response[3] << 8) | response[4]
                    if response[2] == 4:  # 如果数据长度为4，说明是32位整数
                        linear_pos = (linear_pos << 16) | (response[5] << 8) | response[6]
                    self.linear_position = linear_pos
            else:
                logger.warning("No or incomplete response from AIMotor")
        
        except Exception as e:
            logger.error(f"Error getting AIMotor linear position: {e}")
        
        return pitch_pos, linear_pos
    
    def home_motors(self):
        """
        Home both motors (move to zero position)
        
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
        
        # 使能两个电机
        if not self.enable_motors(True, True):
            logger.error("Failed to enable motors for homing")
            return False
        
        # 将两个电机移动到零位置
        pitch_success = self.set_pitch_position(0)
        linear_success = self.set_linear_position(0)
        
        return pitch_success and linear_success
    
    def test_communication(self):
        """
        Test communication with both motors
        
        Returns:
            dict: Test results for each motor
        """
        results = {
            "yf_pitch": False,
            "ai_linear": False,
            "yf_debug_info": "",
            "ai_debug_info": "",
            "errors": []
        }
        
        if not self.is_connected():
            error_msg = "Not connected to motor controller"
            logger.error(error_msg)
            results["errors"].append(error_msg)
            return results
        
        # 测试YF电机通信
        try:
            # 尝试几种不同可能的命令格式，以增加成功的机会
            yf_cmds = [
                # 常规查询命令
                [
                    self.YF_MODBUS_ADDR,  # 地址
                    0x02,                 # 读取指令
                    0x00, 0x00, 0x00, 0x00  # 预留
                ],
                # 备用命令1 - 使用原始协议
                [
                    self.YF_HEADER, self.YF_ADDR, self.CMD_GET_STATUS, 0x00
                ],
                # 备用命令2 - 使用不同地址
                [
                    0x01,                 # 备用地址
                    0x03,                 # 读取保持寄存器
                    0x00, 0x00,           # 起始地址
                    0x00, 0x01            # 寄存器数量
                ]
            ]
            
            logger.info("Testing YF pitch motor communication...")
            yf_response = None
            
            for i, cmd_data in enumerate(yf_cmds):
                # 添加CRC校验码
                if i == 0:  # 第一个命令使用标准Modbus CRC
                    crc = self._calculate_crc16(cmd_data, len(cmd_data))
                    cmd_data.append(crc & 0xFF)
                    cmd_data.append((crc >> 8) & 0xFF)
                elif i == 1:  # 第二个命令使用简单校验和
                    checksum = sum(cmd_data) & 0xFF
                    cmd_data.append(checksum)
                elif i == 2:  # 第三个命令使用标准Modbus CRC
                    crc = self._calculate_crc16(cmd_data, len(cmd_data))
                    cmd_data.append(crc & 0xFF)
                    cmd_data.append((crc >> 8) & 0xFF)
                
                cmd = bytearray(cmd_data)
                logger.debug(f"YF test {i+1}: Sending command: {' '.join(f'{b:02X}' for b in cmd)}")
                
                # 清空接收缓冲区
                self.serial.reset_input_buffer()
                
                # 发送命令
                self.serial.write(cmd)
                time.sleep(0.2)  # 等待响应
                
                # 读取响应
                if self.serial.in_waiting > 0:
                    yf_response = self.serial.read(self.serial.in_waiting)
                    response_hex = ' '.join(f'{b:02X}' for b in yf_response)
                    logger.debug(f"YF test {i+1}: Received response: {response_hex}")
                    results["yf_debug_info"] += f"Test {i+1} response: {response_hex}\n"
                    
                    # 如果有响应，认为通信成功
                    results["yf_pitch"] = True
                    logger.info(f"YF pitch motor communication test {i+1}: SUCCESS")
                    break
                else:
                    logger.debug(f"YF test {i+1}: No response")
                    results["yf_debug_info"] += f"Test {i+1}: No response\n"
            
            if not results["yf_pitch"]:
                logger.warning("YF pitch motor communication test: FAILED (no response from any test)")
        
        except Exception as e:
            error_msg = f"Error testing YF pitch motor communication: {e}"
            logger.error(error_msg)
            results["errors"].append(error_msg)
            results["yf_debug_info"] += f"Exception: {e}\n"
        
        # 测试AIMotor电机通信
        try:
            # 尝试几种不同可能的命令格式，以增加成功的机会
            ai_cmds = [
                # 常规查询命令
                [
                    self.AI_MODBUS_ADDR,  # 地址
                    self.AI_READ_REG,     # 功能码: 读寄存器
                    0x0B,                 # 寄存器地址高: 位置反馈寄存器
                    0x07,                 # 寄存器地址低
                    0x00,                 # 读取数量高字节
                    0x02                  # 读取数量低字节: 2个寄存器
                ],
                # 备用命令1 - 读取不同寄存器
                [
                    self.AI_MODBUS_ADDR,  # 地址
                    self.AI_READ_REG,     # 功能码: 读寄存器
                    0x00,                 # 寄存器地址高
                    0x00,                 # 寄存器地址低
                    0x00,                 # 读取数量高字节
                    0x01                  # 读取数量低字节: 1个寄存器
                ],
                # 备用命令2 - 使用原始协议
                [
                    self.AI_HEADER, self.AI_ADDR, self.CMD_GET_STATUS, 0x00
                ]
            ]
            
            logger.info("Testing AIMotor linear motor communication...")
            ai_response = None
            
            for i, cmd_data in enumerate(ai_cmds):
                # 添加CRC校验码
                if i <= 1:  # 前两个命令使用标准Modbus CRC
                    crc = self._calculate_crc16(cmd_data, len(cmd_data))
                    cmd_data.append(crc & 0xFF)
                    cmd_data.append((crc >> 8) & 0xFF)
                elif i == 2:  # 第三个命令使用简单校验和
                    checksum = sum(cmd_data) & 0xFF
                    cmd_data.append(checksum)
                
                cmd = bytearray(cmd_data)
                logger.debug(f"AIMotor test {i+1}: Sending command: {' '.join(f'{b:02X}' for b in cmd)}")
                
                # 清空接收缓冲区
                self.serial.reset_input_buffer()
                
                # 发送命令
                self.serial.write(cmd)
                time.sleep(0.2)  # 等待响应
                
                # 读取响应
                if self.serial.in_waiting > 0:
                    ai_response = self.serial.read(self.serial.in_waiting)
                    response_hex = ' '.join(f'{b:02X}' for b in ai_response)
                    logger.debug(f"AIMotor test {i+1}: Received response: {response_hex}")
                    results["ai_debug_info"] += f"Test {i+1} response: {response_hex}\n"
                    
                    # 如果有响应，认为通信成功
                    results["ai_linear"] = True
                    logger.info(f"AIMotor linear motor communication test {i+1}: SUCCESS")
                    break
                else:
                    logger.debug(f"AIMotor test {i+1}: No response")
                    results["ai_debug_info"] += f"Test {i+1}: No response\n"
            
            if not results["ai_linear"]:
                logger.warning("AIMotor linear motor communication test: FAILED (no response from any test)")
        
        except Exception as e:
            error_msg = f"Error testing AIMotor linear motor communication: {e}"
            logger.error(error_msg)
            results["errors"].append(error_msg)
            results["ai_debug_info"] += f"Exception: {e}\n"
        
        # 记录串口配置信息，帮助诊断
        results["port_info"] = {
            "port": self.port,
            "baudrate": self.baudrate,
            "timeout": self.timeout,
            "connected": self.is_connected()
        }
        
        return results 
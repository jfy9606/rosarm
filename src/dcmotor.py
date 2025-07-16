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
            # 使用YF系列协议发送命令
            # 帧格式: 帧头(1B) + 地址(1B) + 命令(1B) + 数据长度(1B) + 数据(nB) + 校验和(1B)
            cmd = self.CMD_SET_PITCH
            data = struct.pack("<i", int(position))  # 位置为4字节整数
            speed_data = struct.pack("<B", self.pitch_speed)  # 速度为1字节
            
            data_bytes = data + speed_data
            data_len = len(data_bytes)
            
            # 计算校验和
            checksum = (self.YF_ADDR + cmd + data_len + sum(data_bytes)) & 0xFF
            
            # 组装完整命令
            command = bytes([self.YF_HEADER, self.YF_ADDR, cmd, data_len]) + data_bytes + bytes([checksum])
            
            if self.debug:
                logger.debug(f"Sending YF pitch command: {command.hex()}")
            
            # 发送命令
            self.serial.write(command)
            
            # 等待响应
            time.sleep(0.05)
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received response: {response.hex()}")
                
                # 简单检查响应是否有效
                if len(response) >= 3 and response[0] == self.YF_HEADER:
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
            # 使用AIMotor系列协议发送命令
            # 帧格式: 帧头(1B) + 地址(1B) + 命令(1B) + 数据长度(1B) + 数据(nB) + 校验和(1B)
            cmd = self.CMD_SET_LINEAR
            data = struct.pack("<i", int(position))  # 位置为4字节整数
            speed_data = struct.pack("<B", self.linear_speed)  # 速度为1字节
            
            data_bytes = data + speed_data
            data_len = len(data_bytes)
            
            # 计算校验和
            checksum = (self.AI_ADDR + cmd + data_len + sum(data_bytes)) & 0xFF
            
            # 组装完整命令
            command = bytes([self.AI_HEADER, self.AI_ADDR, cmd, data_len]) + data_bytes + bytes([checksum])
            
            if self.debug:
                logger.debug(f"Sending AIMotor linear command: {command.hex()}")
            
            # 发送命令
            self.serial.write(command)
            
            # 等待响应
            time.sleep(0.05)
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received response: {response.hex()}")
                
                # 简单检查响应是否有效
                if len(response) >= 3 and response[0] == self.AI_HEADER:
                    self.linear_position = position
                    return True
            
            logger.warning("No valid response from AIMotor")
            return False
            
        except Exception as e:
            logger.error(f"Error sending command to AIMotor linear motor: {e}")
            return False
    
    def _send_stop_command(self):
        """
        Send stop command to all motors
        
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
        
        try:
            # 发送YF电机停止命令
            yf_cmd = bytes([self.YF_HEADER, self.YF_ADDR, self.CMD_STOP_ALL, 0, self.YF_ADDR + self.CMD_STOP_ALL])
            self.serial.write(yf_cmd)
            time.sleep(0.05)
            
            # 发送AIMotor电机停止命令
            ai_cmd = bytes([self.AI_HEADER, self.AI_ADDR, self.CMD_STOP_ALL, 0, self.AI_ADDR + self.CMD_STOP_ALL])
            self.serial.write(ai_cmd)
            time.sleep(0.05)
            
            # 清空接收缓冲区
            if self.serial.in_waiting:
                self.serial.read(self.serial.in_waiting)
            
            logger.info("Stop command sent to all motors")
            return True
            
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
            
            try:
                # 发送YF电机速度命令
                data = struct.pack("<B", self.pitch_speed)
                data_len = len(data)
                checksum = (self.YF_ADDR + self.CMD_SET_SPEED + data_len + sum(data)) & 0xFF
                command = bytes([self.YF_HEADER, self.YF_ADDR, self.CMD_SET_SPEED, data_len]) + data + bytes([checksum])
                
                if self.debug:
                    logger.debug(f"Setting YF pitch speed to {pitch_speed}: {command.hex()}")
                
                self.serial.write(command)
                time.sleep(0.05)
                
                # 读取响应
                if self.serial.in_waiting:
                    response = self.serial.read(self.serial.in_waiting)
                    if self.debug:
                        logger.debug(f"Received response: {response.hex()}")
                    
                    # 检查响应
                    if not (len(response) >= 3 and response[0] == self.YF_HEADER):
                        logger.warning("Invalid response from YF motor for speed command")
                        success = False
                
            except Exception as e:
                logger.error(f"Error setting YF pitch speed: {e}")
                success = False
        
        # 设置AIMotor电机速度
        if linear_speed is not None:
            self.linear_speed = min(255, max(0, linear_speed))
            
            try:
                # 发送AIMotor电机速度命令
                data = struct.pack("<B", self.linear_speed)
                data_len = len(data)
                checksum = (self.AI_ADDR + self.CMD_SET_SPEED + data_len + sum(data)) & 0xFF
                command = bytes([self.AI_HEADER, self.AI_ADDR, self.CMD_SET_SPEED, data_len]) + data + bytes([checksum])
                
                if self.debug:
                    logger.debug(f"Setting AIMotor linear speed to {linear_speed}: {command.hex()}")
                
                self.serial.write(command)
                time.sleep(0.05)
                
                # 读取响应
                if self.serial.in_waiting:
                    response = self.serial.read(self.serial.in_waiting)
                    if self.debug:
                        logger.debug(f"Received response: {response.hex()}")
                    
                    # 检查响应
                    if not (len(response) >= 3 and response[0] == self.AI_HEADER):
                        logger.warning("Invalid response from AIMotor for speed command")
                        success = False
                
            except Exception as e:
                logger.error(f"Error setting AIMotor linear speed: {e}")
                success = False
        
        return success
    
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
            # 发送YF电机位置查询命令
            checksum = (self.YF_ADDR + self.CMD_GET_STATUS) & 0xFF
            command = bytes([self.YF_HEADER, self.YF_ADDR, self.CMD_GET_STATUS, 0, checksum])
            
            if self.debug:
                logger.debug(f"Querying YF pitch position: {command.hex()}")
            
            # 清空接收缓冲区
            self.serial.reset_input_buffer()
            
            # 发送命令
            self.serial.write(command)
            time.sleep(0.1)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting >= 7:  # 预期响应长度
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received YF response: {response.hex()}")
                
                # 解析位置数据
                if len(response) >= 7 and response[0] == self.YF_HEADER and response[2] == self.CMD_GET_STATUS:
                    pitch_pos = struct.unpack("<i", response[3:7])[0]
                    self.pitch_position = pitch_pos
            else:
                logger.warning("No or incomplete response from YF motor")
        
        except Exception as e:
            logger.error(f"Error getting YF pitch position: {e}")
        
        # 获取AIMotor电机位置
        try:
            # 发送AIMotor电机位置查询命令
            checksum = (self.AI_ADDR + self.CMD_GET_STATUS) & 0xFF
            command = bytes([self.AI_HEADER, self.AI_ADDR, self.CMD_GET_STATUS, 0, checksum])
            
            if self.debug:
                logger.debug(f"Querying AIMotor linear position: {command.hex()}")
            
            # 清空接收缓冲区
            self.serial.reset_input_buffer()
            
            # 发送命令
            self.serial.write(command)
            time.sleep(0.1)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting >= 7:  # 预期响应长度
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received AIMotor response: {response.hex()}")
                
                # 解析位置数据
                if len(response) >= 7 and response[0] == self.AI_HEADER and response[2] == self.CMD_GET_STATUS:
                    linear_pos = struct.unpack("<i", response[3:7])[0]
                    self.linear_position = linear_pos
            else:
                logger.warning("No or incomplete response from AIMotor")
        
        except Exception as e:
            logger.error(f"Error getting AIMotor linear position: {e}")
        
        return pitch_pos, linear_pos
    
    def home(self):
        """
        Home both motors (move to zero position)
        
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
        
        success = True
        
        # 发送YF电机回零命令
        try:
            checksum = (self.YF_ADDR + self.CMD_HOME) & 0xFF
            command = bytes([self.YF_HEADER, self.YF_ADDR, self.CMD_HOME, 0, checksum])
            
            if self.debug:
                logger.debug(f"Sending YF home command: {command.hex()}")
            
            self.serial.write(command)
            time.sleep(0.05)
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received response: {response.hex()}")
                
                # 检查响应
                if not (len(response) >= 3 and response[0] == self.YF_HEADER):
                    logger.warning("Invalid response from YF motor for home command")
                    success = False
            
        except Exception as e:
            logger.error(f"Error sending home command to YF motor: {e}")
            success = False
        
        # 发送AIMotor电机回零命令
        try:
            checksum = (self.AI_ADDR + self.CMD_HOME) & 0xFF
            command = bytes([self.AI_HEADER, self.AI_ADDR, self.CMD_HOME, 0, checksum])
            
            if self.debug:
                logger.debug(f"Sending AIMotor home command: {command.hex()}")
            
            self.serial.write(command)
            time.sleep(0.05)
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received response: {response.hex()}")
                
                # 检查响应
                if not (len(response) >= 3 and response[0] == self.AI_HEADER):
                    logger.warning("Invalid response from AIMotor for home command")
                    success = False
            
        except Exception as e:
            logger.error(f"Error sending home command to AIMotor: {e}")
            success = False
        
        return success
    
    def test_communication(self):
        """
        Test communication with both motors
        
        Returns:
            dict: Test results for each motor
        """
        results = {
            "yf_pitch": False,
            "ai_linear": False
        }
        
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return results
        
        # 测试YF电机通信
        try:
            # 发送YF电机状态查询命令
            checksum = (self.YF_ADDR + self.CMD_GET_STATUS) & 0xFF
            command = bytes([self.YF_HEADER, self.YF_ADDR, self.CMD_GET_STATUS, 0, checksum])
            
            logger.info("Testing YF pitch motor communication...")
            logger.debug(f"Sending command: {command.hex()}")
            
            # 清空接收缓冲区
            self.serial.reset_input_buffer()
            
            # 发送命令
            self.serial.write(command)
            time.sleep(0.2)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                logger.debug(f"Received response: {response.hex()}")
                
                # 检查响应
                if len(response) >= 3 and response[0] == self.YF_HEADER:
                    results["yf_pitch"] = True
                    logger.info("YF pitch motor communication test: SUCCESS")
                else:
                    logger.warning("YF pitch motor communication test: FAILED (invalid response)")
            else:
                logger.warning("YF pitch motor communication test: FAILED (no response)")
        
        except Exception as e:
            logger.error(f"Error testing YF pitch motor communication: {e}")
        
        # 测试AIMotor电机通信
        try:
            # 发送AIMotor电机状态查询命令
            checksum = (self.AI_ADDR + self.CMD_GET_STATUS) & 0xFF
            command = bytes([self.AI_HEADER, self.AI_ADDR, self.CMD_GET_STATUS, 0, checksum])
            
            logger.info("Testing AIMotor linear motor communication...")
            logger.debug(f"Sending command: {command.hex()}")
            
            # 清空接收缓冲区
            self.serial.reset_input_buffer()
            
            # 发送命令
            self.serial.write(command)
            time.sleep(0.2)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                logger.debug(f"Received response: {response.hex()}")
                
                # 检查响应
                if len(response) >= 3 and response[0] == self.AI_HEADER:
                    results["ai_linear"] = True
                    logger.info("AIMotor linear motor communication test: SUCCESS")
                else:
                    logger.warning("AIMotor linear motor communication test: FAILED (invalid response)")
            else:
                logger.warning("AIMotor linear motor communication test: FAILED (no response)")
        
        except Exception as e:
            logger.error(f"Error testing AIMotor linear motor communication: {e}")
        
        return results 
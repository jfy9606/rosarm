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
    
    # 命令码 - 基于SimpleNetwork实现
    CMD_SET_PITCH = 0x08    # YF俯仰电机命令
    CMD_SET_LINEAR = 0x06   # AImotor进给电机命令
    CMD_STOP_ALL = 0x03     # 停止所有电机
    CMD_GET_STATUS = 0x03   # 获取状态
    CMD_SET_SPEED = 0x05    # 设置速度
    CMD_SET_ACCEL = 0x06    # 设置加速度
    CMD_HOME = 0x07         # 回零点
    CMD_ENABLE = 0x81       # 使能命令
    CMD_DISABLE = 0x80      # 禁用命令
    
    # YF电机指令格式
    YF_START = 0x3E       # YF帧起始字节
    YF_FUNC = 0x01        # YF功能字节
    YF_STATION = 0x01     # YF站号/地址
    
    # AImotor电机指令格式 (ModBus RTU)
    AI_STATION = 0x03     # AIMotor站号/地址
    AI_FUNC_WRITE = 0x06  # AIMotor写寄存器功能码
    AI_FUNC_READ = 0x03   # AIMotor读寄存器功能码
    
    # AImotor寄存器地址
    AI_REG_CTRL_MODE = 0x0200   # 控制模式寄存器
    AI_REG_ENABLE = 0x0303      # 使能寄存器
    AI_REG_SPEED = 0x0603       # 速度寄存器 
    AI_REG_POSITION = 0x0500    # 位置寄存器
    AI_REG_POS_MODE = 0x1104    # 位置模式寄存器
    AI_REG_POS_SETTING = 0x110C # 位置设置寄存器
    AI_REG_POS_DATA = 0x0B07    # 位置数据寄存器
    
    # 从SimpleNetwork示例中提取的代码列表
    CODE_LIST = [
        [0x06,0x03,0x02,0x00,0x01],
        [0x06,0x03,0x03,0x00,0x01],  # 使能开
        [0x06,0x03,0x03,0x00,0x00],  # 使能关
        [0x06,0x02,0x00,0x00,0x00],  # 控制模式：速度控制
        [0x06,0x02,0x00,0x00,0x01],  # 控制模式：位移控制
        [0x06,0x06,0x02,0x00,0x00],  # 速度指令来源：内部
        [0x06,0x03,0x04,0x00,0x1c],
        [0x06,0x03,0x05,0x00,0x01],  # 位移使能开
        [0x06,0x03,0x05,0x00,0x00],  # 位移使能关
        [0x06,0x04,0x00,0x00,0x05],
        [0x06,0x04,0x01,0x00,0x00],
        [0x06,0x05,0x00,0x00,0x02],  # 位置指令来源：内部
        [0x06,0x11,0x00,0x00,0x00],  # 位置运行方式：单次
        [0x06,0x11,0x01,0x00,0x01],  # 位置段数：1段
        [0x06,0x11,0x04,0x00,0x01],  # 绝对位移模式
        [0x06,0x11,0x04,0x00,0x00],  # 相对位移模式
        [0x03,0x0b,0x00,0x00,0x01],
        [0x03,0x0b,0x07,0x00,0x02],  # 读取位置
        [0x03,0x0b,0x03,0x00,0x01],
        [0x03,0x0b,0x05,0x00,0x01],
        [0x03,0x0b,0x18,0x00,0x01],
        [0x03,0x0b,0x1a,0x00,0x01],
        [0x03,0x0b,0x1b,0x00,0x01]
    ]
    
    # 从SimpleNetwork示例中提取的电机命令
    MOTOR_LIST = [
        [0x3E,0x01,0x08,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00],  # 使能命令
        [0x3E,0x01,0x08,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00]   # 关闭命令
    ]
    
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
        
        # 控制模式
        self.linear_mode = 0  # 0=位置模式, 1=速度模式

        # 当前form状态 (参考SimpleNetwork)
        self.form = -1  # -1=未初始化, 0=位置控制, 1=速度控制
        
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
    
    def _modbus_crc16(self, data):
        """
        Calculate Modbus CRC-16 for AIMotor commands
        
        Args:
            data: Data bytes for CRC calculation
            
        Returns:
            int: CRC-16 value
        """
        crc = 0xFFFF
        
        for i in range(len(data)):
            crc ^= data[i]
            
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        
        return crc
    
    def _send_ai_command(self, data):
        """
        发送命令到AIMotor进给电机，添加CRC校验

        Args:
            data: 命令数据，不包含CRC

        Returns:
            bytes: 收到的响应
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return None
            
        try:
            # 添加ModBus CRC-16校验
            crc = self._modbus_crc16(data)
            cmd = bytearray(data + [crc & 0xFF, (crc >> 8) & 0xFF])
            
            # 记录命令
            if self.debug:
                logger.debug(f"AIMotor command: {' '.join(f'{b:02X}' for b in cmd)}")
            
            # 发送命令
            self.serial.write(cmd)
            time.sleep(0.05)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"AIMotor response: {' '.join(f'{b:02X}' for b in response)}")
                return response
            
            return None
        except Exception as e:
            logger.error(f"Error sending command to AIMotor: {e}")
            return None
    
    def _send_yf_command(self, data):
        """
        发送命令到YF俯仰电机

        Args:
            data: 命令数据，需包含完整的命令(YF_START,...)

        Returns:
            bytes: 收到的响应
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return None
            
        try:
            # 发送命令
            if self.debug:
                logger.debug(f"YF command: {' '.join(f'{b:02X}' for b in data)}")
                
            self.serial.write(data)
            time.sleep(0.05)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"YF response: {' '.join(f'{b:02X}' for b in response)}")
                return response
                
            return None
        except Exception as e:
            logger.error(f"Error sending command to YF motor: {e}")
            return None
    
    def _delay_if_needed(self):
        """添加必要的延时以避免串口通信过于频繁"""
        current_time = time.time()
        if current_time - self.last_comm_time < 0.02:  # 确保命令间隔至少20ms
            time.sleep(0.02 - (current_time - self.last_comm_time))
        self.last_comm_time = time.time()
    
    def set_form(self, station_num, form_input):
        """
        设置电机控制模式 (复制SimpleNetwork实现)
        
        Args:
            station_num: 电机站号
            form_input: 控制模式 (0=位置控制, 1=速度控制)
            
        Returns:
            bool: 操作是否成功
        """
        if not self.is_connected():
            logger.error("Not connected to motor controller")
            return False
            
        if form_input == self.form:
            return True
            
        try:
            code_order = []
            
            if form_input == 1:  # 速度控制模式
                code_order = [3, 0, 2, 5, 6, 8]
            elif form_input == 0:  # 位置控制模式
                code_order = [4, 0, 2, 6, 8, 9, 10, 11, 12, 13]
            else:
                logger.error(f"Invalid form input: {form_input}")
                return False
                
            # 发送命令序列
            for i in code_order:
                self._delay_if_needed()
                cmd_data = self.CODE_LIST[i].copy()
                cmd_data.insert(0, station_num)
                crc = self._modbus_crc16(cmd_data)
                cmd_data.append(crc & 0xFF)
                cmd_data.append((crc >> 8) & 0xFF)
                
                self.serial.write(bytearray(cmd_data))
                time.sleep(0.02)
                
                # 清空接收缓冲区
                if self.serial.in_waiting:
                    self.serial.read(self.serial.in_waiting)
                    
            logger.info(f"Set form to {form_input} (0=position, 1=velocity)")
            self.form = form_input
            return True
            
        except Exception as e:
            logger.error(f"Error setting form: {e}")
            return False
    
    def set_pitch_position(self, position, speed=None):
        """
        Set the position of the YF pitch motor (using SimpleNetwork format)
        
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
            # 基于SimpleNetwork中MOTOR_LIST[0]的格式
            cmd_data = bytearray([
                self.YF_START,      # 起始字节 0x3E
                self.YF_FUNC,       # 功能字节 0x01
                self.CMD_SET_PITCH, # 命令字节 0x08
                self.CMD_ENABLE,    # 使能字节 0x81 (包含位置控制)
                # 位置数据 (4字节)
                (position >> 24) & 0xFF,
                (position >> 16) & 0xFF,
                (position >> 8) & 0xFF,
                position & 0xFF,
                # 速度 (1字节)
                self.pitch_speed & 0xFF,
                # 预留字节
                0x00,
                0x00
            ])
            
            # 避免串口过度占用，添加延时
            self._delay_if_needed()
            
            # 发送命令
            if self.debug:
                logger.debug(f"Setting YF pitch position to {position}: {' '.join(f'{b:02X}' for b in cmd_data)}")
                
            self.serial.write(cmd_data)
            time.sleep(0.1)  # 等待响应
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                if self.debug:
                    logger.debug(f"Received response: {' '.join(f'{b:02X}' for b in response)}")
            
            # 更新位置值
            self.pitch_position = position
            return True
            
        except Exception as e:
            logger.error(f"Error sending command to YF pitch motor: {e}")
            return False
    
    def set_linear_position(self, position, speed=None):
        """
        Set the position of the AIMotor linear motor (using SimpleNetwork format)
        
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
            # 确保使用位置控制模式
            if self.form != 0:
                self.set_form(self.AI_STATION, 0)
            
            # 先设置绝对位移模式
            abs_mode_cmd = self.CODE_LIST[14].copy()  # 绝对位移模式
            abs_mode_cmd.insert(0, self.AI_STATION)
            abs_mode_response = self._send_ai_command(abs_mode_cmd)
            
            # 设置位置阈值
            pos_thr = 10  # 位置阈值，参考SimpleNetwork
            thr_cmd = [
                self.AI_STATION,
                0x06,  # 写寄存器
                0x05, 0x15,  # 寄存器地址
                (pos_thr >> 8) & 0xFF, pos_thr & 0xFF  # 位置阈值数据
            ]
            thr_response = self._send_ai_command(thr_cmd)
            
            # 计算位置高低字
            pos_low = position & 0xFFFF
            pos_high = (position >> 16) & 0xFFFF
            if pos_high == 0 and position < 0:
                pos_high = 0xFFFF
                
            # 设置位置、速度和加速度
            pos_cmd = [
                self.AI_STATION,
                0x10,  # 写多个寄存器
                0x11, 0x0C,  # 寄存器地址
                0x00, 0x04,  # 寄存器数量
                0x08,  # 字节数
                (pos_low >> 8) & 0xFF, pos_low & 0xFF,  # 位置低字
                (pos_high >> 8) & 0xFF, pos_high & 0xFF,  # 位置高字
                (self.linear_speed >> 8) & 0xFF, self.linear_speed & 0xFF,  # 速度
                0x00, 0x00  # 加速度 (默认)
            ]
            pos_response = self._send_ai_command(pos_cmd)
            
            # 更新位置值
            self.linear_position = position
            
            if self.debug:
                logger.debug(f"Set AIMotor linear position to {position}, speed {self.linear_speed}")
                
            return True
            
        except Exception as e:
            logger.error(f"Error setting AIMotor linear position: {e}")
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
            # 停止YF电机 - 使用禁用命令
            yf_cmd = bytearray([
                self.YF_START,      # 起始字节 0x3E
                self.YF_FUNC,       # 功能字节 0x01
                self.CMD_SET_PITCH, # 命令字节 0x08
                self.CMD_DISABLE,   # 禁用字节 0x80
                0x00, 0x00, 0x00, 0x00,  # 位置 (不重要)
                0x00,               # 速度 (不重要)
                0x00, 0x00          # 预留字节
            ])
            
            self._delay_if_needed()
            self.serial.write(yf_cmd)
            time.sleep(0.05)
            
            # 停止AIMotor电机 - 禁用电机
            ai_cmd = [
                self.AI_STATION,
                self.AI_FUNC_WRITE,
                0x03, 0x03,  # 禁用寄存器地址
                0x00, 0x00   # 禁用值
            ]
            self._delay_if_needed()
            self._send_ai_command(ai_cmd)
            
            # 清空接收缓冲区
            if self.serial.in_waiting:
                self.serial.read(self.serial.in_waiting)
            
            logger.info("Stop command sent to all motors")
            
            # 更新使能状态
            self.pitch_enabled = False
            self.linear_enabled = False
            
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
        
        # 设置YF电机速度 - YF电机速度通过位置命令一起设置
        if pitch_speed is not None:
            self.pitch_speed = min(255, max(0, pitch_speed))
            logger.info(f"Set YF pitch speed to {self.pitch_speed}")
        
        # 设置AIMotor电机速度
        if linear_speed is not None:
            self.linear_speed = min(255, max(0, linear_speed))
            try:
                # 确保使用速度控制模式
                if self.linear_mode == 0:  # 若为位置模式
                    # 直接更新速度值而不发送命令，下次位置命令会带上新的速度
                    pass
                else:  # 速度模式
                    # 设置速度寄存器
                    speed_cmd = [
                        self.AI_STATION,
                        self.AI_FUNC_WRITE,
                        0x06, 0x03,  # 速度寄存器地址
                        (self.linear_speed >> 8) & 0xFF, self.linear_speed & 0xFF
                    ]
                    self._delay_if_needed()
                    self._send_ai_command(speed_cmd)
                
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
                # 使用motor_list中的命令格式
                cmd_data = bytearray(self.MOTOR_LIST[0 if enable_pitch else 1])
                
                self._delay_if_needed()
                
                # 发送命令
                self.serial.write(cmd_data)
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
                # 发送使能命令 (参照SimpleNetwork中的enable_on/enable_off方法)
                if enable_linear:
                    # 使能开
                    self._send_ai_command([self.AI_STATION] + self.CODE_LIST[1])
                    time.sleep(0.02)
                    # 位移使能开
                    self._send_ai_command([self.AI_STATION] + self.CODE_LIST[7])
                else:
                    # 使能关
                    self._send_ai_command([self.AI_STATION] + self.CODE_LIST[2])
                    time.sleep(0.02)
                    # 位移使能关
                    self._send_ai_command([self.AI_STATION] + self.CODE_LIST[8])
                
                self.linear_enabled = enable_linear
                logger.info(f"AImotor linear motor {'enabled' if enable_linear else 'disabled'}")
            except Exception as e:
                logger.error(f"Error {'enabling' if enable_linear else 'disabling'} AImotor linear motor: {e}")
                success = False
        
        return success
    
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
        
        # 获取AIMotor电机位置 - 从SimpleNetwork示例中可以看到使用CODE_LIST[17]
        try:
            read_pos_cmd = [self.AI_STATION] + self.CODE_LIST[17]  # 读取位置命令
            self._delay_if_needed()
            response = self._send_ai_command(read_pos_cmd)
            
            if response and len(response) >= 7:
                linear_pos = (response[3] << 8) | response[4]
                # 处理32位位置值
                if len(response) >= 9:
                    linear_pos = (linear_pos << 16) | (response[5] << 8) | response[6]
                self.linear_position = linear_pos
                logger.debug(f"Read AIMotor position: {linear_pos}")
        except Exception as e:
            logger.error(f"Error reading AIMotor position: {e}")
        
        return pitch_pos, linear_pos
    
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
        
        # 测试YF电机通信 - 使用MOTOR_LIST中的命令格式
        try:
            # 使用使能命令作为测试
            cmd_data = bytearray(self.MOTOR_LIST[0])  # 使能命令
            
            logger.info("Testing YF pitch motor communication...")
            logger.debug(f"Sending command: {' '.join(f'{b:02X}' for b in cmd_data)}")
            
            # 清空接收缓冲区
            self.serial.reset_input_buffer()
            
            # 发送命令
            self.serial.write(cmd_data)
            time.sleep(0.2)
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                response_hex = ' '.join(f'{b:02X}' for b in response)
                logger.debug(f"Received response: {response_hex}")
                results["yf_debug_info"] += f"Response: {response_hex}\n"
                
                results["yf_pitch"] = True
                logger.info("YF pitch motor communication test: SUCCESS")
            else:
                logger.warning("YF pitch motor communication test: FAILED (no response)")
                # 尝试更多命令格式
                cmd_data = bytearray(self.MOTOR_LIST[1])  # 禁用命令
                logger.debug(f"Trying alternate command: {' '.join(f'{b:02X}' for b in cmd_data)}")
                
                self.serial.reset_input_buffer()
                self.serial.write(cmd_data)
                time.sleep(0.2)
                
                if self.serial.in_waiting:
                    response = self.serial.read(self.serial.in_waiting)
                    response_hex = ' '.join(f'{b:02X}' for b in response)
                    logger.debug(f"Received response: {response_hex}")
                    results["yf_debug_info"] += f"Alt response: {response_hex}\n"
                    
                    results["yf_pitch"] = True
                    logger.info("YF pitch motor communication test: SUCCESS with alternate command")
        
        except Exception as e:
            error_msg = f"Error testing YF pitch motor communication: {e}"
            logger.error(error_msg)
            results["errors"].append(error_msg)
        
        # 测试AIMotor电机通信 - 使用CODE_LIST中的读取位置命令
        try:
            read_pos_cmd = [self.AI_STATION] + self.CODE_LIST[17]  # 读取位置命令
            
            logger.info("Testing AIMotor linear motor communication...")
            logger.debug(f"Sending command: {' '.join(f'{b:02X}' for b in read_pos_cmd)}")
            
            # 添加CRC校验
            crc = self._modbus_crc16(read_pos_cmd)
            read_pos_cmd.append(crc & 0xFF)
            read_pos_cmd.append((crc >> 8) & 0xFF)
            
            # 清空接收缓冲区
            self.serial.reset_input_buffer()
            
            # 发送命令
            self.serial.write(bytearray(read_pos_cmd))
            time.sleep(0.2)
            
            # 读取响应
            if self.serial.in_waiting:
                response = self.serial.read(self.serial.in_waiting)
                response_hex = ' '.join(f'{b:02X}' for b in response)
                logger.debug(f"Received response: {response_hex}")
                results["ai_debug_info"] += f"Response: {response_hex}\n"
                
                results["ai_linear"] = True
                logger.info("AIMotor linear motor communication test: SUCCESS")
            else:
                logger.warning("AIMotor linear motor communication test: FAILED (no response)")
                # 尝试更多命令格式
                read_status_cmd = [self.AI_STATION] + self.CODE_LIST[16]  # 读取速度命令
                crc = self._modbus_crc16(read_status_cmd)
                read_status_cmd.append(crc & 0xFF)
                read_status_cmd.append((crc >> 8) & 0xFF)
                
                logger.debug(f"Trying alternate command: {' '.join(f'{b:02X}' for b in read_status_cmd)}")
                
                self.serial.reset_input_buffer()
                self.serial.write(bytearray(read_status_cmd))
                time.sleep(0.2)
                
                if self.serial.in_waiting:
                    response = self.serial.read(self.serial.in_waiting)
                    response_hex = ' '.join(f'{b:02X}' for b in response)
                    logger.debug(f"Received response: {response_hex}")
                    results["ai_debug_info"] += f"Alt response: {response_hex}\n"
                    
                    results["ai_linear"] = True
                    logger.info("AIMotor linear motor communication test: SUCCESS with alternate command")
        
        except Exception as e:
            error_msg = f"Error testing AIMotor linear motor communication: {e}"
            logger.error(error_msg)
            results["errors"].append(error_msg)
        
        # 记录串口配置信息，帮助诊断
        results["port_info"] = {
            "port": self.port,
            "baudrate": self.baudrate,
            "timeout": self.timeout,
            "connected": self.is_connected()
        }
        
        return results 
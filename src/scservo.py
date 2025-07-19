import serial
import struct
import time
import platform
import logging

# 设置日志
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('SCServo')

class SCServo:
    """
    SCServo communication class for controlling SCServo (FT) servos through serial port
    """
    
    # SCServo command codes
    CMD_PING = 0x01
    CMD_READ_DATA = 0x02
    CMD_WRITE_DATA = 0x03
    CMD_WRITE_SAVE = 0x04
    CMD_READ_MULTI = 0x05
    CMD_REG_WRITE = 0x06
    CMD_ACTION = 0x07
    CMD_SYNC_READ = 0x82
    CMD_SYNC_WRITE = 0x83
    
    # Control table addresses
    ADDR_TORQUE_ENABLE = 40
    ADDR_GOAL_POSITION = 42
    ADDR_GOAL_TIME = 44
    ADDR_GOAL_SPEED = 46
    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_SPEED = 58
    ADDR_PRESENT_LOAD = 60
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMPERATURE = 63
    
    # Error codes
    ERR_INSTRUCTION = 0x01
    ERR_OVERLOAD = 0x02
    ERR_CHECKSUM = 0x03
    ERR_RANGE = 0x04
    ERR_OVERHEAT = 0x05
    ERR_ANGLE_LIMIT = 0x06
    ERR_VOLTAGE = 0x07

    def __init__(self, port=None, baudrate=1000000, timeout=0.1, protocol_end=0):
        """
        Initialize SCServo communication
        
        Args:
            port: Serial port name. If None, needs to be connected later
            baudrate: Baud rate for serial communication
            timeout: Serial timeout in seconds
            protocol_end: Protocol end bit (0 for STS/SMS, 1 for SCS)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.protocol_end = protocol_end
        self._serial = None
        self.is_connected = False
        
        if port:
            self.connect(port, baudrate, timeout)
    
    def connect(self, port, baudrate=1000000, timeout=0.1):
        """
        Connect to the servo controller
        
        Args:
            port: Serial port name
            baudrate: Baud rate for serial communication
            timeout: Serial timeout in seconds
            
        Returns:
            bool: True if connection successful, False otherwise
        """
        # Handle different port naming conventions across platforms
        if platform.system() == "Windows":
            if not port.startswith("COM"):
                port = f"COM{port}" if port.isdigit() else port
        else:  # Linux/Mac
            if not port.startswith("/dev/"):
                port = f"/dev/{port}" if not "/" in port else port
        
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # 导入 Feetech SDK
            from scservo_sdk import PortHandler, PacketHandler
            
            # 创建 PortHandler 和 PacketHandler 实例
            self.portHandler = PortHandler(port)
            self.packetHandler = PacketHandler(self.protocol_end)
            
            # 打开端口
            if self.portHandler.openPort():
                logger.info(f"成功打开端口 {port}")
            else:
                logger.error(f"无法打开端口 {port}")
                return False
                
            # 设置波特率
            if self.portHandler.setBaudRate(baudrate):
                logger.info(f"成功设置波特率为 {baudrate}")
            else:
                logger.error(f"无法设置波特率为 {baudrate}")
                return False
                
            self.is_connected = True
            logger.info(f"已成功连接到舵机控制器，协议类型: {self.protocol_end} (0=STS/SMS, 1=SCS)")
            return True
            
        except serial.SerialException as e:
            logger.error(f"连接到端口 {port} 时发生错误: {e}")
            return False
        except ImportError as e:
            logger.error(f"导入 scservo_sdk 失败，请确保已正确安装: {e}")
            return False
        except Exception as e:
            logger.error(f"连接过程中发生未知错误: {e}")
            return False
    
    def disconnect(self):
        """Close the serial connection"""
        if self.is_connected:
            try:
                if hasattr(self, 'portHandler'):
                    self.portHandler.closePort()
                    
                if self._serial and self._serial.is_open:
                    self._serial.close()
                    
                self.is_connected = False
                logger.info("已断开与舵机控制器的连接")
                return True
                
            except Exception as e:
                logger.error(f"断开连接时发生错误: {e}")
                return False
        return True
    
    def ping(self, servo_id):
        """
        Ping a servo to check if it's responsive
        
        Args:
            servo_id: Servo ID
            
        Returns:
            Tuple: (model_number, result_code, error_code) or (0, -1, 0) if failed
        """
        if not self.is_connected:
            logger.error("未连接到舵机控制器")
            return 0, -1, 0
            
        try:
            # 使用 SDK 的 ping 方法
            model_number, comm_result, error = self.packetHandler.ping(self.portHandler, servo_id)
            
            if comm_result != 0:  # COMM_SUCCESS
                logger.error(f"Ping 通信失败: {self.packetHandler.getTxRxResult(comm_result)}")
                return 0, comm_result, error
                
            if error != 0:
                logger.warning(f"Ping 返回错误: {self.packetHandler.getRxPacketError(error)}")
                
            logger.info(f"[ID:{servo_id:03d}] ping 成功. 舵机型号: {model_number}")
            return model_number, comm_result, error
            
        except Exception as e:
            logger.error(f"Ping 舵机时发生错误: {e}")
            return 0, -1, 0
    
    def set_torque_enable(self, servo_id, enable):
        """
        Enable or disable torque for a servo
        
        Args:
            servo_id: Servo ID
            enable: True to enable, False to disable
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected:
            logger.error("未连接到舵机控制器")
            return False
            
        try:
            value = 1 if enable else 0
            
            # 使用 SDK 写入扭矩使能
            comm_result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, servo_id, self.ADDR_TORQUE_ENABLE, value)
                
            if comm_result != 0:  # COMM_SUCCESS
                logger.error(f"设置扭矩状态失败: {self.packetHandler.getTxRxResult(comm_result)}")
                return False
                
            if error != 0:
                logger.warning(f"设置扭矩状态返回错误: {self.packetHandler.getRxPacketError(error)}")
                return False
                
            action = "使能" if enable else "禁用"
            logger.info(f"[ID:{servo_id:03d}] 舵机扭矩已{action}")
            return True
            
        except Exception as e:
            logger.error(f"设置舵机扭矩时发生错误: {e}")
            return False
    
    def write_position(self, servo_id, position, time=0, speed=0):
        """
        Set the position of a servo
        
        Args:
            servo_id: Servo ID
            position: Target position (0-4095)
            time: Time to reach position in milliseconds (0 for immediate)
            speed: Moving speed (0 for default)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected:
            logger.error("未连接到舵机控制器")
            return False
            
        try:
            # 首先确保舵机能正常通信
            model_number, comm_result, error = self.ping(servo_id)
            if comm_result != 0 or model_number == 0:
                logger.error(f"无法与舵机 ID:{servo_id} 通信，设置位置前请先确认连接")
                return False
                
            # 限制位置范围
            position = max(0, min(4095, position))
            
            # 准备位置数据
            position_data = [
                position & 0xFF,
                (position >> 8) & 0xFF,
                time & 0xFF,
                (time >> 8) & 0xFF
            ]
            
            # 如果需要设置速度
            if speed > 0:
                # 设置速度
                speed_comm_result, speed_error = self.packetHandler.write2ByteTxRx(
                    self.portHandler, servo_id, self.ADDR_GOAL_SPEED, speed)
                    
                if speed_comm_result != 0 or speed_error != 0:
                    logger.warning(f"设置舵机速度失败: {self.packetHandler.getTxRxResult(speed_comm_result)}")
            
            # 写入位置
            comm_result, error = self.packetHandler.writeTxRx(
                self.portHandler, servo_id, self.ADDR_GOAL_POSITION, 4, position_data)
                
            if comm_result != 0:
                logger.error(f"设置位置失败: {self.packetHandler.getTxRxResult(comm_result)}")
                return False
                
            if error != 0:
                error_message = self.packetHandler.getRxPacketError(error)
                logger.warning(f"设置位置返回错误: {error_message}")
                
                # 检查是否过载错误
                if error & 0x20:  # 0x20 是过载错误位
                    logger.error("检测到舵机过载错误，可能原因:")
                    logger.error("1. 舵机负载过大")
                    logger.error("2. 电源电压不稳定")
                    logger.error("3. 舵机内部故障")
                    logger.error("建议: 检查舵机是否被阻塞，减小移动速度或增大移动时间")
                return False
                
            logger.info(f"[ID:{servo_id:03d}] 设置位置:{position}, 时间:{time}, 速度:{speed}")
            return True
            
        except Exception as e:
            logger.error(f"设置舵机位置时发生错误: {e}")
            return False
    
    def read_position(self, servo_id):
        """
        Read the current position of a servo
        
        Args:
            servo_id: Servo ID
            
        Returns:
            Current position (0-4095) or None if error
        """
        if not self.is_connected:
            logger.error("未连接到舵机控制器")
            return None
            
        try:
            # 使用 SDK 读取位置
            position, comm_result, error = self.packetHandler.read2ByteTxRx(
                self.portHandler, servo_id, self.ADDR_PRESENT_POSITION)
                
            if comm_result != 0:
                logger.error(f"读取位置失败: {self.packetHandler.getTxRxResult(comm_result)}")
                return None
                
            if error != 0:
                logger.warning(f"读取位置返回错误: {self.packetHandler.getRxPacketError(error)}")
                
            logger.debug(f"[ID:{servo_id:03d}] 当前位置: {position}")
            return position
            
        except Exception as e:
            logger.error(f"读取舵机位置时发生错误: {e}")
            return None
    
    def sync_write_position(self, positions, times=None):
        """
        Write positions to multiple servos simultaneously
        
        Args:
            positions: Dictionary of {servo_id: position}
            times: Dictionary of {servo_id: time} or single time value for all servos
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_connected or not positions:
            logger.error("未连接到舵机控制器或位置列表为空")
            return False
            
        try:
            from scservo_sdk import GroupSyncWrite
            
            # 初始化 GroupSyncWrite 实例
            groupSyncWrite = GroupSyncWrite(
                self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, 4)
            
            for servo_id, position in positions.items():
                # 限制位置范围
                position = max(0, min(4095, position))
                
                # 处理时间参数
                if times is None:
                    time_value = 0
                elif isinstance(times, dict):
                    time_value = times.get(servo_id, 0)
                else:
                    time_value = times
                
                # 准备数据
                param = [
                    position & 0xFF,
                    (position >> 8) & 0xFF,
                    time_value & 0xFF,
                    (time_value >> 8) & 0xFF
                ]
                
                # 添加参数
                result = groupSyncWrite.addParam(servo_id, param)
                if result != True:
                    logger.error(f"[ID:{servo_id:03d}] 添加同步写入参数失败")
                    return False
            
            # 执行同步写入
            comm_result = groupSyncWrite.txPacket()
            if comm_result != 0:
                logger.error(f"同步写入位置失败: {self.packetHandler.getTxRxResult(comm_result)}")
                return False
                
            logger.info(f"成功同步写入 {len(positions)} 个舵机位置")
            return True
            
        except Exception as e:
            logger.error(f"同步写入舵机位置时发生错误: {e}")
            return False 
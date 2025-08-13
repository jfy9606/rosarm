import serial
import platform
import time
import struct
import sys
import logging

# 导入独立的电机控制器
from .ai_linear_controller import AILinearController
from .ysf4_bldc_controller import YSF4BLDCController

# 设置日志
logging.basicConfig(level=logging.DEBUG, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('DCMotorController')

class DCMotorController:
    """
    大型直流电机控制器管理类 - 统一管理两种电机:
    - YSF4 BLDC电机: 俯仰电机（pitch motor）- 使用YSF4协议
    - AImotor电机: 进给电机（linear feed motor）- 使用ModBus RTU协议
    
    该类作为统一接口，委托给具体的电机控制器实现
    """
    
    def __init__(self, port=None, baudrate=115200, timeout=0.5, debug=False):
        """
        初始化 DC 电机控制器管理器
        
        Args:
            port: 串口名称。如果为 None，则需要稍后连接
            baudrate: 串口通信的波特率
            timeout: 串口超时（秒）
            debug: 是否启用调试模式
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.debug = debug
        
        # 如果启用调试，设置日志级别
        if debug:
            logger.setLevel(logging.DEBUG)
        
        # 创建电机控制器实例
        self.ysf4_controller = YSF4BLDCController(motor_id=1, debug=debug)
        self.ai_controller = AILinearController(debug=debug)
        
        # 通信延时控制
        self.last_comm_time = 0
        self.sleeptime_code = 0.01  # 与SimpleNetwork中的sleeptime_code一致
        
        if port:
            self.connect(port, baudrate, timeout)
    
    def set_debug(self, debug=True):
        """设置调试模式"""
        self.debug = debug
        if debug:
            logger.setLevel(logging.DEBUG)
        else:
            logger.setLevel(logging.INFO)
        
        # 同时设置子控制器的调试模式
        self.ysf4_controller.set_debug(debug) if hasattr(self.ysf4_controller, 'set_debug') else None
        self.ai_controller.set_debug(debug)
    
    def connect(self, port, baudrate=115200, timeout=0.5):
        """
        连接到电机控制器
        
        Args:
            port: 串口名称
            baudrate: 串口波特率
            timeout: 串口超时（秒）
            
        Returns:
            bool: 连接成功返回 True，否则返回 False
        """
        try:
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            
            # 关闭之前的连接
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            logger.info(f"尝试连接到串口 {port}，波特率 {baudrate}...")
            
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
            
            # 将串口连接分享给子控制器（AI 需要现有串口；YSF4 控制器自身 connect 使用端口字符串）
            self.ai_controller.set_serial(self.serial)
            self.ysf4_controller.port = port
            self.ysf4_controller.baudrate = baudrate
            self.ysf4_controller.timeout = timeout
            
            logger.info(f"已连接到串口 {port}")
            
            # 初始化电机 - 先进行通信测试
            time.sleep(0.5)  # 等待电机控制器稳定
            
            # 分别连接/测试两个电机
            logger.info("正在测试电机通信...")
            # 对于YSF4，需要让其打开自身串口连接
            ysf_ok = self.ysf4_controller.connect(port, baudrate, timeout)
            ai_ok = self.ai_controller.test_communication()
            
            if ysf_ok or ai_ok:
                logger.info(f"电机通信测试结果: YSF4俯仰电机={ysf_ok}, AImotor进给电机={ai_ok}")
                return True
                
            logger.warning(f"电机通信测试失败: YSF4={ysf_ok}, AI={ai_ok}")
            # 尝试不同的波特率
            alternate_baudrates = [9600, 19200, 38400, 57600, 115200]
            if baudrate in alternate_baudrates:
                alternate_baudrates.remove(baudrate)
            
            for alt_baudrate in alternate_baudrates:
                logger.info(f"尝试使用备用波特率 {alt_baudrate}...")
                try:
                    self.serial.close()
                    self.serial = serial.Serial(
                        port=port,
                        baudrate=alt_baudrate,
                        timeout=timeout,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE
                    )
                    self.baudrate = alt_baudrate
                    
                    # 清空缓冲区
                    self.serial.reset_input_buffer()
                    self.serial.reset_output_buffer()
                    
                    # 更新子控制器连接参数
                    self.ai_controller.set_serial(self.serial)
                    ysf_ok = self.ysf4_controller.connect(port, alt_baudrate, timeout)
                    ai_ok = self.ai_controller.test_communication()
                    if ysf_ok or ai_ok:
                        logger.info(f"使用波特率 {alt_baudrate} 通信成功: YSF4={ysf_ok}, AI={ai_ok}")
                        return True
                except Exception as e:
                    logger.error(f"尝试波特率 {alt_baudrate} 失败: {e}")
            
            # 所有波特率都失败
            logger.error("所有波特率都无法与电机通信")
            self.connected = False
            return False
            
        except Exception as e:
            logger.error(f"连接到电机控制器失败: {e}")
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
    
    # 委托方法 - 俯仰电机（YSF4）相关
    def set_pitch_position(self, position, speed=None):
        """设置俯仰电机位置 - 委托给YSF4控制器"""
        return self.ysf4_controller.set_position(position, speed)
    
    # 委托方法 - AImotor进给电机相关
    def set_linear_position(self, position, speed=None):
        """设置AImotor进给电机位置 - 委托给AI控制器"""
        return self.ai_controller.pos_form(self.ai_controller.AI_STATION, position, speed)
    
    def pos_form(self, station_num, position, vel=None, pos_mode=True):
        """委托给AI控制器的pos_form方法"""
        return self.ai_controller.pos_form(station_num, position, vel, pos_mode)
    
    def vel_form(self, station_num, vel):
        """委托给AI控制器的vel_form方法"""
        return self.ai_controller.vel_form(station_num, vel)
    
    def set_form(self, station_num, form_input):
        """委托给AI控制器的set_form方法"""
        return self.ai_controller.set_form(station_num, form_input)
    
    # 兼容 robot_arm.py 的接口方法
    def set_motor_speed(self, pitch_speed=None, linear_speed=None):
        """
        设置电机速度 - 兼容robot_arm.py接口
        
        Args:
            pitch_speed: 俯仰电机速度
            linear_speed: 进给电机速度
        """
        results = {}
        
        if pitch_speed is not None:
            results['pitch'] = self.ysf4_controller.set_speed(pitch_speed)
        
        if linear_speed is not None:
            results['linear'] = self.ai_controller.vel_form(self.ai_controller.AI_STATION, linear_speed)
        
        return results
    
    def get_pitch_position(self):
        """获取俯仰电机位置 - 兼容robot_arm.py接口"""
        if hasattr(self.ysf4_controller, 'position'):
            return self.ysf4_controller.position
        else:
            # 尝试读取位置
            pos = self.ysf4_controller.read_position()
            return pos if pos is not None else 0
    
    def get_linear_position(self):
        """获取进给电机位置 - 兼容robot_arm.py接口"""
        return self.ai_controller.position
    
    # 通用电机控制方法
    def enable_motors(self, enable_pitch=True, enable_linear=True):
        """
        使能电机
        
        Args:
            enable_pitch: 是否使能俯仰电机
            enable_linear: 是否使能进给电机
            
        Returns:
            dict: 各电机使能结果
        """
        results = {}
        
        if enable_pitch:
            results['pitch'] = self.ysf4_controller.enable()
        
        if enable_linear:
            results['linear'] = self.ai_controller.enable()
        
        return results
    
    def stop_all(self):
        """停止所有电机"""
        results = {}
        
        # 停止俯仰电机
        results['pitch'] = self.ysf4_controller.disable()
        
        # 停止AImotor
        results['linear'] = self.ai_controller.disable()
        
        return results
    
    def test_communication(self):
        """
        测试电机通信
        
        Returns:
            dict: 各电机通信测试结果
        """
        results = {
            "pitch": False,
            "linear": False
        }
        
        try:
            # 测试俯仰电机通信（YSF4）
            results["pitch"] = self.ysf4_controller.test_communication()
            
            # 测试AImotor进给电机通信
            results["linear"] = self.ai_controller.test_communication()
            
        except Exception as e:
            logger.error(f"通信测试异常: {e}")
        
        return results
    
    def get_status(self):
        """
        获取所有电机状态
        
        Returns:
            dict: 包含所有电机状态的字典
        """
        return {
            "connected": self.is_connected(),
            "port": self.port,
            "baudrate": self.baudrate,
            "pitch": self.ysf4_controller.get_motor_info() if hasattr(self.ysf4_controller, 'get_motor_info') else {},
            "linear": self.ai_controller.get_status()
        }
    
    def get_motor_positions(self):
        """
        获取所有电机位置
        
        Returns:
            dict: 各电机位置信息
        """
        try:
            return {
                "pitch_position": self.ysf4_controller.position if hasattr(self.ysf4_controller, 'position') else None,
                "linear_position": self.ai_controller.position,
                "timestamp": time.time()
            }
        except Exception as e:
            logger.error(f"获取电机位置失败: {e}")
            return {
                "pitch_position": None,
                "linear_position": None,
                "timestamp": time.time(),
                "error": str(e)
            }
    
    @staticmethod
    def scan_ports():
        """
        扫描可用的串口
        
        Returns:
            list: 可用串口列表
        """
        import serial.tools.list_ports
        
        available_ports = []
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            available_ports.append(port.device)
        
        return available_ports

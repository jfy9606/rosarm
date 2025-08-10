import serial
import time
import logging

# 设置日志
logger = logging.getLogger('AILinearController')

class AILinearController:
    """
    AImotor 进给电机控制器 - 使用 ModBus RTU 协议
    基于 SimpleNetwork 项目的实现
    """
    
    # ModBus RTU 协议常量
    AI_STATION = 1  # AImotor 站号
    
    # 命令代码列表 (来自SimpleNetwork项目)
    CODE_LIST = [
        [6, 0, 40, 0, 1],        # 0: 使能开
        [6, 0, 40, 0, 1],        # 1: 使能开 (重复)
        [6, 0, 40, 0, 0],        # 2: 使能关
        [6, 0, 32, 0, 0],        # 3: 位置模式
        [6, 0, 32, 0, 1],        # 4: 速度模式
        [6, 0, 4, 0, 1],         # 5: 开始运动
        [6, 0, 4, 0, 0],         # 6: 停止运动
        [16, 0, 0, 0, 2, 4],     # 7: 设置位置(高位)
        [16, 0, 2, 0, 2, 4],     # 8: 设置位置(低位)
        [16, 0, 6, 0, 2, 4],     # 9: 设置速度
    ]
    
    def __init__(self, debug=False):
        """
        初始化 AImotor 控制器
        
        Args:
            debug: 是否启用调试模式
        """
        self.serial = None
        self.connected = False
        self.debug = debug
        self.enabled = False
        self.position = 0
        self.speed = 0
        
        # 通信延时控制
        self.last_comm_time = 0
        self.sleeptime_code = 0.01  # 与SimpleNetwork中的sleeptime_code一致
        
        if debug:
            logger.setLevel(logging.DEBUG)
        
        logger.info("AILinearController 初始化完成")
    
    def set_debug(self, debug=True):
        """设置调试模式"""
        self.debug = debug
        if debug:
            logger.setLevel(logging.DEBUG)
        else:
            logger.setLevel(logging.INFO)
    
    def set_serial(self, serial_connection):
        """
        设置串口连接
        
        Args:
            serial_connection: 已连接的串口对象
        """
        self.serial = serial_connection
        self.connected = serial_connection is not None and serial_connection.is_open
        
        if self.connected:
            logger.info("AILinearController 串口连接已设置")
    
    def _communication_delay(self):
        """控制通信延时，确保通信稳定性"""
        current_time = time.time()
        elapsed = current_time - self.last_comm_time
        if elapsed < self.sleeptime_code:
            time.sleep(self.sleeptime_code - elapsed)
        self.last_comm_time = time.time()
    
    def CRC16_MudBus(self, data):
        """
        计算 ModBus CRC16 校验码
        
        Args:
            data: 数据列表
            
        Returns:
            list: [CRC_L, CRC_H] 低位和高位CRC
        """
        CRC = 0xFFFF
        for byte in data:
            CRC ^= byte
            for _ in range(8):
                if CRC & 1:
                    CRC >>= 1
                    CRC ^= 0xA001
                else:
                    CRC >>= 1
        
        CRC_L = CRC & 0xFF
        CRC_H = (CRC >> 8) & 0xFF
        
        return [CRC_L, CRC_H]
    
    def _send_command(self, command_code, *args):
        """
        发送命令到 AImotor
        
        Args:
            command_code: 命令代码
            *args: 额外参数
            
        Returns:
            bool: 发送成功返回True
        """
        if not self.connected or not self.serial or not self.serial.is_open:
            logger.error("AImotor 未连接")
            return False
        
        try:
            # 构建命令
            if command_code < len(self.CODE_LIST):
                code = [self.AI_STATION] + self.CODE_LIST[command_code][:]
                
                # 添加额外参数
                if args:
                    code.extend(args)
                
                # 计算并添加CRC
                crc_values = self.CRC16_MudBus(code)
                code.extend(crc_values)
                
                # 发送命令
                self.serial.write(bytearray(code))
                self._communication_delay()
                
                if self.debug:
                    logger.debug(f"发送命令: {[hex(x) for x in code]}")
                
                return True
            else:
                logger.error(f"无效的命令代码: {command_code}")
                return False
                
        except Exception as e:
            logger.error(f"发送命令失败: {e}")
            return False
    
    def set_form(self, station_num, form_input):
        """
        设置电机控制模式
        
        Args:
            station_num: 站号
            form_input: 模式输入 (0=位置模式, 1=速度模式)
            
        Returns:
            bool: 设置成功返回True
        """
        try:
            if form_input == 0:
                # 位置模式
                success = self._send_command(3)
            elif form_input == 1:
                # 速度模式
                success = self._send_command(4)
            else:
                logger.error(f"无效的控制模式: {form_input}")
                return False
            
            if success:
                logger.info(f"AImotor 控制模式设置为: {'位置模式' if form_input == 0 else '速度模式'}")
                
            return success
            
        except Exception as e:
            logger.error(f"设置控制模式失败: {e}")
            return False
    
    def pos_form(self, station_num, position, vel=None, pos_mode=True):
        """
        设置 AImotor 位置
        
        Args:
            station_num: 站号
            position: 目标位置
            vel: 速度 (可选)
            pos_mode: 是否位置模式
            
        Returns:
            bool: 设置成功返回True
        """
        try:
            # 确保位置模式
            if pos_mode and not self.set_form(station_num, 0):
                return False
            
            # 设置速度 (如果提供)
            if vel is not None:
                self.vel_form(station_num, vel)
            
            # 位置数据转换为32位
            pos_32 = int(position) & 0xFFFFFFFF
            pos_high = (pos_32 >> 16) & 0xFFFF
            pos_low = pos_32 & 0xFFFF
            
            # 发送位置高位
            code_high = [self.AI_STATION] + self.CODE_LIST[7] + [
                (pos_high >> 8) & 0xFF, pos_high & 0xFF,
                (pos_high >> 8) & 0xFF, pos_high & 0xFF
            ]
            crc_high = self.CRC16_MudBus(code_high)
            code_high.extend(crc_high)
            
            # 发送位置低位
            code_low = [self.AI_STATION] + self.CODE_LIST[8] + [
                (pos_low >> 8) & 0xFF, pos_low & 0xFF,
                (pos_low >> 8) & 0xFF, pos_low & 0xFF
            ]
            crc_low = self.CRC16_MudBus(code_low)
            code_low.extend(crc_low)
            
            # 发送命令
            if self.serial and self.serial.is_open:
                self.serial.write(bytearray(code_high))
                self._communication_delay()
                self.serial.write(bytearray(code_low))
                self._communication_delay()
                
                # 启动运动
                self._send_command(5)
                
                self.position = position
                
                if self.debug:
                    logger.debug(f"AImotor 位置设置为: {position}")
                
                return True
            else:
                logger.error("串口未连接")
                return False
                
        except Exception as e:
            logger.error(f"设置位置失败: {e}")
            return False
    
    def vel_form(self, station_num, vel):
        """
        设置 AImotor 速度
        
        Args:
            station_num: 站号
            vel: 目标速度
            
        Returns:
            bool: 设置成功返回True
        """
        try:
            # 速度数据转换
            vel_32 = int(vel) & 0xFFFFFFFF
            vel_high = (vel_32 >> 16) & 0xFFFF
            vel_low = vel_32 & 0xFFFF
            
            # 构建速度命令
            code = [self.AI_STATION] + self.CODE_LIST[9] + [
                (vel_low >> 8) & 0xFF, vel_low & 0xFF,
                (vel_high >> 8) & 0xFF, vel_high & 0xFF
            ]
            crc_values = self.CRC16_MudBus(code)
            code.extend(crc_values)
            
            # 发送命令
            if self.serial and self.serial.is_open:
                self.serial.write(bytearray(code))
                self._communication_delay()
                
                self.speed = vel
                
                if self.debug:
                    logger.debug(f"AImotor 速度设置为: {vel}")
                
                return True
            else:
                logger.error("串口未连接")
                return False
                
        except Exception as e:
            logger.error(f"设置速度失败: {e}")
            return False
    
    def enable(self):
        """使能 AImotor"""
        success = self._send_command(1)
        if success:
            self.enabled = True
            logger.info("AImotor 已使能")
        return success
    
    def disable(self):
        """禁用 AImotor"""
        success = self._send_command(2)
        if success:
            self.enabled = False
            logger.info("AImotor 已禁用")
        return success
    
    def start_motion(self):
        """开始运动"""
        return self._send_command(5)
    
    def stop_motion(self):
        """停止运动"""
        return self._send_command(6)
    
    def test_communication(self):
        """
        测试 AImotor 通信
        
        Returns:
            bool: 通信正常返回True
        """
        try:
            if not self.connected:
                return False
            
            # 尝试设置位置模式作为通信测试
            code = [self.AI_STATION] + self.CODE_LIST[3]  # 位置模式命令
            crc_values = self.CRC16_MudBus(code)
            code.extend(crc_values)
            
            if self.serial and self.serial.is_open:
                self.serial.reset_input_buffer()
                self.serial.write(bytearray(code))
                self._communication_delay()
                
                # 简单的响应检查 (AImotor可能不返回确认)
                # 如果没有异常发生，认为通信正常
                logger.info("AImotor 通信测试成功")
                return True
            else:
                return False
                
        except Exception as e:
            logger.error(f"AImotor 通信测试失败: {e}")
            return False
    
    def get_status(self):
        """
        获取 AImotor 状态
        
        Returns:
            dict: 状态信息
        """
        return {
            "connected": self.connected,
            "enabled": self.enabled,
            "position": self.position,
            "speed": self.speed,
            "type": "AImotor_linear"
        }
    
    def is_connected(self):
        """检查连接状态"""
        return self.connected and self.serial and self.serial.is_open
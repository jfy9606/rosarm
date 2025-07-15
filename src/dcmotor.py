import serial
import platform
import time
import struct

class DCMotorController:
    """
    Controller class for the two large DC motors:
    - YF型号: 大臂的俯仰电机（pitch motor）
    - AImotor型号: 大臂的进给电机（linear feed motor）
    通过串口通信控制
    """
    
    # Command codes
    CMD_SET_PITCH = 0x01    # YF俯仰电机命令
    CMD_SET_LINEAR = 0x02   # AImotor进给电机命令
    CMD_STOP_ALL = 0x03
    CMD_GET_STATUS = 0x04
    CMD_SET_SPEED = 0x05
    CMD_SET_ACCEL = 0x06
    CMD_HOME = 0x07
    
    def __init__(self, port=None, baudrate=115200, timeout=0.1):
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
        self._serial = None
        
        if port:
            self.connect(port, baudrate, timeout)
    
    def connect(self, port, baudrate=115200, timeout=0.1):
        """
        Connect to the motor controller
        
        Args:
            port: Serial port name
            baudrate: Baud rate for serial communication
            timeout: Serial timeout in seconds
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
            return True
        except serial.SerialException as e:
            print(f"Error connecting to port {port}: {e}")
            return False
    
    def disconnect(self):
        """Close the serial connection"""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
    
    def _calculate_checksum(self, data):
        """
        Calculate checksum for a data packet
        
        Args:
            data: Bytes to calculate checksum for
            
        Returns:
            Checksum byte
        """
        checksum = 0
        for b in data:
            checksum ^= b  # XOR checksum
        return checksum
    
    def _send_command(self, cmd, data=None):
        """
        Send a command to the motor controller
        
        Args:
            cmd: Command byte
            data: Data bytes to send (optional)
            
        Returns:
            True if command was sent, False otherwise
        """
        if not self._serial or not self._serial.is_open:
            print("Serial port not open")
            return False
        
        if data is None:
            data = []
        
        # Create packet: header (0xAA, 0x55) + cmd + length + data + checksum
        length = len(data)
        packet = bytearray([0xAA, 0x55, cmd, length]) + bytearray(data)
        checksum = self._calculate_checksum(packet)
        packet.append(checksum)
        
        try:
            self._serial.write(packet)
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def _receive_response(self, expected_length=None):
        """
        Receive a response from the motor controller
        
        Args:
            expected_length: Expected response length (optional)
            
        Returns:
            Response data or None if no valid response
        """
        if not self._serial or not self._serial.is_open:
            print("Serial port not open")
            return None
        
        # Find header (0xAA, 0x55)
        header = b''
        timeout_start = time.time()
        
        while len(header) < 2 and (time.time() - timeout_start) < self.timeout:
            h = self._serial.read(1)
            if not h:
                continue  # No data yet
            
            if len(header) == 0:
                if h == b'\xAA':
                    header += h
            elif len(header) == 1:
                if h == b'\x55':
                    header += h
                else:
                    header = b''  # Reset
                    if h == b'\xAA':
                        header += h
        
        if len(header) != 2:
            return None  # Timeout or no valid header found
        
        # Read command byte
        cmd = self._serial.read(1)
        if not cmd:
            return None
        
        # Read length byte
        length_bytes = self._serial.read(1)
        if not length_bytes:
            return None
        length = length_bytes[0]
        
        # Read data
        data = self._serial.read(length)
        if len(data) != length:
            return None
        
        # Read checksum
        checksum = self._serial.read(1)
        if not checksum:
            return None
        
        # Verify checksum
        packet = header + cmd + length_bytes + data
        calculated_checksum = self._calculate_checksum(packet)
        if checksum[0] != calculated_checksum:
            print(f"Checksum error: {checksum[0]} != {calculated_checksum}")
            return None
        
        return cmd[0], data
    
    def set_pitch_position(self, position, speed=None):
        """
        Set the YF型号俯仰电机 position
        
        Args:
            position: Target position (-32768 to 32767)
            speed: Optional speed parameter (0-255)
            
        Returns:
            True if successful, False otherwise
        """
        # Convert position to bytes (little-endian signed 16-bit)
        position_bytes = list(struct.pack("<h", position))
        
        # Add speed if provided
        if speed is not None:
            position_bytes.append(min(255, max(0, speed)))
            
        if not self._send_command(self.CMD_SET_PITCH, position_bytes):
            return False
        
        response = self._receive_response()
        return response is not None and response[0] == self.CMD_SET_PITCH
    
    def set_linear_position(self, position, speed=None):
        """
        Set the AImotor型号进给电机 position
        
        Args:
            position: Target position (-32768 to 32767)
            speed: Optional speed parameter (0-255)
            
        Returns:
            True if successful, False otherwise
        """
        # Convert position to bytes (little-endian signed 16-bit)
        position_bytes = list(struct.pack("<h", position))
        
        # Add speed if provided
        if speed is not None:
            position_bytes.append(min(255, max(0, speed)))
            
        if not self._send_command(self.CMD_SET_LINEAR, position_bytes):
            return False
        
        response = self._receive_response()
        return response is not None and response[0] == self.CMD_SET_LINEAR
    
    def stop_all(self):
        """
        Stop all motors immediately
        
        Returns:
            True if successful, False otherwise
        """
        if not self._send_command(self.CMD_STOP_ALL):
            return False
        
        response = self._receive_response()
        return response is not None and response[0] == self.CMD_STOP_ALL
    
    def get_status(self):
        """
        Get the current status of both motors
        
        Returns:
            Dictionary with motor status or None if error
        """
        if not self._send_command(self.CMD_GET_STATUS):
            return None
        
        response = self._receive_response()
        if not response or response[0] != self.CMD_GET_STATUS:
            return None
        
        data = response[1]
        if len(data) < 8:
            return None
        
        # Parse the response data
        status = {
            'pitch_position': struct.unpack("<h", data[0:2])[0],
            'linear_position': struct.unpack("<h", data[2:4])[0],
            'pitch_moving': bool(data[4]),
            'linear_moving': bool(data[5]),
            'pitch_error': data[6],
            'linear_error': data[7]
        }
        
        return status
    
    def home_motors(self):
        """
        Send motors to home position
        
        Returns:
            True if successful, False otherwise
        """
        if not self._send_command(self.CMD_HOME):
            return False
        
        response = self._receive_response()
        return response is not None and response[0] == self.CMD_HOME
    
    def set_motor_speed(self, pitch_speed=None, linear_speed=None):
        """
        Set the speed for both motors
        
        Args:
            pitch_speed: Speed for YF俯仰电机 (0-255)
            linear_speed: Speed for AImotor进给电机 (0-255)
            
        Returns:
            True if successful, False otherwise
        """
        data = []
        
        if pitch_speed is not None:
            data.append(min(255, max(0, pitch_speed)))
        else:
            data.append(255)  # Default to max speed
            
        if linear_speed is not None:
            data.append(min(255, max(0, linear_speed)))
        else:
            data.append(255)  # Default to max speed
            
        if not self._send_command(self.CMD_SET_SPEED, data):
            return False
        
        response = self._receive_response()
        return response is not None and response[0] == self.CMD_SET_SPEED
    
    def set_motor_acceleration(self, pitch_accel=None, linear_accel=None):
        """
        Set acceleration parameters for both motors
        
        Args:
            pitch_accel: Acceleration for YF俯仰电机 (0-255)
            linear_accel: Acceleration for AImotor进给电机 (0-255)
            
        Returns:
            True if successful, False otherwise
        """
        data = []
        
        if pitch_accel is not None:
            data.append(min(255, max(0, pitch_accel)))
        else:
            data.append(10)  # Default value
            
        if linear_accel is not None:
            data.append(min(255, max(0, linear_accel)))
        else:
            data.append(10)  # Default value
            
        if not self._send_command(self.CMD_SET_ACCEL, data):
            return False
        
        response = self._receive_response()
        return response is not None and response[0] == self.CMD_SET_ACCEL 
import serial
import struct
import time
import platform

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

    def __init__(self, port=None, baudrate=115200, timeout=0.1):
        """
        Initialize SCServo communication
        
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
        Connect to the servo controller
        
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
        Calculate the checksum for a packet
        
        Args:
            data: Bytes to calculate checksum for
        
        Returns:
            Checksum byte
        """
        checksum = (~sum(data)) & 0xFF
        return checksum
    
    def _send_packet(self, servo_id, cmd, data=None):
        """
        Send a packet to the servo
        
        Args:
            servo_id: Servo ID (0-253, 254 for broadcast)
            cmd: Command byte
            data: Data bytes to send (optional)
            
        Returns:
            True if packet was sent, False otherwise
        """
        if not self._serial or not self._serial.is_open:
            print("Serial port not open")
            return False
        
        if data is None:
            data = []
        
        # Create packet
        length = len(data) + 2  # data length + cmd byte + checksum byte
        packet = bytearray([0xFF, 0xFF, servo_id, length, cmd]) + bytearray(data)
        checksum = self._calculate_checksum(packet[2:])  # start from ID
        packet.append(checksum)
        
        try:
            self._serial.write(packet)
            return True
        except Exception as e:
            print(f"Error sending packet: {e}")
            return False
    
    def _receive_packet(self, expected_length=None):
        """
        Receive a response packet
        
        Args:
            expected_length: Expected packet length (optional)
            
        Returns:
            Tuple of (servo_id, error, data) or None if no valid packet
        """
        if not self._serial or not self._serial.is_open:
            print("Serial port not open")
            return None
        
        # Find header (0xFF, 0xFF)
        header = b''
        while len(header) < 2:
            h = self._serial.read(1)
            if not h:
                return None  # Timeout
            
            if len(header) == 0:
                if h == b'\xff':
                    header += h
            elif len(header) == 1:
                if h == b'\xff':
                    header += h
                else:
                    header = b''  # Reset
                    if h == b'\xff':
                        header += h
        
        # Read ID
        servo_id = self._serial.read(1)
        if not servo_id:
            return None
        servo_id = servo_id[0]
        
        # Read length
        length_bytes = self._serial.read(1)
        if not length_bytes:
            return None
        length = length_bytes[0]
        
        # Read error
        error = self._serial.read(1)
        if not error:
            return None
        error = error[0]
        
        # Read data (length - 2 bytes, excluding error and checksum)
        data = self._serial.read(length - 2)
        if len(data) != length - 2:
            return None
        
        # Read checksum
        checksum = self._serial.read(1)
        if not checksum:
            return None
        checksum = checksum[0]
        
        # Verify checksum
        calculated_checksum = self._calculate_checksum(
            bytes([servo_id, length]) + error + data
        )
        if checksum != calculated_checksum:
            print(f"Checksum error: {checksum} != {calculated_checksum}")
            return None
        
        return servo_id, error, data
    
    def ping(self, servo_id):
        """
        Ping a servo to check if it's responsive
        
        Args:
            servo_id: Servo ID
            
        Returns:
            True if servo responds, False otherwise
        """
        if not self._send_packet(servo_id, self.CMD_PING):
            return False
        
        response = self._receive_packet()
        return response is not None
    
    def read_position(self, servo_id):
        """
        Read the current position of a servo
        
        Args:
            servo_id: Servo ID
            
        Returns:
            Current position (0-4095) or None if error
        """
        if not self._send_packet(servo_id, self.CMD_READ_DATA, 
                               [self.ADDR_PRESENT_POSITION, 2]):
            return None
        
        response = self._receive_packet()
        if not response:
            return None
        
        _, error, data = response
        if error != 0:
            print(f"Servo error: {error}")
            return None
        
        if len(data) != 2:
            return None
        
        position = data[0] + (data[1] << 8)
        return position
    
    def write_position(self, servo_id, position, time=0):
        """
        Set the position of a servo
        
        Args:
            servo_id: Servo ID
            position: Target position (0-4095)
            time: Time to reach position in milliseconds (0 for immediate)
            
        Returns:
            True if successful, False otherwise
        """
        # Clamp position to valid range
        position = max(0, min(4095, position))
        
        # Prepare data bytes
        pos_low = position & 0xFF
        pos_high = (position >> 8) & 0xFF
        time_low = time & 0xFF
        time_high = (time >> 8) & 0xFF
        
        if not self._send_packet(servo_id, self.CMD_WRITE_DATA, 
                              [self.ADDR_GOAL_POSITION, pos_low, pos_high, time_low, time_high]):
            return False
        
        response = self._receive_packet()
        return response is not None
    
    def set_torque_enable(self, servo_id, enable):
        """
        Enable or disable torque for a servo
        
        Args:
            servo_id: Servo ID
            enable: True to enable, False to disable
            
        Returns:
            True if successful, False otherwise
        """
        value = 1 if enable else 0
        
        if not self._send_packet(servo_id, self.CMD_WRITE_DATA, 
                              [self.ADDR_TORQUE_ENABLE, value, 0]):
            return False
        
        response = self._receive_packet()
        return response is not None
    
    def sync_write_position(self, positions, times=None):
        """
        Write positions to multiple servos simultaneously
        
        Args:
            positions: Dictionary of {servo_id: position}
            times: Dictionary of {servo_id: time} or single time value for all servos
            
        Returns:
            True if successful, False otherwise
        """
        if not positions:
            return False
        
        # Prepare data for sync write
        data = [self.ADDR_GOAL_POSITION, 4]  # Address, data length (pos + time = 2+2 bytes)
        
        for servo_id, position in positions.items():
            # Clamp position to valid range
            position = max(0, min(4095, position))
            pos_low = position & 0xFF
            pos_high = (position >> 8) & 0xFF
            
            # Handle time parameter
            if times is None:
                time = 0
            elif isinstance(times, dict):
                time = times.get(servo_id, 0)
            else:
                time = times
            
            time_low = time & 0xFF
            time_high = (time >> 8) & 0xFF
            
            data.extend([servo_id, pos_low, pos_high, time_low, time_high])
        
        return self._send_packet(254, self.CMD_SYNC_WRITE, data)  # Broadcast ID = 254 
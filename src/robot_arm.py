from .scservo import SCServo
from .dcmotor import DCMotorController
import time
import math


class RobotArm:
    """
    Main robot arm control class that integrates SCServo joints and DC motors
    """
    
    # Default joint IDs for servo motors
    DEFAULT_JOINT_IDS = {
        'joint1': 1,  # Base rotation
        'joint2': 2,  # Shoulder
        'joint3': 3,  # Elbow
        'joint4': 4,  # Wrist pitch
        'joint5': 5,  # Wrist roll (not used in current setup)
        'joint6': 6   # Gripper (not used in current setup)
    }
    
    # Joint limits in servo position units (0-4095)
    DEFAULT_JOINT_LIMITS = {
        'joint1': (0, 4095),
        'joint2': (500, 3500),
        'joint3': (500, 3500),
        'joint4': (500, 3500),
        'joint5': (500, 3500),
        'joint6': (500, 3500)
    }
    
    # DC motor limits
    DEFAULT_PITCH_LIMITS = (-10000, 10000)
    DEFAULT_LINEAR_LIMITS = (0, 20000)
    
    def __init__(self, servo_port=None, motor_port=None, joint_ids=None, joint_limits=None):
        """
        Initialize the robot arm controller
        
        Args:
            servo_port: Serial port for servo control
            motor_port: Serial port for DC motor control
            joint_ids: Dictionary mapping joint names to servo IDs
            joint_limits: Dictionary mapping joint names to (min, max) position limits
        """
        # Initialize servo controller
        self.servo = SCServo(port=servo_port) if servo_port else SCServo()
        
        # Initialize DC motor controller
        self.motor = DCMotorController(port=motor_port) if motor_port else DCMotorController()
        
        # Set joint IDs and limits
        self.joint_ids = joint_ids if joint_ids else self.DEFAULT_JOINT_IDS
        self.joint_limits = joint_limits if joint_limits else self.DEFAULT_JOINT_LIMITS
        
        # Set DC motor limits
        self.pitch_limits = self.DEFAULT_PITCH_LIMITS
        self.linear_limits = self.DEFAULT_LINEAR_LIMITS
        
        # Current joint positions
        self.current_positions = {}
        
        # Current DC motor positions
        self.current_pitch = 0
        self.current_linear = 0
        
        # Movement speeds
        self.servo_speed = 0  # Time in ms (0 = max speed)
        self.pitch_speed = 100
        self.linear_speed = 100
    
    def connect(self, servo_port, motor_port, servo_baudrate=1000000, motor_baudrate=115200):
        """
        Connect to the servo and motor controllers
        
        Args:
            servo_port: Serial port for servo control
            motor_port: Serial port for DC motor control
            servo_baudrate: Baud rate for servo communication
            motor_baudrate: Baud rate for motor communication
            
        Returns:
            True if both connections were successful, False otherwise
        """
        servo_connected = self.servo.connect(servo_port, baudrate=servo_baudrate)
        motor_connected = self.motor.connect(motor_port, baudrate=motor_baudrate)
        
        return servo_connected and motor_connected
    
    def disconnect(self):
        """
        Disconnect from both controllers
        """
        self.servo.disconnect()
        self.motor.disconnect()
    
    def enable_torque(self, enable=True):
        """
        Enable or disable torque for all servos
        
        Args:
            enable: True to enable, False to disable
            
        Returns:
            True if successful for all servos, False otherwise
        """
        success = True
        for joint, servo_id in self.joint_ids.items():
            if not self.servo.set_torque_enable(servo_id, enable):
                success = False
        
        return success
    
    def _clamp_joint_position(self, joint, position):
        """
        Clamp a joint position to within its limits
        
        Args:
            joint: Joint name
            position: Target position
            
        Returns:
            Clamped position value
        """
        if joint in self.joint_limits:
            min_pos, max_pos = self.joint_limits[joint]
            return max(min_pos, min(max_pos, position))
        return position
    
    def _clamp_motor_position(self, motor_type, position):
        """
        Clamp a DC motor position to within its limits
        
        Args:
            motor_type: 'pitch' or 'linear'
            position: Target position
            
        Returns:
            Clamped position value
        """
        if motor_type == 'pitch':
            min_pos, max_pos = self.pitch_limits
        else:  # linear
            min_pos, max_pos = self.linear_limits
            
        return max(min_pos, min(max_pos, position))
    
    def read_joint_positions(self):
        """
        Read the current positions of all servos
        
        Returns:
            Dictionary of joint positions or None if error
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
        Read the status of both DC motors
        
        Returns:
            Dictionary with motor status or None if error
        """
        status = self.motor.get_status()
        if status:
            self.current_pitch = status['pitch_position']
            self.current_linear = status['linear_position']
        
        return status
    
    def set_joint_position(self, joint, position, blocking=False, timeout=5.0):
        """
        Set the position of a specific joint
        
        Args:
            joint: Joint name
            position: Target position
            blocking: If True, wait for movement to complete
            timeout: Maximum time to wait if blocking
            
        Returns:
            True if successful, False otherwise
        """
        if joint not in self.joint_ids:
            print(f"Unknown joint: {joint}")
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
                
                # Check if position is close enough
                if abs(current_pos - position) < 10:
                    break
                
                time.sleep(0.05)
        
        if success:
            self.current_positions[joint] = position
            
        return success
    
    def set_joint_positions(self, positions, blocking=False, timeout=5.0):
        """
        Set the positions of multiple joints
        
        Args:
            positions: Dictionary of {joint: position}
            blocking: If True, wait for all movements to complete
            timeout: Maximum time to wait if blocking
            
        Returns:
            True if successful, False otherwise
        """
        # Convert joint names to servo IDs and clamp positions
        servo_positions = {}
        times = {}
        
        for joint, position in positions.items():
            if joint in self.joint_ids:
                servo_id = self.joint_ids[joint]
                clamped_position = self._clamp_joint_position(joint, position)
                servo_positions[servo_id] = clamped_position
                times[servo_id] = self.servo_speed
            else:
                print(f"Unknown joint: {joint}")
        
        if not servo_positions:
            return False
        
        # Use sync write to set positions simultaneously
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
                        
                        # Check if position is close enough
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
    
    def set_pitch_position(self, position, blocking=False, timeout=5.0):
        """
        Set the pitch DC motor position
        
        Args:
            position: Target position
            blocking: If True, wait for movement to complete
            timeout: Maximum time to wait if blocking
            
        Returns:
            True if successful, False otherwise
        """
        position = self._clamp_motor_position('pitch', position)
        
        success = self.motor.set_pitch_position(position, self.pitch_speed)
        
        if success and blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.motor.get_status()
                
                if status is None:
                    return False
                
                if not status['pitch_moving'] and abs(status['pitch_position'] - position) < 10:
                    break
                    
                time.sleep(0.05)
        
        if success:
            self.current_pitch = position
            
        return success
    
    def set_linear_position(self, position, blocking=False, timeout=5.0):
        """
        Set the linear feed DC motor position
        
        Args:
            position: Target position
            blocking: If True, wait for movement to complete
            timeout: Maximum time to wait if blocking
            
        Returns:
            True if successful, False otherwise
        """
        position = self._clamp_motor_position('linear', position)
        
        success = self.motor.set_linear_position(position, self.linear_speed)
        
        if success and blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.motor.get_status()
                
                if status is None:
                    return False
                
                if not status['linear_moving'] and abs(status['linear_position'] - position) < 10:
                    break
                    
                time.sleep(0.05)
        
        if success:
            self.current_linear = position
            
        return success
    
    def set_speeds(self, servo_speed=None, pitch_speed=None, linear_speed=None):
        """
        Set speeds for the different motors
        
        Args:
            servo_speed: Time in ms for servo movements (0 = max speed)
            pitch_speed: Speed for pitch motor (0-255)
            linear_speed: Speed for linear motor (0-255)
            
        Returns:
            True if successful, False otherwise
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
    
    def stop_all(self):
        """
        Stop all motors immediately
        
        Returns:
            True if successful, False otherwise
        """
        dc_success = self.motor.stop_all()
        
        # For servos, we set torque off which effectively stops them
        servo_success = self.enable_torque(False)
        
        return dc_success and servo_success
    
    def home(self, blocking=True, timeout=10.0):
        """
        Move the arm to the home position
        
        Args:
            blocking: If True, wait for movement to complete
            timeout: Maximum time to wait if blocking
            
        Returns:
            True if successful, False otherwise
        """
        # Define home positions for all joints
        home_positions = {
            'joint1': 2048,  # Center position
            'joint2': 2048,
            'joint3': 2048,
            'joint4': 2048
        }
        
        # Set all joints to home position
        joints_success = self.set_joint_positions(home_positions, blocking=blocking, timeout=timeout)
        
        # Home the DC motors
        dc_success = self.motor.home_motors()
        
        if dc_success and blocking:
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.motor.get_status()
                
                if status is None:
                    return False
                
                if not status['pitch_moving'] and not status['linear_moving']:
                    self.current_pitch = status['pitch_position']
                    self.current_linear = status['linear_position']
                    break
                    
                time.sleep(0.05)
        
        return joints_success and dc_success
    
    def calibrate(self):
        """
        Perform calibration routine
        
        Returns:
            True if successful, False otherwise
        """
        # First disable torque
        self.enable_torque(False)
        
        # Then re-enable torque
        servo_success = self.enable_torque(True)
        
        # Home the motors
        return servo_success and self.home()
    
    def move_cartesian(self, x, y, z, speed=None):
        """
        Move the end effector to a Cartesian position (simplified)
        This is a very simplified implementation that maps coordinates directly to motors
        For a real implementation, inverse kinematics would be needed
        
        Args:
            x: X coordinate (maps to joint1)
            y: Y coordinate (maps to linear feed)
            z: Z coordinate (maps to pitch)
            speed: Movement speed
            
        Returns:
            True if successful, False otherwise
        """
        if speed is not None:
            self.set_speeds(servo_speed=speed, pitch_speed=speed, linear_speed=speed)
        
        # Map x to joint1 (base rotation)
        # This is a simplified mapping and would need proper inverse kinematics in real use
        joint1_pos = int(x * 10 + 2048)  # Simple scaling, 2048 is center
        
        # Map y to linear feed
        linear_pos = int(y * 100)
        
        # Map z to pitch
        pitch_pos = int(z * 100)
        
        # Execute the movements
        j1_success = self.set_joint_position('joint1', joint1_pos)
        linear_success = self.set_linear_position(linear_pos)
        pitch_success = self.set_pitch_position(pitch_pos)
        
        return j1_success and linear_success and pitch_success 
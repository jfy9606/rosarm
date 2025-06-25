#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from arm_trajectory.msg import TrajectoryPath, TrajectoryPoint
from arm_trajectory.srv import PlanTrajectory, PlanTrajectoryRequest, PlanTrajectoryResponse
import sys
import os
# 获取脚本的目录
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)
# 导入鲸鱼优化器
try:
    # 直接导入本地文件
    import sys
    import os
    # 获取当前脚本路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 将当前目录添加到sys.path
    if current_dir not in sys.path:
        sys.path.append(current_dir)
    # 直接导入本地文件
from whales_optimizer import WhaleOptimizer, forward_kinematics_dh
except ImportError as e:
    rospy.logerr(f"导入WhaleOptimizer失败: {e}")
    sys.exit(1)
from math import pi
import tf.transformations

class ArmTrajectoryBridge:
    """Bridge node for connecting stereo vision with trajectory planning"""
    
    def __init__(self):
        rospy.init_node('arm_trajectory_bridge', anonymous=True)
        
        # Initialize robot arm parameters
        self.setup_robot_params()
        
        # Initialize TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Setup services
        self.plan_srv = rospy.Service('plan_arm_trajectory', PlanTrajectory, self.handle_plan_trajectory)
        
        # Publishers
        self.trajectory_pub = {}
        for arm_id in self.arm_configs:
            self.trajectory_pub[arm_id] = rospy.Publisher(
                f'/{arm_id}/trajectory_command', JointTrajectory, queue_size=1)
        
        # Subscribe to detected objects
        self.detected_poses_sub = rospy.Subscriber(
            '/stereo_vision/detected_poses', PoseArray, self.handle_detected_poses)
        
        # Subscriber for commands
        self.command_sub = rospy.Subscriber('/arm_command', String, self.handle_command)
        
        # Store detected objects and target positions
        self.detected_objects = {}
        self.target_positions = {}
        
        # Initialize the whale optimizer for each arm
        self.optimizers = {}
        for arm_id, config in self.arm_configs.items():
            self.optimizers[arm_id] = WhaleOptimizer(
                num_whales=30,
                max_iter=100,
                forward_kinematics_func=lambda joints, arm=arm_id: self.forward_kinematics(joints, arm),
                joint_limits=config['joint_limits']
            )
        
        rospy.loginfo("Trajectory Bridge Node initialized")
    
    def setup_robot_params(self):
        """Set up parameters for each robot arm in the system"""
        # This would typically be loaded from URDF or parameter server
        # For now, we'll hardcode configurations for multiple arms
        
        self.arm_configs = {
            'arm1': {
                'dh_params': [
                    ('revolute', 0, 0, 13, pi/2),        # Joint 1: (type, d, θ, a, α)
                    ('prismatic', 0, pi/4, 0, 0),        # Joint 2
                    ('revolute', 13, 0, 0, pi/2),        # Joint 3
                    ('revolute', 0, 0, 25, pi/2),        # Joint 4
                    ('revolute', 0, pi/2, 0, pi/2),      # Joint 5 (fixed)
                    ('prismatic', 0, pi/2, 0, 0)         # Joint 6
                ],
                'joint_limits': [
                    [-pi, pi],      # theta1 (rad)
                    [0, 43],        # d2 (cm)
                    [-pi/2, pi/2],  # theta3 (rad)
                    [0, pi],        # theta4 (rad)
                    [pi/2, pi/2],   # theta5 (rad, fixed)
                    [5, 15]         # d6 (cm)
                ],
                'default_pose': [0, 0, 0, 0, pi/2, 10]  # Default joint configuration
            },
            'arm2': {
                'dh_params': [
                    ('revolute', 0, 0, 13, pi/2),
                    ('prismatic', 0, pi/4, 0, 0),
                    ('revolute', 13, 0, 0, pi/2),
                    ('revolute', 0, 0, 25, pi/2),
                    ('revolute', 0, pi/2, 0, pi/2),
                    ('prismatic', 0, pi/2, 0, 0)
                ],
                'joint_limits': [
                    [-pi, pi],
                    [0, 43],
                    [-pi/2, pi/2],
                    [0, pi],
                    [pi/2, pi/2],
                    [5, 15]
                ],
                'default_pose': [0, 0, 0, 0, pi/2, 10]
            }
        }
    
    def forward_kinematics(self, joint_values, arm_id):
        """Calculate forward kinematics for the specified arm"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return np.eye(4)  # Return identity matrix on error
        
        return forward_kinematics_dh(joint_values, self.arm_configs[arm_id]['dh_params'])
    
    def inverse_kinematics(self, target_pose, arm_id, initial_joints=None):
        """Use WOA to solve inverse kinematics for the specified arm"""
        if arm_id not in self.optimizers:
            rospy.logerr(f"No optimizer for arm ID: {arm_id}")
            return None, float('inf')
        
        # If no initial guess, use the default pose
        if initial_joints is None:
            initial_joints = self.arm_configs[arm_id]['default_pose']
        
        # Run the optimizer
        best_joints, best_fitness, _ = self.optimizers[arm_id].optimize(target_pose, initial_joints)
        
        return best_joints, best_fitness
    
    def generate_trajectory(self, start_joints, end_joints, execution_time, arm_id, num_points=50):
        """
        Generate a smooth trajectory from start to end joint positions
        using cubic interpolation
        """
        trajectory = TrajectoryPath()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.arm_id = arm_id
        trajectory.execution_time = execution_time
        
        start_joints = np.array(start_joints)
        end_joints = np.array(end_joints)
        
        # Create time points
        times = np.linspace(0, execution_time, num_points)
        dt = execution_time / (num_points - 1)
        
        # Calculate velocities at endpoints (zero for start and end)
        start_vel = np.zeros_like(start_joints)
        end_vel = np.zeros_like(end_joints)
        
        # Calculate cubic polynomial coefficients
        a0 = start_joints
        a1 = start_vel
        a2 = 3 * (end_joints - start_joints) / execution_time**2 - 2 * start_vel / execution_time - end_vel / execution_time
        a3 = 2 * (start_joints - end_joints) / execution_time**3 + (start_vel + end_vel) / execution_time**2
        
        # Generate trajectory points
        for t in times:
            # Calculate joint positions using cubic polynomial
            q = a0 + a1 * t + a2 * t**2 + a3 * t**3
            
            # Calculate joint velocities
            qd = a1 + 2 * a2 * t + 3 * a3 * t**2
            
            # Calculate joint accelerations
            qdd = 2 * a2 + 6 * a3 * t
            
            # Create a trajectory point
            point = TrajectoryPoint()
            point.header.stamp = rospy.Time.now() + rospy.Duration(t)
            point.joint_positions = q.tolist()
            point.joint_velocities = qd.tolist()
            point.joint_accelerations = qdd.tolist()
            
            # Calculate the end effector pose for this joint configuration
            T = self.forward_kinematics(q, arm_id)
            
            # Convert T to a pose
            pose = Pose()
            pose.position.x = T[0, 3]
            pose.position.y = T[1, 3]
            pose.position.z = T[2, 3]
            
            # Simple conversion of rotation matrix to quaternion
            quat = tf.transformations.quaternion_from_matrix(T)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            
            point.pose = pose
            trajectory.points.append(point)
        
        return trajectory
    
    def trajectory_to_joint_trajectory(self, trajectory):
        """Convert our TrajectoryPath to ROS JointTrajectory message"""
        joint_traj = JointTrajectory()
        joint_traj.header = trajectory.header
        
        # Set joint names based on the arm type
        if trajectory.arm_id in self.arm_configs:
            joint_traj.joint_names = [f"{trajectory.arm_id}_joint{i+1}" for i in range(len(self.arm_configs[trajectory.arm_id]['joint_limits']))]
        
        # Convert points
        for point in trajectory.points:
            jtp = JointTrajectoryPoint()
            jtp.positions = point.joint_positions
            jtp.velocities = point.joint_velocities
            jtp.accelerations = point.joint_accelerations
            jtp.time_from_start = point.header.stamp - trajectory.header.stamp
            joint_traj.points.append(jtp)
        
        return joint_traj
    
    def handle_plan_trajectory(self, req):
        """Service handler to plan a trajectory"""
        response = PlanTrajectoryResponse()
        
        # Check if requested arm exists
        if req.arm_id not in self.arm_configs:
            response.success = False
            response.message = f"Unknown arm ID: {req.arm_id}"
            return response
        
        try:
            # Get current joint state (would normally come from joint state publisher)
            current_joints = self.arm_configs[req.arm_id]['default_pose']
            
            # Create target transformation matrix from provided pose
            target_pose = np.eye(4)
            
            # Set the position part
            target_pose[0:3, 3] = [
                req.target_pose.position.x,
                req.target_pose.position.y,
                req.target_pose.position.z
            ]
            
            # Set the orientation part (quaternion to rotation matrix)
            q = [
                req.target_pose.orientation.x,
                req.target_pose.orientation.y,
                req.target_pose.orientation.z,
                req.target_pose.orientation.w
            ]
            target_pose[0:3, 0:3] = tf.transformations.quaternion_matrix(q)[0:3, 0:3]
            
            # Compute inverse kinematics
            target_joints, fitness = self.inverse_kinematics(target_pose, req.arm_id, current_joints)
            
            if fitness > 0.1:  # Threshold for acceptable IK solution
                response.success = False
                response.message = f"Could not find a valid IK solution. Fitness: {fitness}"
                return response
            
            # Generate trajectory
            trajectory = self.generate_trajectory(
                current_joints, target_joints, req.execution_time, req.arm_id)
            
            response.success = True
            response.message = "Trajectory planning successful"
            response.trajectory = trajectory
            
            return response
            
        except Exception as e:
            response.success = False
            response.message = f"Error planning trajectory: {str(e)}"
            return response
    
    def handle_detected_poses(self, msg):
        """Handle detected object poses from stereo vision"""
        # In a real system, we'd need to identify which detection corresponds to which object
        # For now, just store all the poses and assume they're identified properly elsewhere
        self.detected_objects = {}
        
        # Store detected objects with unique IDs (just using indices for now)
        for i, pose in enumerate(msg.poses):
            object_id = f"object_{i}"
            self.detected_objects[object_id] = pose
            
            # Log detection
            rospy.loginfo(f"Detected object {object_id} at position: [{pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}]")
    
    def handle_command(self, msg):
        """Handle text commands for controlling the arms"""
        cmd = msg.data.strip().lower()
        parts = cmd.split()
        
        if len(parts) < 1:
            return
        
        if parts[0] == "pick" and len(parts) >= 3:
            # Format: "pick [arm_id] [object_id]"
            arm_id = parts[1]
            object_id = parts[2]
            self.execute_pick_command(arm_id, object_id)
            
        elif parts[0] == "place" and len(parts) >= 5:
            # Format: "place [arm_id] [x] [y] [z]"
            arm_id = parts[1]
            try:
                x = float(parts[2])
                y = float(parts[3])
                z = float(parts[4])
                self.execute_place_command(arm_id, x, y, z)
            except ValueError:
                rospy.logerr("Invalid coordinates in place command")
                
        elif parts[0] == "home" and len(parts) >= 2:
            # Format: "home [arm_id]"
            arm_id = parts[1]
            self.move_to_home(arm_id)
            
        else:
            rospy.logwarn(f"Unknown command: {cmd}")
    
    def execute_pick_command(self, arm_id, object_id):
        """Execute a pick command for the specified arm and object"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return
        
        if object_id not in self.detected_objects:
            rospy.logerr(f"Unknown object ID: {object_id}")
            return
        
        # Get target pose (the detected object's position)
        target_pose = self.detected_objects[object_id]
        
        # Create service request
        req = PlanTrajectoryRequest()
        req.arm_id = arm_id
        req.target_pose = target_pose
        req.execution_time = 5.0  # 5 seconds execution time
        req.avoid_obstacles = True
        
        # Call our own service handler
        response = self.handle_plan_trajectory(req)
        
        if response.success:
            # Convert to joint trajectory and publish
            joint_traj = self.trajectory_to_joint_trajectory(response.trajectory)
            self.trajectory_pub[arm_id].publish(joint_traj)
            rospy.loginfo(f"Executing pick command for arm {arm_id} to object {object_id}")
        else:
            rospy.logerr(f"Failed to plan trajectory: {response.message}")
    
    def execute_place_command(self, arm_id, x, y, z):
        """Execute a place command for the specified arm to the given position"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return
        
        # Create target pose
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0  # Default orientation (no rotation)
        
        # Create service request
        req = PlanTrajectoryRequest()
        req.arm_id = arm_id
        req.target_pose = target_pose
        req.execution_time = 5.0  # 5 seconds execution time
        req.avoid_obstacles = True
        
        # Call our own service handler
        response = self.handle_plan_trajectory(req)
        
        if response.success:
            # Convert to joint trajectory and publish
            joint_traj = self.trajectory_to_joint_trajectory(response.trajectory)
            self.trajectory_pub[arm_id].publish(joint_traj)
            rospy.loginfo(f"Executing place command for arm {arm_id} to position [{x}, {y}, {z}]")
        else:
            rospy.logerr(f"Failed to plan trajectory: {response.message}")
    
    def move_to_home(self, arm_id):
        """Move the specified arm to its home position"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return
        
        # Get home joint configuration
        home_joints = self.arm_configs[arm_id]['default_pose']
        
        # Get current joint state (in a real system this would come from joint state publisher)
        current_joints = home_joints  # Placeholder, would be replaced with actual current state
        
        # Generate trajectory to home position
        trajectory = self.generate_trajectory(current_joints, home_joints, 3.0, arm_id)
        
        # Convert to joint trajectory and publish
        joint_traj = self.trajectory_to_joint_trajectory(trajectory)
        self.trajectory_pub[arm_id].publish(joint_traj)
        rospy.loginfo(f"Moving arm {arm_id} to home position")

if __name__ == '__main__':
    try:
        node = ArmTrajectoryBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 
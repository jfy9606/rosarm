#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    motor_dir = get_package_share_directory('motor')
    servo_dir = get_package_share_directory('servo')
    trajectory_dir = get_package_share_directory('trajectory')
    gui_dir = get_package_share_directory('gui')
    vision_dir = get_package_share_directory('vision')

    # Launch configuration variables
    motor_port = LaunchConfiguration('motor_port')
    servo_port = LaunchConfiguration('servo_port')
    motor_baudrate = LaunchConfiguration('motor_baudrate')
    servo_baudrate = LaunchConfiguration('servo_baudrate')
    use_camera = LaunchConfiguration('use_camera')

    # Declare the launch arguments
    declare_motor_port_cmd = DeclareLaunchArgument(
        'motor_port',
        default_value='/dev/ttyUSB0',
        description='Motor control serial port')

    declare_servo_port_cmd = DeclareLaunchArgument(
        'servo_port',
        default_value='/dev/ttyUSB1',
        description='Servo control serial port')

    declare_motor_baudrate_cmd = DeclareLaunchArgument(
        'motor_baudrate',
        default_value='115200',
        description='Motor serial baudrate')

    declare_servo_baudrate_cmd = DeclareLaunchArgument(
        'servo_baudrate',
        default_value='1000000',
        description='Servo serial baudrate')

    declare_use_camera_cmd = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Whether to use camera')

    # Include motor control launch file
    include_motor_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motor_dir, 'launch', 'motor_control.launch.py')),
        launch_arguments={
            'port': motor_port,
            'baudrate': motor_baudrate
        }.items()
    )

    # Define launch nodes
    wrist_node_cmd = Node(
        package='servo',
        executable='wrist_node',
        name='wrist_node',
        output='screen',
        parameters=[{
            'port': servo_port,
            'baudrate': servo_baudrate
        }]
    )

    vacuum_node_cmd = Node(
        package='servo',
        executable='vacuum_node',
        name='vacuum_node',
        output='screen',
        parameters=[{
            'port': servo_port,
            'baudrate': servo_baudrate
        }]
    )

    arm_node_cmd = Node(
        package='servo',
        executable='arm_node',
        name='arm_node',
        output='screen',
        parameters=[{
            'motor_port': motor_port,
            'servo_port': servo_port,
            'motor_baudrate': motor_baudrate,
            'servo_baudrate': servo_baudrate
        }]
    )

    trajectory_node_cmd = Node(
        package='trajectory',
        executable='trajectory_node.py',
        name='trajectory_node',
        output='screen'
    )

    path_planner_node_cmd = Node(
        package='trajectory',
        executable='path_planner_node.py',
        name='path_planner_node',
        output='screen'
    )

    joint_state_publisher_cmd = Node(
        package='gui',
        executable='joint_state_publisher_node',
        name='joint_state_publisher',
        output='screen'
    )

    gui_node_cmd = Node(
        package='gui',
        executable='main_window',
        name='gui_node',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_motor_port_cmd)
    ld.add_action(declare_servo_port_cmd)
    ld.add_action(declare_motor_baudrate_cmd)
    ld.add_action(declare_servo_baudrate_cmd)
    ld.add_action(declare_use_camera_cmd)

    # Add motor control launch
    ld.add_action(include_motor_launch_cmd)

    # Add servo nodes
    ld.add_action(wrist_node_cmd)
    ld.add_action(vacuum_node_cmd)
    ld.add_action(arm_node_cmd)

    # Add trajectory nodes
    ld.add_action(trajectory_node_cmd)
    ld.add_action(path_planner_node_cmd)

    # Add GUI nodes
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(gui_node_cmd)

    # Add camera nodes if enabled
    # Conditionally include camera launch files
    # This would use a LaunchConfiguration to conditionally launch
    # based on the use_camera parameter

    return ld 
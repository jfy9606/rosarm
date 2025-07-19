#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher', default='true')
    
    # Declare launch arguments
    declare_use_joint_state_publisher = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value=use_joint_state_publisher,
        description='Whether to launch the joint state publisher')
    
    # GUI main window node
    gui_node = Node(
        package='gui',
        executable='tkgui_main.py',
        name='gui_node',
        output='screen'
    )
    
    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='gui',
        executable='joint_state_publisher.py',
        name='joint_state_publisher',
        output='screen',
        condition=LaunchConfiguration('use_joint_state_publisher')
    )
    
    return LaunchDescription([
        declare_use_joint_state_publisher,
        gui_node,
        joint_state_publisher_node
    ]) 
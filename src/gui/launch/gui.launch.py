#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Declare launch arguments
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value=use_rviz,
        description='Whether to launch RViz')
    
    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='gui',
        executable='joint_state_publisher_node',
        name='joint_state_publisher',
        output='screen'
    )
    
    # GUI main window node
    gui_node = Node(
        package='gui',
        executable='main_window',
        name='gui_node',
        output='screen'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=LaunchConfiguration('use_rviz')
    )
    
    return LaunchDescription([
        declare_use_rviz_arg,
        joint_state_publisher_node,
        gui_node,
        rviz_node
    ]) 
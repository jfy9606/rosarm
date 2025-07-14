from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor',
            executable='motor_node',
            name='motor_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'device_port': '/dev/ttyUSB0'},
                {'baud_rate': 115200}
            ]
        )
    ]) 
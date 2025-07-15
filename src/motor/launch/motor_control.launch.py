from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    motor_port = LaunchConfiguration('motor_port')
    motor_baudrate = LaunchConfiguration('motor_baudrate')
    config_file = LaunchConfiguration('config_file')
    
    # 声明参数
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'motor_port',
            default_value='/dev/ttyUSB0',
            description='电机控制串口设备'
        ),
        
        DeclareLaunchArgument(
            'motor_baudrate',
            default_value='115200',
            description='电机串口波特率'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='硬件配置文件路径'
        ),
        
        # 电机控制节点
        Node(
            package='motor',
            executable='motor_node_main',
            name='motor_node',
            output='screen',
            parameters=[{
                'port': motor_port,
                'baudrate': motor_baudrate,
                'config_file': config_file
            }]
        )
    ]) 
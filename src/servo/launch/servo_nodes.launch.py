from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    servo_port = LaunchConfiguration('servo_port')
    servo_baudrate = LaunchConfiguration('servo_baudrate')
    vacuum_id = LaunchConfiguration('vacuum_id')
    config_file = LaunchConfiguration('config_file')
    
    # 声明参数
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'servo_port',
            default_value='/dev/ttyUSB1',
            description='舵机控制串口设备'
        ),
        
        DeclareLaunchArgument(
            'servo_baudrate',
            default_value='1000000',
            description='舵机串口波特率'
        ),
        
        DeclareLaunchArgument(
            'vacuum_id',
            default_value='10',
            description='真空吸盘舵机ID'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='硬件配置文件路径'
        ),
        
        # 腕部舵机控制节点
        Node(
            package='servo',
            executable='wrist_node_main',
            name='wrist_node',
            output='screen',
            parameters=[{
                'port': servo_port,
                'baudrate': servo_baudrate,
                'config_file': config_file
            }]
        ),
        
        # 真空吸盘控制节点
        Node(
            package='servo',
            executable='vacuum_node_main',
            name='vacuum_node',
            output='screen',
            parameters=[{
                'port': servo_port,
                'baudrate': servo_baudrate,
                'vacuum_id': vacuum_id,
                'config_file': config_file
            }]
        )
    ]) 
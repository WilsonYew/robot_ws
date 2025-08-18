from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_launch.py',
            name='ydlidar_driver',
            parameters=[{
                'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            }]
        )
    ])

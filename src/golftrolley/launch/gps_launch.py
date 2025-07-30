from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud': 9600
            }]
        )
    ])

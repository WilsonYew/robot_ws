from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ydlidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
    ydlidar_launch = os.path.join(ydlidar_pkg, 'launch', 'ydlidar_launch.py')

    return LaunchDescription([
        # Include another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch)
        ),
  ])
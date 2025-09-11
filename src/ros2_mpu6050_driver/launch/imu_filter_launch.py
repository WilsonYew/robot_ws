from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    imu_params = os.path.join(get_package_share_directory('mpu6050driver'),'params','imu_filter.yaml')
    ekf_params = os.path.join(get_package_share_directory('mpu6050driver'),'params','ekf.yaml')

    imu_filter = Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    remappings=[('imu/data_raw', '/imu/data_raw'),
                ('imu/data',     '/imu/data')],
    parameters=[imu_params],
    )

    ekf = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_params],
    )

    return LaunchDescription([
        imu_filter,
        ekf
    ])
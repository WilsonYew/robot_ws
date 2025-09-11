#!/usr/bin/env python3
# ROS 2 Jazzy — Bringup via includes (Mode B)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time   = LaunchConfiguration('use_sim_time')
    ekf_yaml       = LaunchConfiguration('ekf_yaml')
    imu_gain       = LaunchConfiguration('imu_gain')
    imu_zeta       = LaunchConfiguration('imu_zeta')
    grid_range_max = LaunchConfiguration('grid_range_max')

    # 1) Your robot (URDF, TFs, ros2_control, etc.)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('golftrolley'), 'launch', 'launch_robot.launch.py'])
        ),
        # pass through any args your robot launch declares:
        launch_arguments={'use_ros2_control': 'true', 'sim_mode': 'false'}.items()
    )

    # 2) IMU driver + Madgwick + EKF (your earlier combo launch)
    imu_filter_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('mpu6050driver'), 'launch', 'imu_filter_launch.py'])
        ),
        # these only work if imu_filter_launch.py declares them
        launch_arguments={
            'use_sim_time': use_sim_time,
            'imu_gain': imu_gain,
            'imu_zeta': imu_zeta,
            'ekf_yaml': ekf_yaml,  # make sure your imu_filter_launch.py accepts an ekf_yaml arg
        }.items()
    )

    # 3) LiDAR bringup (replace with your driver’s actual launch file & args)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ydlidar_ros2_driver'), 'launch', 'ydlidar.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4) Stereo camera bringup (replace with your stereo launch file)
    stereo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('stereo_camera_node'), 'launch', 'stereo.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5) RTAB-Map (Mode B: use LiDAR grid, stereo VO, IMU)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'stereo': 'true',
            'approx_sync': 'true',
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'left_image_topic': '/left/image_rect',
            'right_image_topic': '/right/image_rect',
            'left_camera_info_topic': '/left/camera_info',
            'right_camera_info_topic': '/right/camera_info',
            'imu_topic': '/imu/filtered',
            'subscribe_scan': 'true',
            'rtabmap_args': (
                '--params '
                'Grid/FromDepth=false;'
                'Grid/RayTracing=true;'
                f'Grid/RangeMax={grid_range_max};'
                'Grid/MaxObstacleHeight=2.0;'
                'Vis/MinInliers=12;'
                'Kp/DetectorStrategy=6'
            ),
        }.items()
    )

    return LaunchDescription([
        # Global args (tweak at launch time)
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu_gain',     default_value='0.05'),
        DeclareLaunchArgument('imu_zeta',     default_value='0.01'),
        DeclareLaunchArgument('grid_range_max', default_value='8.0'),
        DeclareLaunchArgument(
            'ekf_yaml',
            default_value=PathJoinSubstitution([FindPackageShare('golftrolley'), 'config', 'ekf.yaml'])
        ),

        # Includes
        robot_launch,
        imu_filter_ekf,
        lidar_launch,
        stereo_launch,
        rtabmap_launch,
    ])


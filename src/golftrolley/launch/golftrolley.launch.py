#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --- IMU bringup (delay slightly to let USB enumerate) ---
    robot = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('golftrolley'),
                                       'launch', 'launch_robot.launch.py'])
            )
        )]
    )

    imu_driver = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('mpu6050driver'),
                                       'launch', 'mpu6050driver_launch.py'])
            )
        )]
    )

    imu_filter_ekf = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('mpu6050driver'),
                                       'launch', 'imu_filter_launch.py'])
            )
        )]
    )

    # --- Stereo camera (use parameters dict; add respawn) ---
    stereo_launch = TimerAction(
        period=6.0,
        actions=[Node(
            package='stereo_camera_node',
            executable='stereo_camera_node',
            name='stereo_camera_node',
            output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=[{
                'fps': 10,
                'width': 640,
                'height': 240,
                'encoding': 'mono8',
                'use_rectification': True
            }]
        )]
    )

    # --- LiDAR: stable port + explicit args + respawn; start last ---
    lidar_launch = TimerAction(
        period=8.0,   # give hub time; tune 6–10 s if needed
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ydlidar_ros2_driver'),
                                       'launch', 'ydlidar_launch.py'])
            ),
        )]
    )

    rtabmap = TimerAction(
        period=20.0,  # after stereo & lidar
        actions=[
            Node(
                package='rtabmap_odom',          # or 'rtabmap_ros' if that's where stereo_odometry is
                executable='stereo_odometry',
                name='stereo_vo',
                output='screen',
                parameters=[{
                    'approx_sync': True,
                    'queue_size': 50,
                    'publish_tf': False,
                    'frame_id': 'base_link',
                    # RTAB-Map params must be strings:
                    'Vis/MinInliers': '12',
                    'Kp/DetectorStrategy': '6',
                    'Mem/ImagePreDecimation': '2',
                }],
                remappings=[
                    ('left/image_rect','/left/image_rect'),
                    ('right/image_rect','/right/image_rect'),
                    ('left/camera_info','/left/camera_info'),
                    ('right/camera_info','/right/camera_info'),
                    # add IMU later once stable:
                    # ('imu','/imu/filtered'),
                    ('odom','/vo'),
                ],
            )
        ]
    )
    
    pkg = 'golftrolley'
    nav2_params = os.path.join(get_package_share_directory(pkg), 'config', 'nav2_params.yaml')

    nav2_stack = TimerAction(
        period=30.0,  # after lidar (8s) and VO (20s in your last snippet) — adjust so inputs exist
        actions=[
            # Map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[nav2_params],
            ),
            # AMCL
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params],
            ),
            # Planner, controller, smoother, behaviors, BT nav, waypoint follower
            Node(package='nav2_planner', executable='planner_server', name='planner_server',
                output='screen', parameters=[nav2_params]),
            Node(package='nav2_controller', executable='controller_server', name='controller_server',
                output='screen', parameters=[nav2_params],
                remappings=[('/cmd_vel','/cmd_vel')]),  # Nav2 -> twist_mux (your mux listens to /cmd_vel)
            Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
                output='screen', parameters=[nav2_params]),
            Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
                output='screen', parameters=[nav2_params]),
            Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
                output='screen', parameters=[nav2_params]),
            Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
                output='screen', parameters=[nav2_params]),
            # Lifecycle manager
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                output='screen', parameters=[nav2_params]),
        ]
    )

    return LaunchDescription([
        robot,
        imu_driver,
        imu_filter_ekf,
        stereo_launch,
        lidar_launch,
        rtabmap,
        # nav2_stack,
    ])

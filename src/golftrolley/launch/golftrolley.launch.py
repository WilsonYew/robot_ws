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

    # --- Stereo camera (use parameters dict; add respawn) ---
    stereo_launch = TimerAction(
        period=20.0,
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

    disparity_launch = TimerAction(
        period=30.0,
        actions=[Node(
            package='stereo_image_proc', executable='disparity_node',
            parameters=[{'approximate_sync': True}],
            remappings=[
                ('left/image_rect',  '/left/image_rect'),
                ('right/image_rect', '/right/image_rect'),
                ('left/camera_info','/left/camera_info'),
                ('right/camera_info','/right/camera_info'),
                ('disparity',       '/stereo/disparity'),
            ],
            respawn=True
        )]
    )
    
    pkg = 'golftrolley'
    slam_params = os.path.join(get_package_share_directory(pkg), 'config', 'slam_toolbox.yaml')


    slam_online_async = TimerAction(
        period=40.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
            ])),
            launch_arguments={
                # Most builds accept these; see note below to list args on your system.
                'slam_params_file': slam_params,
                'use_sim_time': 'false',
                # 'namespace': '',           # optional
            }.items()
        )]
    )

    # nav2_params = os.path.join(get_package_share_directory(pkg), 'config', 'nav2_params.yaml')

    # nav2_stack = TimerAction(
    #     period=30.0,  # after lidar (8s) and VO (20s in your last snippet) — adjust so inputs exist
    #     actions=[
    #         # Planner, controller, smoother, behaviors, BT nav, waypoint follower
    #         Node(package='nav2_planner', executable='planner_server', name='planner_server',
    #             output='screen', parameters=[nav2_params]),
    #         Node(package='nav2_controller', executable='controller_server', name='controller_server',
    #             output='screen', parameters=[nav2_params],
    #             remappings=[('/cmd_vel','/cmd_vel')]),  # Nav2 -> twist_mux (your mux listens to /cmd_vel)
    #         Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
    #             output='screen', parameters=[nav2_params]),
    #         Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
    #             output='screen', parameters=[nav2_params]),
    #         Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
    #             output='screen', parameters=[nav2_params]),
    #         Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
    #             output='screen', parameters=[nav2_params]),
    #         # Lifecycle manager
    #         Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
    #             output='screen', parameters=[nav2_params]),
    #     ]
    # )

    return LaunchDescription([
        robot,
        imu_driver,
        imu_filter_ekf,
        stereo_launch,
        # disparity_launch,
        lidar_launch,
        # rtabmap,
        slam_online_async,
        # nav2_stack,
    ])

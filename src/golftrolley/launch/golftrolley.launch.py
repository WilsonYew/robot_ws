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
    mono_launch = TimerAction(
        period=10.0,
        actions=[Node(
            package="stereo_camera_node",          # <-- change if your package name differs
            executable="mono_camera_node",         # <-- must match your built executable
            name="mono_camera_node",
            output="screen",
            parameters=[{
                "device": '/dev/video0',          # adjust if needed
                "fps": 10,                 # open source at 30 fps
                "width": 320,                   # try 640x480 if your CPU can handle it
                "height": 240,
                "frame_id": 'camera_link_optical', # must match URDF
                "topic": '/image_rgb',
                "use_mjpeg": True,
            }],
        )]
    )


    pkg = 'golftrolley'
    slam_params = os.path.join(get_package_share_directory(pkg), 'config', 'slam_toolbox.yaml')


    slam_online_async = TimerAction(
        period=20.0,
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

    follow = TimerAction(
        period=30.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('golftrolley'), 'launch', 'follow.launch.py'
            ])),
        )]
    )

    return LaunchDescription([
        robot,
        mono_launch,
        lidar_launch,
        slam_online_async,
        follow,
    ])

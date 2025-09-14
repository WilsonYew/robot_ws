from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    disparity_launch = TimerAction(
        period=2.0,
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
    

    follow = TimerAction(
        period=10.0,  # start after disparity is alive
        actions=[Node(
            package='golftrolley', executable='stereo_person_follower.py', name='follower',
            parameters=[{
                'det_topic': '/detections',            # <-- your detector topic
                'disparity_topic': '/stereo/disparity',
                'camera_info_topic': '/left/camera_info',
                'cmd_vel_out': '/cmd_vel_tracker',
                'desired_distance': 1.6,
                'k_linear': 0.7,
                'k_angular': 1.2,
                'bbox_downsample': 4,
                'depth_percentile': 0.25,
                'min_valid_disp': 0.2
            }],
            respawn=True
        )]
    )

    return LaunchDescription([
        disparity_launch,
        follow,
    ])

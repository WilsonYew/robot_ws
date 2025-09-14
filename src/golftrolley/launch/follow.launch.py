from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    disparity_launch = Node(
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
        )
    

    follow = Node(
        package='golftrolley',
        executable='mono_lidar_follower.py',   # <-- include .py when using ament_cmake PROGRAMS
        name='follower',
        parameters=[{
            'det_topic': '/detections',
            'camera_info_topic': '/left/camera_info',
            'scan_topic': '/scan',
            'cmd_vel_out': '/cmd_vel_tracker',
            'desired_distance': 1.8,
            'k_linear': 0.5, 'k_angular': 1.1,
            'deadband_z': 0.12, 'deadband_th_deg': 1.8,
            'alpha': 0.25, 'v_max': 0.8, 'w_max': 1.2, 'v_min': 0.05, 'w_min': 0.15,
            'scan_angle_window_deg': 8.0, 'scan_min_range': 0.4, 'scan_max_range': 6.0,
        }],
        respawn=True
    )


    pkg = 'golftrolley'
    collision_params = os.path.join(get_package_share_directory(pkg), 'config', 'collision_monitor.yaml')
    smoother_params = os.path.join(get_package_share_directory(pkg), 'config', 'nav2_velocity_smoother.yaml')
    nav2_outdoor = os.path.join(get_package_share_directory(pkg), 'config', 'nav2_outdoor.yaml')

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=[collision_params],
        output='screen')



    smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[smoother_params],
        remappings=[('cmd_vel','/cmd_vel_stamped'),
                    ('smoothed_cmd_vel','/cmd_vel')],
        output='screen')

    to_diff = Node(
        package='topic_tools', executable='relay', name='cmd_vel_to_diff',
        arguments=['/cmd_vel', '/diff_cont/cmd_vel'],  # simple relay
        output='screen'
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['velocity_smoother', 'collision_monitor']
        }]
    )

    yolo_v8 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch',
                'yolov8.launch.py'
            ])
        ),
        launch_arguments={
            'input_image_topic': '/left/image_rect',   # matches yolo_bringup arg
            'model': 'yolov8n.pt',           # optional, defaults are fine
            'threshold': '0.35',
            'yolo_encoding': 'mono8',        # only if your frames are mono8
        }.items()
    )

    yolo_delayed = TimerAction(period=8.0, actions=[yolo_v8])
    follower_delayed = TimerAction(period=15.0, actions=[follow])

    return LaunchDescription([
        disparity_launch,

        collision_monitor,
        smoother,
        lifecycle_mgr,
        to_diff,
        # follow,
        # yolo_v8
        follower_delayed,
        yolo_delayed,
        # nav2,
    ])

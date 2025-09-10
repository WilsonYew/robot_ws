from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os

def generate_launch_description():
    # Paths
    pkg_share = os.path.join(os.getenv('HOME'), 'robot_ws', 'install')  # adjust
    urdf = os.path.join(os.getenv('HOME'), 'robot_ws', 'src', 'your_pkg', 'urdf', 'robot.urdf')
    nav2_params = os.path.join(os.getenv('HOME'), 'robot_ws', 'src', 'your_pkg', 'config', 'nav2_params.yaml')
    rtabmap_launch = PythonLaunchDescriptionSource(
        os.path.join('/opt/ros/jazzy/share/rtabmap_launch/launch/rtabmap.launch.py')
    )
    nav2_bringup = PythonLaunchDescriptionSource(
        os.path.join('/opt/ros/jazzy/share/nav2_bringup/launch/bringup_launch.py')
    )

    # Joystick + teleop
    joy = Node(package='joy', executable='joy_node', name='joy_node', output='screen')
    teleop = Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy',
        parameters=[{'axis_linear.x': 1, 'scale_linear.x': 0.6,
                     'axis_angular.yaw': 0, 'scale_angular.yaw': 1.5,
                     'enable_button': 4, 'enable_turbo_button': 5}]
    )

    # Robot description + TF
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf).read()}], output='screen'
    )
    # Example static TF for lidar -> base_link (adjust numbers & frame names)
    static_tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.0','0.0','0.2','0','0','0','base_link','laser_frame']
    )

    # LiDAR driver (change to your driver & params)
    lidar = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
        name='ydlidar', output='screen',
        parameters=[{'frame_id': 'laser_frame'}]
    )

    # RTAB-Map (stereo example; set your topics!)
    rtabmap = IncludeLaunchDescription(
        rtabmap_launch,
        launch_arguments={
            'stereo': 'true',
            'approx_sync': 'true',
            'frame_id': 'base_link',
            'left_image_topic': '/left/image_rect',
            'right_image_topic': '/right/image_rect',
            'left_camera_info_topic': '/left/camera_info',
            'right_camera_info_topic': '/right/camera_info',
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            'rtabmap_args': '--params,Grid/Sensor=1,Grid/RayTracing=true,Grid/RangeMax=12.0,Grid/MaxObstacleHeight=1.5',
            'map_always_update': 'true'
        }.items()
    )

    # Nav2 (uses map from RTAB-Map and laser for costmaps)
    nav2 = IncludeLaunchDescription(
        nav2_bringup,
        launch_arguments={
            'use_sim_time': 'false',
            'slam': 'False',  # because RTAB-Map is doing SLAM already
            'params_file': nav2_params
        }.items()
    )

    return LaunchDescription([joy, teleop, rsp, static_tf_laser, lidar, rtabmap, nav2])
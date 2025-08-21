from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_camera_split',
            executable='stereo_split_camera_node',
            name='stereo_split_camera_node',
            parameters=[
                {'device': '/dev/video0'},
                {'width': 1280},
                {'height': 480},
                {'frame_rate': 30},
                {'left_camera_info_path': '/home/golftrolley/robot_ws/src/stereo_camera_split/params/left_camera.yaml'},
                {'right_camera_info_path': '//home/golftrolley/robot_ws/src/stereo_camera_split/params/right_camera.yaml'}
            ]
        )
    ])

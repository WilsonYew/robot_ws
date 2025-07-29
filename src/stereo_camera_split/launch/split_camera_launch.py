from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_camera_split',
            executable='stereo_split_camera_node',
            name='stereo_split_camera_node',
            parameters=[
                {'device': '/dev/video2'},
                {'width': 2560},
                {'height': 720},
                {'frame_rate': 30},
                {'left_camera_info_path': '/path/to/left.yaml'},
                {'right_camera_info_path': '/path/to/right.yaml'}
            ]
        )
    ])

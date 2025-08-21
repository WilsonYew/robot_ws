from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # V4L2 camera driver (publishes /camera/image_raw)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            namespace='camera',
            parameters=[
                {"video_device": "/dev/video0"},
                {"image_size": [1280, 480]},   # Full stereo frame
                {"pixel_format": "YUYV"},      # Force YUYV (avoids MJPG conversion lag)
                {"frame_rate": 30},
                {"use_compressed_depth": False},  # turn off depth transport
                {"use_compressed": False}        # optional: disable JPEG too
            ],
            output="screen"
        ),

        # Stereo image splitter
        Node(
            package='stereo_camera_split',
            executable='stereo_split_camera_node',
            name='stereo_splitter',
            parameters=[
                {'left_camera_info_path': '/home/golftrolley/robot_ws/src/stereo_camera_split/params/left_camera.yaml'},
                {'right_camera_info_path': '/home/golftrolley/robot_ws/src/stereo_camera_split/params/right_camera.yaml'}
            ]
        )
    ])
# ROS 2 node: subscribes to a side-by-side stereo image and splits it into left/right

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import os

class StereoSplitter(Node):
    def __init__(self):
        super().__init__('stereo_splitter')

        # Parameters
        self.declare_parameter('left_camera_info_path', '/home/golftrolley/robot_ws/src/stereo_camera_split/params/left_camera.yaml')
        self.declare_parameter('right_camera_info_path', '/home/golftrolley/robot_ws/src/stereo_camera_split/params/right_camera.yaml')

        left_info_path = self.get_parameter('left_camera_info_path').get_parameter_value().string_value
        right_info_path = self.get_parameter('right_camera_info_path').get_parameter_value().string_value

        # Publishers
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera/right/camera_info', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Load calibration
        self.bridge = CvBridge()
        self.left_info = self.load_camera_info(left_info_path)
        self.right_info = self.load_camera_info(right_info_path)

        self.get_logger().info("Stereo Splitter Node started, waiting for /camera/image_raw")

    def load_camera_info(self, path):
        if not os.path.exists(path):
            self.get_logger().warn(f"CameraInfo YAML not found: {path}")
            return CameraInfo()

        with open(path, 'r') as file:
            calib_data = yaml.safe_load(file)

        info_msg = CameraInfo()
        info_msg.width = calib_data['image_width']
        info_msg.height = calib_data['image_height']
        info_msg.k = calib_data['camera_matrix']['data']
        info_msg.d = calib_data['distortion_coefficients']['data']
        info_msg.r = calib_data['rectification_matrix']['data']
        info_msg.p = calib_data['projection_matrix']['data']
        info_msg.distortion_model = calib_data['distortion_model']
        return info_msg

    def image_callback(self, msg: Image):
        # Convert ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w, _ = frame.shape
        half_w = w // 2
        left_img = frame[:, :half_w]
        right_img = frame[:, half_w:]

        # Convert back to ROS Image
        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')

        # Timestamp + frame_id
        stamp = msg.header.stamp
        left_msg.header.stamp = stamp
        right_msg.header.stamp = stamp
        left_msg.header.frame_id = 'camera_left'
        right_msg.header.frame_id = 'camera_right'

        # Fill camera info
        self.left_info.header.stamp = stamp
        self.right_info.header.stamp = stamp
        self.left_info.header.frame_id = 'camera_left'
        self.right_info.header.frame_id = 'camera_right'

        # Publish
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.left_info_pub.publish(self.left_info)
        self.right_info_pub.publish(self.right_info)

def main(args=None):
    rclpy.init(args=args)
    node = StereoSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
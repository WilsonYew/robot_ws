# stereo_split_camera_node.py
# ROS 2 Python node for splitting side-by-side stereo image into left and right images

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os

class StereoSplitCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_split_camera_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('left_camera_info_path', '')
        self.declare_parameter('right_camera_info_path', '')

        device = self.get_parameter('device').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, frame_rate)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))


        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera device: {device}")
            return

        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera/right/camera_info', 10)

        # Load calibration
        self.left_info = self.load_camera_info(self.get_parameter('left_camera_info_path').get_parameter_value().string_value)
        self.right_info = self.load_camera_info(self.get_parameter('right_camera_info_path').get_parameter_value().string_value)

        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

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

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        height, width, _ = frame.shape
        half_width = width // 2
        left_img = frame[:, :half_width]
        right_img = frame[:, half_width:]

        stamp = self.get_clock().now().to_msg()

        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')
        left_msg.header.stamp = stamp
        right_msg.header.stamp = stamp
        left_msg.header.frame_id = 'camera_left'
        right_msg.header.frame_id = 'camera_right'

        self.left_info.header.stamp = stamp
        self.right_info.header.stamp = stamp
        self.left_info.header.frame_id = 'camera_left'
        self.right_info.header.frame_id = 'camera_right'

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.left_info_pub.publish(self.left_info)
        self.right_info_pub.publish(self.right_info)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoSplitCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from stereo_msgs.msg import DisparityImage
from vision_msgs.msg import Detection2DArray

class FollowFromDisparity(Node):
    def __init__(self):
        super().__init__('follow_from_disparity')

        # Params you can tune at runtime with ros2 param set …
        self.declare_parameter('det_topic', '/detections')           # vision_msgs/Detection2DArray
        self.declare_parameter('disparity_topic', '/stereo/disparity')
        self.declare_parameter('camera_info_topic', '/left/camera_info')
        self.declare_parameter('cmd_vel_out', '/cmd_vel_tracker')

        self.declare_parameter('target_class_id', '')  # optional filter, empty = first detection
        self.declare_parameter('bbox_downsample', 4)   # sample every Nth pixel in bbox
        self.declare_parameter('depth_percentile', 0.3)# robust depth (0..1)
        self.declare_parameter('min_valid_disp', 0.1)  # drop near-zero disparities
        self.declare_parameter('desired_distance', 1.5)
        self.declare_parameter('max_lin_vel', 0.8)
        self.declare_parameter('max_ang_vel', 1.2)

        # Simple P gains (tune!)
        self.declare_parameter('k_linear', 0.7)
        self.declare_parameter('k_angular', 1.5)

        # Timeouts
        self.declare_parameter('det_timeout_sec', 0.75)
        self.declare_parameter('disp_timeout_sec', 0.5)

        # Cache
        self.fx = self.cx = None
        self.last_disp = None     # (stamp, disparity np.array, f, B)
        self.last_det_stamp = None

        # Subs / pubs
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').get_parameter_value().string_value,
            self._on_info, 1)

        self.sub_disp = self.create_subscription(
            DisparityImage,
            self.get_parameter('disparity_topic').get_parameter_value().string_value,
            self._on_disparity, 1)

        self.sub_det = self.create_subscription(
            Detection2DArray,
            self.get_parameter('det_topic').get_parameter_value().string_value,
            self._on_dets, 1)

        self.pub_cmd = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_out').get_parameter_value().string_value,
            10)

        self.timer = self.create_timer(0.05, self._on_timer)  # 20 Hz control

        self.get_logger().info('FollowFromDisparity ready.')

    # --- Callbacks ---
    def _on_info(self, msg: CameraInfo):
        self.fx = msg.k[0]  # fx
        self.cx = msg.k[2]  # cx

    def _on_disparity(self, msg: DisparityImage):
        # Grab f (px), B (m) from message; some builds use 't' or 'T'
        f = msg.f
        B = msg.t if hasattr(msg, 't') else msg.theta  # fallback if field name differs
        w, h = msg.image.width, msg.image.height
        # Convert embedded image buffer to float32 array
        # encoding is typically 32FC1
        disp = np.frombuffer(msg.image.data, dtype=np.float32).reshape(h, w)
        self.last_disp = (msg.header.stamp, disp, f, B)

    def _on_dets(self, msg: Detection2DArray):
        self.last_det_stamp = msg.header.stamp
        self.last_dets = msg.detections

    # --- Control loop ---
    def _on_timer(self):
        # prerequisites
        if self.fx is None or self.cx is None or self.last_disp is None:
            return

        # timeouts
        now = self.get_clock().now().to_msg()
        if self.last_det_stamp is None:
            return

        def age_ms(stamp):
            return (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e6

        if age_ms(self.last_det_stamp) > self.get_parameter('det_timeout_sec').value * 1000:
            return
        if age_ms(self.last_disp[0]) > self.get_parameter('disp_timeout_sec').value * 1000:
            return

        # choose detection
        target_id = self.get_parameter('target_class_id').get_parameter_value().string_value
        det = None
        for d in self.last_dets:
            if not d.results:
                continue
            cls_id = d.results[0].hypothesis.class_id
            if (not target_id) or (cls_id == target_id):
                det = d
                break
        if det is None:
            return

        # bbox
        cx_det = det.bbox.center.position.x
        cy_det = det.bbox.center.position.y
        w_det  = det.bbox.size.x
        h_det  = det.bbox.size.y

        stamp, disp, f_msg, B = self.last_disp
        if f_msg > 0:
            f = f_msg  # prefer per-message f
        else:
            f = self.fx

        # sample disparity inside bbox
        step = max(1, int(self.get_parameter('bbox_downsample').value))
        xmin = int(max(0, cx_det - w_det/2)); xmax = int(min(disp.shape[1], cx_det + w_det/2))
        ymin = int(max(0, cy_det - h_det/2)); ymax = int(min(disp.shape[0], cy_det + h_det/2))
        roi = disp[ymin:ymax:step, xmin:xmax:step].flatten()
        roi = roi[np.isfinite(roi)]
        roi = roi[roi > float(self.get_parameter('min_valid_disp').value)]
        if roi.size < 20:
            # not enough valid disparity
            return

        # robust depth: lower percentile to bias toward nearer person
        p = float(self.get_parameter('depth_percentile').value)
        d_med = np.quantile(roi, p)
        # Z = f * B / d
        if d_med <= 0 or B <= 0:
            return
        Z = (f * B) / d_med

        # bearing from bbox center pixel
        u = cx_det
        # pinhole: x_cam = (u - cx)/fx * Z, small-angle yaw ≈ atan((u-cx)/fx)
        yaw = np.arctan2((u - self.cx), self.fx)

        # P-control
        desired = float(self.get_parameter('desired_distance').value)
        k_lin = float(self.get_parameter('k_linear').value)
        k_ang = float(self.get_parameter('k_angular').value)

        v = k_lin * (Z - desired)
        w = k_ang * (-yaw)  # steer to center (negative to turn toward positive yaw)

        # saturate
        v = float(np.clip(v,
                          -abs(self.get_parameter('max_lin_vel').value),
                           abs(self.get_parameter('max_lin_vel').value)))
        w = float(np.clip(w,
                          -abs(self.get_parameter('max_ang_vel').value),
                           abs(self.get_parameter('max_ang_vel').value)))

        # slow down when turning hard
        v *= max(0.0, 1.0 - min(1.0, abs(w)/self.get_parameter('max_ang_vel').value))

        cmd = Twist()
        cmd.linear.x = max(0.0, v)  # no reverse in follower for safety; set negative allowed if you want
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(FollowFromDisparity())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# golftrolley/scripts/mono_lidar_follower.py
#
# Person-following controller using:
#   - Bearing from monocular detection bbox + CameraInfo
#   - Range from LiDAR sector around that bearing
#
# Publishes /cmd_vel_tracker (Twist) for your twist_mux chain.
#
# QoS is sensor-like (BestEffort/Volatile) to match your publishers.

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo, LaserScan
from geometry_msgs.msg import Twist


class MonoLidarFollower(Node):
    def __init__(self) -> None:
        super().__init__('follower')

        # ------------ Parameters ------------
        # Topics
        self.declare_parameter('det_topic', '/detections')              # Detection2DArray
        self.declare_parameter('camera_info_topic', '/left/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_out', '/cmd_vel_tracker')

        # Optional target filtering (by class_id). Empty = first detection.
        self.declare_parameter('target_class_id', '')

        # Control targets / limits
        self.declare_parameter('desired_distance', 1.8)   # meters
        self.declare_parameter('k_linear', 0.5)
        self.declare_parameter('k_angular', 1.1)
        self.declare_parameter('v_max', 0.8)
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('v_min', 0.05)             # overcome stiction
        self.declare_parameter('w_min', 0.15)

        # Deadbands + smoothing
        self.declare_parameter('deadband_z', 0.12)        # m
        self.declare_parameter('deadband_th_deg', 1.8)    # deg
        self.declare_parameter('alpha', 0.25)             # IIR 0..1 (lower=smoother)

        # LiDAR sampling window / limits
        self.declare_parameter('scan_angle_window_deg', 8.0)  # +/- degrees around bearing
        self.declare_parameter('scan_min_range', 0.4)
        self.declare_parameter('scan_max_range', 6.0)

        # Timeouts (drop commands if stale)
        self.declare_parameter('det_timeout_sec', 0.6)
        self.declare_parameter('scan_timeout_sec', 0.6)
        self.declare_parameter('info_timeout_sec', 2.0)

        # ------------ State ------------
        self.fx = None
        self.cx = None
        self.last_dets = None
        self.last_det_stamp = None
        self.last_scan = None
        self.last_scan_stamp = None

        # IIR filter state
        self.Z_prev = None
        self.theta_prev = None

        # ------------ QoS ------------
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ------------ Subs/Pubs ------------
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._on_info,
            qos_sensor
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            qos_sensor
        )
        self.sub_det = self.create_subscription(
            Detection2DArray,
            self.get_parameter('det_topic').value,
            self._on_dets,
            qos_sensor
        )

        self.pub_cmd = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_out').value,
            10
        )

        # 20 Hz loop
        self.timer = self.create_timer(0.05, self._on_timer)

        self.get_logger().info('Mono+LiDAR follower ready.')

    # ---------- Callbacks ----------
    def _on_info(self, msg: CameraInfo):
        # Prefer P (projection) if available; otherwise K
        if len(msg.p) >= 12 and msg.p[0] != 0.0:
            self.fx = float(msg.p[0])
            self.cx = float(msg.p[2])
        elif len(msg.k) >= 9 and msg.k[0] != 0.0:
            self.fx = float(msg.k[0])
            self.cx = float(msg.k[2])
        else:
            self.fx = self.cx = None
        # no timestamp gate here; caminfo changes rarely

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_stamp = msg.header.stamp

    def _on_dets(self, msg: Detection2DArray):
        self.last_dets = msg.detections
        self.last_det_stamp = msg.header.stamp

    # ---------- Helpers ----------
    def _age_sec(self, stamp) -> float:
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9

    def _range_from_scan(self, yaw_rad: float):
        """Robust LiDAR range around the given yaw (base_link frame)."""
        scan = self.last_scan
        if scan is None:
            return None

        # Clamp if outside FOV
        if yaw_rad < scan.angle_min or yaw_rad > scan.angle_max:
            return None

        ang_inc = scan.angle_increment
        idx_center = int((yaw_rad - scan.angle_min) / ang_inc)

        half_win = max(1, int(math.radians(
            float(self.get_parameter('scan_angle_window_deg').value)) / abs(ang_inc)))
        i0 = max(0, idx_center - half_win)
        i1 = min(len(scan.ranges) - 1, idx_center + half_win)

        rmin = float(self.get_parameter('scan_min_range').value)
        rmax = float(self.get_parameter('scan_max_range').value)
        window = [r for r in scan.ranges[i0:i1+1] if math.isfinite(r) and rmin <= r <= rmax]
        if not window:
            return None

        window.sort()
        return window[int(0.3 * len(window))]  # 30th percentile for robustness

    def _publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    # ---------- Control Loop ----------
    def _on_timer(self):
        # Need intrinsic fx/cx, recent dets & scan
        if self.fx is None or self.cx is None:
            # CameraInfo not received or invalid yet
            return

        det_age = self._age_sec(self.last_det_stamp)
        scan_age = self._age_sec(self.last_scan_stamp)
        if det_age > float(self.get_parameter('det_timeout_sec').value) or \
           scan_age > float(self.get_parameter('scan_timeout_sec').value):
            self._publish_stop()
            return

        # Choose a detection
        det = None
        target_id = self.get_parameter('target_class_id').value
        if self.last_dets:
            for d in self.last_dets:
                if not d.results:
                    continue
                cls = d.results[0].hypothesis.class_id
                if (not target_id) or (cls == target_id):
                    det = d
                    break
        if det is None:
            self._publish_stop()
            return

        # BBox center (pixel)
        u = det.bbox.center.position.x
        # Pinhole: yaw ≈ atan((u - cx)/fx) (optical frame ~ base yaw)
        yaw = math.atan2((u - self.cx), self.fx)

        # Range from LiDAR sector around bearing
        Z = self._range_from_scan(yaw)
        if Z is None:
            self._publish_stop()
            return

        # --------- Filtering & Control ---------
        alpha = float(self.get_parameter('alpha').value)
        if self.Z_prev is None:
            self.Z_prev = Z
        if self.theta_prev is None:
            self.theta_prev = yaw

        # IIR low-pass
        Z_f = alpha * Z + (1.0 - alpha) * self.Z_prev
        th_f = alpha * yaw + (1.0 - alpha) * self.theta_prev
        self.Z_prev, self.theta_prev = Z_f, th_f

        desired = float(self.get_parameter('desired_distance').value)
        err_z = Z_f - desired
        err_th = -th_f  # steer to center

        db_z = float(self.get_parameter('deadband_z').value)
        db_th = math.radians(float(self.get_parameter('deadband_th_deg').value))

        k_lin = float(self.get_parameter('k_linear').value)
        k_ang = float(self.get_parameter('k_angular').value)

        v = 0.0 if abs(err_z) < db_z else k_lin * err_z
        w = 0.0 if abs(err_th) < db_th else k_ang * err_th

        # Saturation + minimums
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        v_min = float(self.get_parameter('v_min').value)
        w_min = float(self.get_parameter('w_min').value)

        v = max(min(v, v_max), -v_max)
        w = max(min(w, w_max), -w_max)

        if 0.0 < v < v_min: v = v_min
        if -v_min < v < 0.0: v = -v_min
        if 0.0 < w < w_min: w = w_min
        if -w_min < w < 0.0: w = -w_min

        # Slow forward motion when turning
        if w_max > 0.0:
            v *= max(0.0, 1.0 - min(1.0, abs(w) / w_max))

        # Safety: forward only
        if v < 0.0:
            v = 0.0

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = MonoLidarFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

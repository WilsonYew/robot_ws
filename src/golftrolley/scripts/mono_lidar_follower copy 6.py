#!/usr/bin/env python3
# Mono camera person follower (YOLO only), robust to slow detections
# - Angular cmd from bbox center vs principal point (cx, fx)
# - Linear cmd from bbox pixel height vs target height (distance proxy)
# - Hold last command briefly when detections pause

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from yolo_msgs.msg import DetectionArray as YoloDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist


class MonoCamFollower(Node):
    def __init__(self):
        super().__init__('mono_cam_follower')

        # ---------- Parameters ----------
        # Topics
        self.declare_parameter('det_topic', '/yolo/detections')
        self.declare_parameter('camera_info_topic', '/left/camera_info')
        self.declare_parameter('cmd_vel_out', '/cmd_vel_tracker')

        # Intrinsic overrides (use these if CameraInfo is missing)
        self.declare_parameter('fx_override_px', 470.57825541925916)
        self.declare_parameter('cx_override_px', 160.105131149292)
        self.declare_parameter('img_width_override', 320)

        # Target selection
        self.declare_parameter('target_class_name', 'person')
        self.declare_parameter('min_score', 0.25)

        # Angular control (to center)
        self.declare_parameter('k_angular', 1.2)           # rad/s per rad error
        self.declare_parameter('deadband_th_deg', 2.0)     # deg
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('w_min', 0.15)

        # Linear control (distance proxy from bbox height)
        self.declare_parameter('target_box_height_px', 220.0)
        self.declare_parameter('k_linear', 0.003)          # m/s per pixel error
        self.declare_parameter('deadband_px', 8.0)         # px
        self.declare_parameter('v_max', 0.7)
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('forward_only', True)

        # Staleness, debug
        self.declare_parameter('debug', True)
        self.declare_parameter('det_timeout_sec', 3.0)     # tolerant for slow YOLO
        self.declare_parameter('info_timeout_sec', 2.0)

        # Robustness to slow detections
        self.declare_parameter('use_msg_stamp', False)      # False = use receive time
        self.declare_parameter('det_qos_reliability', 'best_effort')  # or 'reliable'
        self.declare_parameter('hold_on_stale', True)
        self.declare_parameter('hold_max_sec', 1.5)
        self.declare_parameter('hold_decay_lin', 0.98)      # linear decays slowly
        self.declare_parameter('hold_decay_ang', 0.85)      # angular decays faster

        # Turning slowdown controls
        self.declare_parameter('slow_on_turn', True)        # scale v by turn amount
        self.declare_parameter('v_turn_min', 0.0)           # keep at least this v while turning

        # ---------- State ----------
        self.fx = None
        self.cx = None
        self.img_width = None
        self.last_dets = None
        self.last_det_stamp = None
        self.last_debug_time = self.get_clock().now()

        # For "hold last command"
        self.last_cmd = Twist()
        self.last_cmd_time = None

        # ---------- QoS ----------
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        det_rel = str(self.get_parameter('det_qos_reliability').value).lower()
        qos_det = QoSProfile(
            depth=10,
            reliability=(ReliabilityPolicy.RELIABLE if det_rel == 'reliable'
                         else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---------- Subs / Pub ----------
        self.create_subscription(CameraInfo,
                                 self.get_parameter('camera_info_topic').value,
                                 self._on_info, qos_sensor)
        self.create_subscription(YoloDetectionArray,
                                 self.get_parameter('det_topic').value,
                                 self._on_dets, qos_det)
        self.pub_cmd = self.create_publisher(Twist,
                                             self.get_parameter('cmd_vel_out').value,
                                             10)

        self.timer = self.create_timer(0.05, self._on_timer)  # 20 Hz
        self.get_logger().info('Mono camera follower ready.')

    # ---------- Callbacks ----------
    def _on_info(self, msg: CameraInfo):
        # Prefer P (Projection) then K (Camera matrix)
        if len(msg.p) >= 12 and msg.p[0] != 0.0:
            self.fx = float(msg.p[0]); self.cx = float(msg.p[2])
        elif len(msg.k) >= 9 and msg.k[0] != 0.0:
            self.fx = float(msg.k[0]); self.cx = float(msg.k[2])
        else:
            self.fx = self.cx = None
        self.img_width = msg.width if hasattr(msg, 'width') and msg.width > 0 else None

    def _on_dets(self, msg: YoloDetectionArray):
        self.last_dets = msg.detections
        if bool(self.get_parameter('use_msg_stamp').value):
            self.last_det_stamp = msg.header.stamp
        else:
            self.last_det_stamp = self.get_clock().now().to_msg()  # receive time

    # ---------- Helpers ----------
    def _age_sec(self, stamp):
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9

    def _pick_person(self):
        want = self.get_parameter('target_class_name').value.strip().lower()
        min_score = float(self.get_parameter('min_score').value)
        if not self.last_dets:
            return None
        best, best_score = None, -1.0
        for d in self.last_dets:
            name = getattr(d, 'class_name', '')
            score = float(getattr(d, 'score', 0.0))
            if want and name.lower() != want:
                continue
            if score < min_score:
                continue
            if score > best_score:
                best, best_score = d, score
        return best

    def _dbg(self, txt):
        if not bool(self.get_parameter('debug').value):
            return
        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds > 0.2e9:  # ~5 Hz
            self.get_logger().info(txt)
            self.last_debug_time = now

    def _publish_stop(self):
        self.pub_cmd.publish(Twist())

    # ---------- Control loop ----------
    def _on_timer(self):
        now = self.get_clock().now()

        # Effective intrinsics (overrides first, else CameraInfo)
        fx_ovr = float(self.get_parameter('fx_override_px').value)
        cx_ovr = float(self.get_parameter('cx_override_px').value)
        img_w_ovr = int(self.get_parameter('img_width_override').value)

        fx = fx_ovr if fx_ovr > 0 else self.fx
        if cx_ovr >= 0:
            cx = cx_ovr
        else:
            width = self.img_width if (self.img_width and self.img_width > 0) else img_w_ovr
            cx = (width - 1) * 0.5 if width and width > 0 else None

        if fx is None or cx is None:
            self._dbg('WAIT: intrinsics not available (fx/cx)')
            return

        # Stale? (with hold support)
        age = self._age_sec(self.last_det_stamp)
        if age > float(self.get_parameter('det_timeout_sec').value):
            if bool(self.get_parameter('hold_on_stale').value) and self.last_cmd_time is not None:
                if (now - rclpy.time.Time.from_msg(self.last_cmd_time)).nanoseconds / 1e9 < float(self.get_parameter('hold_max_sec').value):
                    dec_lin = float(self.get_parameter('hold_decay_lin').value)
                    dec_ang = float(self.get_parameter('hold_decay_ang').value)
                    self.last_cmd.linear.x  *= dec_lin
                    self.last_cmd.angular.z *= dec_ang
                    self.pub_cmd.publish(self.last_cmd)
                    self._dbg('HOLD: reusing last cmd')
                    return
            self._dbg('STOP: detections stale')
            self._publish_stop()
            return

        d = self._pick_person()
        if d is None:
            self._dbg('STOP: no person above min_score')
            self._publish_stop()
            return

        # bbox center and size
        u = float(d.bbox.center.position.x)
        h = float(d.bbox.size.y)

        # ---- Angular control (use effective fx/cx) ----
        th_err = -math.atan2((u - cx), fx)
        w = float(self.get_parameter('k_angular').value) * th_err

        db_th = math.radians(float(self.get_parameter('deadband_th_deg').value))
        w_max = float(self.get_parameter('w_max').value)
        w_min = float(self.get_parameter('w_min').value)

        if abs(th_err) < db_th:
            w = 0.0
        w = max(min(w, w_max), -w_max)
        if 0.0 < w < w_min: w = w_min
        if -w_min < w < 0.0: w = -w_min

        # ---- Linear control ----
        h_tgt = float(self.get_parameter('target_box_height_px').value)
        px_err = h_tgt - h   # positive if person is too far (bbox too small)
        v = float(self.get_parameter('k_linear').value) * px_err

        db_px = float(self.get_parameter('deadband_px').value)
        v_max = float(self.get_parameter('v_max').value)
        v_min = float(self.get_parameter('v_min').value)
        forward_only = bool(self.get_parameter('forward_only').value)

        if abs(px_err) < db_px:
            v = 0.0
        v = max(min(v, v_max), -v_max)
        if forward_only and v < 0.0:
            v = 0.0
        if 0.0 < v < v_min:
            v = v_min

        # Optional: reduce forward speed when turning hard, with creep floor
        if bool(self.get_parameter('slow_on_turn').value) and w_max > 0.0:
            scale = 1.0 - min(1.0, abs(w) / w_max)
            v *= max(0.0, scale)
            v_turn_min = float(self.get_parameter('v_turn_min').value)
            if v > 0.0 and v < v_turn_min:
                v = v_turn_min

        # Publish + remember for hold
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        self.last_cmd = cmd
        self.last_cmd_time = now.to_msg()

        self._dbg(f'cmd v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} | '
                  f'u={u:.1f} h={h:.1f} | th_err={math.degrees(th_err):.1f}deg px_err={px_err:.1f}')


def main():
    rclpy.init()
    node = MonoCamFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

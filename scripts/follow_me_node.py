#!/usr/bin/env python3
"""
Follow-Me Node
==============
Tracks the nearest person using OAK-D Lite MobileNet-SSD spatial detections.
No OpenCV needed — 3D position comes directly from the OAK-D VPU.

Subscribes:
  /oak/nn/spatial_detections  vision_msgs/Detection3DArray  — person detections w/ 3D pose

Publishes:
  /cmd_vel                    geometry_msgs/Twist            — robot velocity (10 Hz)
  /follow_me/debug            std_msgs/String                — JSON status (5 Hz)

Services:
  /set_follow_me_mode         std_srvs/SetBool               — enable/disable

Behaviour:
  - Finds nearest person (MobileNet class_id '15') above confidence threshold
  - Angular PID: centres person laterally (target position.x = 0)
  - Linear PID:  maintains target_distance_m; stops if person < min_distance_m
  - Stops and waits if no person for > person_timeout_s
  - Disables autonomous nav mode (/motor/set_autonomous_mode false) on enable
  - Publishes zero velocity on disable or node exit

OAK camera frame (oak_rgb_camera_optical_frame):
  x  right (positive = person to right of robot)
  y  down
  z  forward / depth
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection3DArray

PERSON_CLASS_ID = '15'   # MobileNet-SSD COCO label 15 = 'person'


class FollowMeNode(Node):
    def __init__(self):
        super().__init__('follow_me_node')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('target_distance_m',  1.0)   # desired following distance
        self.declare_parameter('min_distance_m',     0.4)   # stop below this
        self.declare_parameter('person_timeout_s',   2.0)   # stop if no person this long
        self.declare_parameter('linear_kp',          0.3)   # m/s per metre distance error
        self.declare_parameter('angular_kp',         0.6)   # rad/s per metre lateral error
        self.declare_parameter('max_linear_ms',      0.20)  # cap (m/s)
        self.declare_parameter('max_angular_rs',     0.8)   # cap (rad/s)
        self.declare_parameter('min_confidence',     0.5)   # detection score threshold
        self.declare_parameter('control_hz',        10.0)

        self.target_distance = self.get_parameter('target_distance_m').value
        self.min_distance    = self.get_parameter('min_distance_m').value
        self.person_timeout  = self.get_parameter('person_timeout_s').value
        self.linear_kp       = self.get_parameter('linear_kp').value
        self.angular_kp      = self.get_parameter('angular_kp').value
        self.max_linear      = self.get_parameter('max_linear_ms').value
        self.max_angular     = self.get_parameter('max_angular_rs').value
        self.min_confidence  = self.get_parameter('min_confidence').value

        # ── state ────────────────────────────────────────────────────────────
        self.enabled           = False
        self.last_person_time  = None
        self.person_x          = None   # lateral offset (m) in camera frame
        self.person_z          = None   # depth / distance (m)
        self.person_confidence = 0.0

        # ── subscribers ──────────────────────────────────────────────────────
        det_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.det_sub = self.create_subscription(
            Detection3DArray, '/oak/nn/spatial_detections',
            self._on_detections, det_qos
        )

        # ── publishers ───────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub   = self.create_publisher(String, '/follow_me/debug', 10)

        # ── services ─────────────────────────────────────────────────────────
        self.mode_srv = self.create_service(
            SetBool, '/set_follow_me_mode', self._set_mode_cb
        )

        # Client to disable autonomous navigation when follow-me activates
        self.auto_mode_client = self.create_client(SetBool, '/motor/set_autonomous_mode')

        # ── timers ───────────────────────────────────────────────────────────
        hz = self.get_parameter('control_hz').value
        self.create_timer(1.0 / hz, self._control_loop)
        self.create_timer(0.2,      self._publish_debug)   # 5 Hz debug

        self.get_logger().info(
            'follow_me_node ready — call /set_follow_me_mode true to activate'
        )

    # ── detection callback ───────────────────────────────────────────────────

    def _on_detections(self, msg):
        """Select the nearest person above confidence threshold."""
        best_z = None
        best_x = None
        best_conf = 0.0

        for det in msg.detections:
            for result in det.results:
                if result.hypothesis.class_id != PERSON_CLASS_ID:
                    continue
                conf = result.hypothesis.score
                if conf < self.min_confidence:
                    continue
                z = result.pose.pose.position.z
                x = result.pose.pose.position.x
                if best_z is None or z < best_z:
                    best_z    = z
                    best_x    = x
                    best_conf = conf

        if best_z is not None:
            self.person_z          = best_z
            self.person_x          = best_x
            self.person_confidence = best_conf
            self.last_person_time  = time.time()

    # ── control loop ────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.enabled:
            return

        twist = Twist()
        now   = time.time()

        # No person detected recently — stop and wait
        if (self.last_person_time is None or
                (now - self.last_person_time) > self.person_timeout):
            self.cmd_vel_pub.publish(twist)
            return

        z = self.person_z
        x = self.person_x

        # Safety: too close — full stop
        if z < self.min_distance:
            self.cmd_vel_pub.publish(twist)
            return

        # Linear PID: close the distance error
        dist_err = z - self.target_distance
        if abs(dist_err) > 0.05:
            vx = self.linear_kp * dist_err
            twist.linear.x = max(-self.max_linear, min(self.max_linear, vx))

        # Angular PID: centre person in frame
        # positive x → person right of centre → turn right → negative angular.z
        if abs(x) > 0.05:
            wz = -self.angular_kp * x
            twist.angular.z = max(-self.max_angular, min(self.max_angular, wz))

        self.cmd_vel_pub.publish(twist)

    # ── debug publisher ─────────────────────────────────────────────────────

    def _publish_debug(self):
        now = time.time()
        person_detected = (self.last_person_time is not None and
                           (now - self.last_person_time) < self.person_timeout)
        msg = String()
        msg.data = json.dumps({
            'enabled':         self.enabled,
            'person_detected': person_detected,
            'distance_m':      round(self.person_z, 2) if self.person_z is not None else None,
            'lateral_m':       round(self.person_x, 2) if self.person_x is not None else None,
            'confidence':      round(self.person_confidence, 2),
        })
        self.debug_pub.publish(msg)

    # ── service handler ─────────────────────────────────────────────────────

    def _set_mode_cb(self, request, response):
        if request.data == self.enabled:
            response.success = True
            response.message = f'follow_me already {"enabled" if self.enabled else "disabled"}'
            return response

        self.enabled = request.data

        if self.enabled:
            # Turn off autonomous navigation so it doesn't compete for /cmd_vel
            if self.auto_mode_client.service_is_ready():
                req = SetBool.Request()
                req.data = False
                self.auto_mode_client.call_async(req)
            self.get_logger().info('Follow-me ENABLED')
        else:
            # Zero velocity immediately on disable
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Follow-me DISABLED')

        response.success = True
        response.message = f'follow_me {"enabled" if self.enabled else "disabled"}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())   # zero velocity on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

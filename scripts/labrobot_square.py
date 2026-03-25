#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class LabRobotSquare(Node):
    def __init__(self):
        super().__init__('labrobot_square')
        self.pub = self.create_publisher(Twist, '/model/labrobot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.update)
        self.state = 0
        self.start_time = time.time()
        self.phase_start = None
        self.twist = Twist()

        # Motion parameters (tune to match your robot)
        self.forward_speed = 0.5        # m/s
        self.turn_speed = 0.5           # rad/s
        self.side_duration = 10.0       # 5m / 0.5 m/s = 10 seconds to drive 5m
        self.turn_duration = 3.14 / 2 / self.turn_speed  # ~90 deg turn
        self.ramp_time = 1.0            # seconds for ramp up/down

    def ramped_value(self, t, total, max_val):
        # Trapezoidal profile: ramp up, hold, ramp down
        if t < self.ramp_time:
            return max_val * (t / self.ramp_time)
        elif t > total - self.ramp_time:
            return max_val * max(0.0, (total - t) / self.ramp_time)
        else:
            return max_val

    def update(self):
        now = time.time()
        if self.state >= 8:
            self.stop()
            return

        if self.state % 2 == 0:
            # Driving straight
            if self.phase_start is None:
                self.phase_start = now
                self.get_logger().info(f"Moving forward [{self.state//2 + 1}/4]")
            t = now - self.phase_start
            v = self.ramped_value(t, self.side_duration, self.forward_speed)
            self.twist.linear.x = v
            self.twist.angular.z = 0.0
            if t >= self.side_duration:
                self.phase_start = None
                self.state += 1
        else:
            # Turning 90 degrees
            if self.phase_start is None:
                self.phase_start = now
                self.get_logger().info("Turning 90 degrees")
            t = now - self.phase_start
            w = self.ramped_value(t, self.turn_duration, self.turn_speed)
            self.twist.linear.x = 0.0
            self.twist.angular.z = w
            if t >= self.turn_duration:
                self.phase_start = None
                self.state += 1

        self.pub.publish(self.twist)

    def stop(self):
        self.twist = Twist()
        self.pub.publish(self.twist)
        self.destroy_timer(self.timer)
        self.get_logger().info("Square path complete. Returned to origin.")


def main():
    rclpy.init()
    node = LabRobotSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

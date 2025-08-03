#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class BasicOdometryPublisher(Node):
    """
    A basic odometry publisher for real robot hardware.
    This provides static odometry at origin for testing purposes.
    In a real robot, this should be replaced with actual wheel encoder/IMU-based odometry.
    """

    def __init__(self):
        super().__init__('basic_odom_publisher')
        
        # Create publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer to publish odometry at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_odometry)
        
        self.get_logger().info('Basic odometry publisher started')

    def publish_odometry(self):
        current_time = self.get_clock().now()
        
        # Create odometry message (static at origin for basic testing)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position (static at origin)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        
        # Orientation (identity quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        
        # Velocity (zero for static odometry)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Broadcast transform (odom -> base_footprint)
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = BasicOdometryPublisher()
    
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

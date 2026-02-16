#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from gogo_interfaces.msg import EncoderFeedback
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.get_logger().info("OdometryNode initialized")

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Publisher for nav_msgs/Odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Subscriber to encoder feedback
        self.encoder_sub = self.create_subscription(EncoderFeedback, 'encoders', self.encoder_callback, sensor_qos)

        # Initialize state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.ticks_per_meter = 19858.10  # adjust to your robot

        self.prev_time = None

    def encoder_callback(self, msg: EncoderFeedback):
        if self.prev_left_ticks is None:
            # First message
            self.prev_left_ticks = msg.left_ticks
            self.prev_right_ticks = msg.right_ticks
            self.prev_time = self.get_clock().now()
            return

        # Compute change in ticks
        delta_left = msg.left_ticks - self.prev_left_ticks
        delta_right = msg.right_ticks - self.prev_right_ticks

        self.prev_left_ticks = msg.left_ticks
        self.prev_right_ticks = msg.right_ticks

        # Convert ticks to meters
        d_left = delta_left / self.ticks_per_meter
        d_right = delta_right / self.ticks_per_meter
        d_center = (d_left + d_right) / 2.0

        # Simple differential drive model
        wheel_base = 0.5  # meters, adjust for your robot
        d_theta = (d_right - d_left) / wheel_base

        # Update pose
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Compute velocities
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9  # seconds
        if dt > 0:
            vx = d_center / dt
            vy = 0.0
            vtheta = d_theta / dt
        else:
            vx = 0.0
            vy = 0.0
            vtheta = 0.0
        self.prev_time = now

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Simple orientation quaternion around Z axis
        odom_msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.theta / 2.0),
            w=math.cos(self.theta / 2.0)
        )

        # Twist
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vtheta

        # Publish
        self.odom_pub.publish(odom_msg)
        self.get_logger().info("Published odometry message")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import math
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from gogo_interfaces.msg import EncoderFeedback
from gogo_description.frames import ODOM_FRAME, BASE_LINK_FRAME
from gogo_description.robot_constants import WHEEL_RADIUS, WHEEL_SEPARATION, ENCODER_CPR
from gogo_description.qos_profiles import SENSOR_QOS

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.get_logger().info("OdometryNode initialized")

        # Publisher for nav_msgs/Odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', SENSOR_QOS)

        # Subscriber to encoder feedback
        self.encoder_sub = self.create_subscription(EncoderFeedback, 'encoders', self.encoder_callback, SENSOR_QOS)

        self.declare_parameter("wheel_radius", WHEEL_RADIUS)
        self.declare_parameter("wheel_separation", WHEEL_SEPARATION)
        self.declare_parameter("encoder_cpr", ENCODER_CPR)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.encoder_cpr = self.get_parameter("encoder_cpr").get_parameter_value().double_value

        self.ticks_per_meter = self.encoder_cpr / (2 * math.pi * self.wheel_radius)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None

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
        d_theta = (d_right - d_left) / self.wheel_separation

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
        odom_msg.header.frame_id = ODOM_FRAME
        odom_msg.child_frame_id = BASE_LINK_FRAME

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
        odom_msg.twist.twist.angular.z = vtheta

        # Publish
        self.odom_pub.publish(odom_msg)
        self.publish_tf(msg)
    
    def publish_tf(self, msg: EncoderFeedback):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_LINK_FRAME
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

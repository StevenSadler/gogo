#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from gogo_description.qos_profiles import (
    CMD_VEL_QOS,
    SENSOR_QOS,
    GOAL_QOS
)


class GoalFollow(Node):
    def __init__(self):
        super().__init__("goal_follow")

        self.get_logger().info("GoalFollow initialized")

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", CMD_VEL_QOS)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, SENSOR_QOS)
        self.goal_sub = self.create_subscription(PoseStamped, "goal_pose", self.goal_callback, GOAL_QOS)

        # Parameters
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.50)
        self.declare_parameter("position_tolerance", 0.05)
        self.declare_parameter("angle_tolerance", 0.10)

        self.max_linear_speed = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self.position_tolerance = self.get_parameter("position_tolerance").get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter("angle_tolerance").get_parameter_value().double_value
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Goal state
        self.goal_x = None
        self.goal_y = None

        self.get_logger().info(
            f"Params: "
            f"linear={self.max_linear_speed}, "
            f"angular={self.max_angular_speed}"
        )

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.get_logger().info(
            f"New goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}"
        )

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Convert quaternion to yaw
        self.theta = math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )

        # Motion control will be added next

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)

    node = GoalFollow()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
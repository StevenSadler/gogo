#!/usr/bin/env python3

import math
import rclpy
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
from gogo_description.frames import BASE_LINK_FRAME, ODOM_FRAME
from gogo_description.qos_profiles import (
    CMD_VEL_QOS,
    GOAL_QOS
)


class GoalFollow(Node):
    def __init__(self):
        super().__init__("goal_follow")

        self.get_logger().info("GoalFollow initialized")

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_nav", CMD_VEL_QOS)

        # Subscribers
        self.goal_sub = self.create_subscription(PoseStamped, "goal_pose", self.goal_callback, GOAL_QOS)

        # Parameters
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.50)
        self.declare_parameter("position_tolerance", 0.05)
        self.declare_parameter("angle_tolerance", 0.10)

        self.max_linear_speed = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self.position_tolerance = self.get_parameter("position_tolerance").get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter("angle_tolerance").get_parameter_value().double_value

        self.control_rate = self.get_parameter("control_rate").get_parameter_value().double_value
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Goal state
        self.goal_pose = None
        self.goal_active = False

        self.get_logger().info(
            f"Params: "
            f"control_rate={self.control_rate} Hz, "
            f"linear={self.max_linear_speed}, "
            f"angular={self.max_angular_speed}"
        )

        self.log_counter = 0
    
    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg.pose
        self.goal_active = True

        self.get_logger().info(
            f"New goal received: "
            f"x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}, "
            f"frame={msg.header.frame_id}"
        )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_goal_pose_stamped(self) -> PoseStamped:
        # Goal is a fixed target in odom.
        # Use latest TF state because controller needs current goal-relative position
        # not the transform from the instant RViz created the goal.        
        goal = PoseStamped()
        goal.header.frame_id = ODOM_FRAME
        goal.header.stamp = rclpy.time.Time().to_msg()
        goal.pose = self.goal_pose

        return goal
    
    def control_loop(self):
        if not self.goal_active:
            self.stop_robot()
            return
        
        # if we are here, goal_active is True and goal_pose is not None
        goal = self.get_goal_pose_stamped()
        
        try:
            goal_base = self.tf_buffer.transform(
                goal,
                BASE_LINK_FRAME,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

        except Exception as e:
            self.get_logger().warn(f"Goal transform failed: {e}")
            return

        goal_x = goal_base.pose.position.x
        goal_y = goal_base.pose.position.y

        distance = math.hypot(goal_x, goal_y)
        heading_error = math.atan2(goal_y, goal_x)

        cmd = Twist()

        if distance <= self.position_tolerance:
            self.get_logger().info("Goal reached")
            self.stop_robot()
            self.goal_active = False
            self.goal_pose = None
            return
        
        elif abs(heading_error) > self.angle_tolerance:
            cmd.linear.x = 0.0
            if heading_error > 0:
                cmd.angular.z = self.max_angular_speed
            else:
                cmd.angular.z = -self.max_angular_speed
        
        else:
            cmd.linear.x = self.max_linear_speed
            cmd.angular.z = 0.0

        self.log_counter += 1
        if self.log_counter >= self.control_rate:
            self.log_counter = 0
            self.get_logger().info(
                f"Goal({BASE_LINK_FRAME}): x={goal_x:.2f}, "
                f"y={goal_y:.2f}, "
                f"Distance={distance:.2f} m, "
                f"Heading error={math.degrees(heading_error):.1f} degrees, "
                f"Command: linear={cmd.linear.x:.2f}, "
                f"angular={cmd.angular.z:.2f}"
            )
        
        self.cmd_pub.publish(cmd)


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
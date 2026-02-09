#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from gogo_interfaces.msg import ModeSelect
import time

# -------------------------
# Named test scenarios
# -------------------------
FULL_FORWARD            = (0.1, 0.0)
FULL_REVERSE            = (-0.1, 0.0)
FULL_LEFT               = (0.0, 0.72)
FULL_RIGHT              = (0.0, -0.72)
FULL_FORWARD_HALF_LEFT  = (0.1, 0.36)
FULL_REVERSE_HALF_RIGHT = (-0.1, -0.36)
FULL_FORWARD_HALF_RIGHT = (0.1, -0.36)
FULL_REVERSE_HALF_LEFT  = (-0.1, 0.36)
STOP                    = (0.0, 0.0)

# -------------------------
# Test sequence
# -------------------------
TEST_SEQUENCE = [
    FULL_FORWARD,
    FULL_REVERSE,
    FULL_RIGHT,
    FULL_LEFT,
    FULL_FORWARD_HALF_RIGHT,
    FULL_REVERSE_HALF_LEFT,
]

class CmdVelTestNode(Node):
    def __init__(self):
        super().__init__('cmdvel_test_node')

        # Publisher QoS
        cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        mode_select_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # ROS subscriptions and publishers
        self.pub = self.create_publisher(Twist, 'cmd_vel', cmd_vel_qos)
        self.mode_pub = self.create_publisher(ModeSelect, 'mode_select', mode_select_qos)

        # Parameters
        self.declare_parameter('move_duration', 2.0)
        self.declare_parameter('stop_duration', 1.0)

        self.move_duration = self.get_parameter('move_duration').get_parameter_value().double_value
        self.stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value

        # Internal state for timer-based sequence
        self.sequence = []
        for movement in TEST_SEQUENCE:
            # Add movement segment and corresponding stop segment
            self.sequence.append((movement, self.move_duration))
            self.sequence.append((STOP, self.stop_duration))
        self.current_index = 0
        self.start_time = None

        # Timer tick interval
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(f"CmdVelTestNode initialized with {len(TEST_SEQUENCE)} segments")

    def timer_callback(self):
        # First tick: record start time
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        current_time = self.get_clock().now().nanoseconds / 1e9
        segment, duration = self.sequence[self.current_index]

        # Publish current segment
        twist = Twist()
        twist.linear.x = segment[0]
        twist.angular.z = segment[1]
        self.pub.publish(twist)

        # Check if we should advance to the next segment
        if self.start_time + duration <= current_time:
            self.start_time = current_time
            self.current_index += 1
            if self.current_index >= len(self.sequence):
                self.get_logger().info("Test sequence complete. Motors stopped.")
                # Stop publishing and shutdown timer
                twist = Twist()
                self.pub.publish(twist)  # ensure motors stop
                self.timer.cancel()
                return
            next_seg = self.sequence[self.current_index][0]
            self.get_logger().info(f"Next segment: linear={next_seg[0]}, angular={next_seg[1]}")

def main():
    rclpy.init()
    node = CmdVelTestNode()
    try:
        rclpy.spin(node)
    finally:
        # Ensure motors stop on exit
        twist = Twist()
        node.pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

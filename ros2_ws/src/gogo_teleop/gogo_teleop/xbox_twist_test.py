#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from gogo_interfaces.msg import ModeSelect

class XboxTwistTest(Node):
    """
    Test harness for XboxTwist + mode selection.
    Prints cmd_vel and mode_select messages to console.
    """

    def __init__(self):
        super().__init__("xbox_twist_test")

        # Subscribe to cmd_vel and mode_select
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(ModeSelect, "mode_select", self.mode_cb, 10)

        self.get_logger().info("XboxTwistTest node started. Move joystick or press buttons...")

    def cmd_vel_cb(self, msg: Twist):
        self.get_logger().info(f"[CMD_VEL] linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}")

    def mode_cb(self, msg: ModeSelect):
        self.get_logger().info(f"[MODE_SELECT] mode={msg.mode}")


def main(args=None):
    rclpy.init(args=args)
    node = XboxTwistTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class XboxTwist(Node):
    def __init__(self):
        super().__init__('xbox_twist')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Parameters for mapping
        self.declare_parameter('enable_button', 4)  # LB
        self.declare_parameter('axis_linear_x', 1)  # left stick vertical
        self.declare_parameter('axis_angular_z', 3) # right stick horizontal
        self.declare_parameter('scale_linear', 0.05)
        self.declare_parameter('scale_angular', 0.36)

        self.enable_button = self.get_parameter('enable_button').value
        self.axis_linear = self.get_parameter('axis_linear_x').value
        self.axis_angular = self.get_parameter('axis_angular_z').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value

        self.get_logger().warn(f"PARAMS: scale_linear={self.scale_linear}, scale_angular={self.scale_angular}")

    def joy_callback(self, msg: Joy):
        twist = Twist()
        if msg.buttons[self.enable_button]:
            twist.linear.x = msg.axes[self.axis_linear] * self.scale_linear
            twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = XboxTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

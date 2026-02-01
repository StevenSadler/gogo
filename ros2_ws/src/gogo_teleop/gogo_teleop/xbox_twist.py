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
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)

    def joy_callback(self, msg: Joy):
        twist = Twist()
        enable_button = self.get_parameter('enable_button').value
        axis_linear = self.get_parameter('axis_linear_x').value
        axis_angular = self.get_parameter('axis_angular_z').value
        scale_linear = self.get_parameter('scale_linear').value
        scale_angular = self.get_parameter('scale_angular').value

        if msg.buttons[enable_button]:
            twist.linear.x = msg.axes[axis_linear] * scale_linear
            twist.angular.z = msg.axes[axis_angular] * scale_angular
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

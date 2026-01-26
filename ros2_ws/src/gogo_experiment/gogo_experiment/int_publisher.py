#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gogo_interfaces.msg import IntStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class IntPublisher(Node):
    def __init__(self):
        super().__init__("int_publisher")

        qos = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1,
            reliability = QoSReliabilityPolicy.BEST_EFFORT
        )

        self.pub = self.create_publisher(IntStamped, "int_topic", qos)

        self.targets = [100, 400, 700, 400]
        self.target_index = 0
        self.count = 0
        self.timer_period = 0.02   # seconds


        self.get_logger().info(f"Publishing at {self.timer_period * 1000} ms")

        self.timer = self.create_timer(self.timer_period, self.timerCallback)
    
    def timerCallback(self):
        self.count += 1
        msg = IntStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.int_value = self.targets[self.target_index]
        msg.counter = self.count

        self.pub.publish(msg)

        self.get_logger().info(f"Publishing count={self.count} at time={msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

        self.target_index = (self.target_index + 1) % len(self.targets)

        

def main():
    rclpy.init()
    node = IntPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
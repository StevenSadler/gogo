#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gogo_interfaces.msg import ModeSelect
from gogo_description.qos_profiles import (
    CMD_VEL_QOS, 
    MODE_SELECT_QOS
)

class CmdVelMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")

        self.current_mode = "MODE_IDLE"
        self.operator_enabled = False

        self.joy_sub = self.create_subscription(Twist, "cmd_vel_joy", self.joy_callback, CMD_VEL_QOS)
        self.nav_sub = self.create_subscription(Twist, "cmd_vel_nav", self.nav_callback, CMD_VEL_QOS)
        self.mode_sub = self.create_subscription(ModeSelect, "mode_select", self.mode_callback, MODE_SELECT_QOS)
        self.enable_sub = self.create_subscription(Bool, "operator_enable", self.enable_callback, MODE_SELECT_QOS)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', CMD_VEL_QOS)

        self.get_logger().info("cmd_vel_mux initialized")
    
    def mode_callback(self, msg):
        self.current_mode = msg.mode
        self.get_logger().info(f"Mode changed: {self.current_mode}")

    def enable_callback(self, msg):
        self.operator_enabled = msg.data

        self.get_logger().info(f"Operator enable: {self.operator_enabled}")

        if not self.operator_enabled:
            self.cmd_pub.publish(Twist())
    
    def joy_callback(self, msg):
        if not self.operator_enabled:
            return
        
        if self.current_mode == "MODE_DRIVE":
            self.cmd_pub.publish(msg)
    
    def nav_callback(self, msg):
        if not self.operator_enabled:
            return
        
        if self.current_mode == "MODE_AUTO":
            self.cmd_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
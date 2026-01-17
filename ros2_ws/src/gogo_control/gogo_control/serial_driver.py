#!usr/bin/dev python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import serial
from math import pi


class SerialDriver(Node):
    def __init__(self):
        super().__init__("serial_driver")

        self.command_label = 'C'
        self.state_label = 'S'

        self.declare_parameter("wheel_radius", 0.038)     # 1.5 inches
        self.declare_parameter("wheel_separation", 0.278) # 0.228 beam + 2 * 0.13 hub width + 0.024 wheel length = 0.278
        self.declare_parameter("encoder_cpr", 4741.34)    # Pololu 4847 has 48 CPR * 98.7779 gear ratio = 4741.34

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.encoder_cpr_ = self.get_parameter("encoder_cpr").get_parameter_value().double_value

        self.ticks_per_meter_ = self.encoder_cpr_ / (2 * pi * self.wheel_radius_)

        # this is the port when arduino is connected to PC VM
        # i need to check the port when it is connected to raspberry pi and parameterize this
        self.serial_ = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)
        

        self.get_logger().info("Using wheel_radius: %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel_separation: %f" % self.wheel_separation_)

        self.vel_sub_ = self.create_subscription(TwistStamped, "gogo_control/cmd_vel", self.velCallback, 10)        
    
    def velCallback(self, msg):
        v = msg.twist.linear.x
        w = msg.twist.angular.z

        half_wL = w * self.wheel_separation_ / 2
        
        # wheel linear velocities in mps
        v_left = v - half_wL
        v_right = v + half_wL

        # convert left and right vels in mps to ticks per second as integer because Roboclaw needs that
        left_tps = int(v_left * self.ticks_per_meter_)
        right_tps = int(v_right * self.ticks_per_meter_)

        # create a string message containing left and right wheel vel to send to arduino through serial
        serial_cmd = f"{self.command_label}: {left_tps} {right_tps}\n"
        # self.serial_.write(serial_cmd.encode())

        self.get_logger().info(serial_cmd)


def main():
    rclpy.init()
    node = SerialDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
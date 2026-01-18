#!usr/bin/dev python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import serial
from serial.serialutil import SerialException
from math import pi


class SerialDriver(Node):
    def __init__(self):
        super().__init__("serial_driver")

        self.COMMAND_LABEL = 'C'
        self.STATE_LABEL = 'S'
        self.MAX_TPS = 5000

        self.declare_parameter("wheel_radius", 0.038)     # 1.5 inches
        self.declare_parameter("wheel_separation", 0.278) # 0.228 beam + 2 * 0.013 hub width + 0.024 wheel length = 0.278
        self.declare_parameter("encoder_cpr", 4741.34)    # Pololu 4847 has 48 CPR * 98.7779 gear ratio = 4741.34
        self.declare_parameter("arduino_port", "/dev/ttyACM0") # port when running ros on PC VM

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.encoder_cpr_ = self.get_parameter("encoder_cpr").get_parameter_value().double_value
        self.arduino_port_ = self.get_parameter("arduino_port").get_parameter_value().string_value

        self.ticks_per_meter_ = self.encoder_cpr_ / (2 * pi * self.wheel_radius_)        

        self.get_logger().info(f"Using wheel_radius: {self.wheel_radius_}")
        self.get_logger().info(f"Using wheel_separation: {self.wheel_separation_}")
        self.get_logger().info(f"Using encoder_cpr: {self.encoder_cpr_}")
        self.get_logger().info(f"Using arduino_port: {self.arduino_port_}")

        self.vel_sub_ = self.create_subscription(TwistStamped, "gogo_control/cmd_vel", self.velCallback, 10)

        self.arduino_ = None
        self.timer_ = self.create_timer(1.0, self.connectArduino)
    
    def connectArduino(self):
        if self.arduino_ is not None:
            return
        
        try:
            self.get_logger().info(f"Trying to open {self.arduino_port_}")
            self.arduino_ = serial.Serial(self.arduino_port_, 115200, timeout=0.1)
            self.get_logger().info("Arduino connected")
        except SerialException as e:
            self.get_logger().warn(f"Arduino not available: {e}")

    
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

        left_tps = max(min(left_tps, self.MAX_TPS), -self.MAX_TPS)
        right_tps = max(min(right_tps, self.MAX_TPS), -self.MAX_TPS)

        # create a string message containing left and right wheel vel to send to arduino through serial
        serial_cmd = f"{self.COMMAND_LABEL}: {left_tps} {right_tps}\n"
        if self.arduino_ is not None:
            try:
                self.arduino_.write(serial_cmd.encode())
            except:
                self.get_logger().error("Lost connection to Arduino")
                self.arduino_.close()
                self.arduino_ = None

        # while testing only
        self.get_logger().info(serial_cmd)
    
    def destroy_node(self):
        if self.arduino_:
            stop_cmd = f"{self.COMMAND_LABEL}: 0 0\n"
            self.arduino_.write(stop_cmd.encode())
            self.arduino_.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
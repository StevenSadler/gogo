#!usr/bin/dev python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import serial
from serial.serialutil import SerialException
from math import pi
import time


class SerialDriver(Node):
    def __init__(self):
        super().__init__("serial_driver")

        self.COMMAND_LABEL = 'C'
        self.MAX_TPS = 5000

        self.SEND_RATE_HZ = 20.0
        self.CMD_TIMEOUT_SEC = 1.0

        self.declare_parameter("wheel_radius", 0.038)     # 1.5 inches
        self.declare_parameter("wheel_separation", 0.278) # 0.228 beam + 2 * 0.013 hub width + 0.024 wheel length = 0.278
        self.declare_parameter("encoder_cpr", 4741.34)    # Pololu 4847 has 48 CPR * 98.7779 gear ratio = 4741.34
        self.declare_parameter("arduino_port", "/dev/ttyACM0") # port when running ros on PC VM
        self.declare_parameter("baudrate", 115200)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.encoder_cpr_ = self.get_parameter("encoder_cpr").get_parameter_value().double_value
        self.arduino_port_ = self.get_parameter("arduino_port").get_parameter_value().string_value
        self.baudrate_ = self.get_parameter("baudrate").get_parameter_value().integer_value

        self.ticks_per_meter_ = self.encoder_cpr_ / (2 * pi * self.wheel_radius_)

        # save previous calculated state
        self.last_left_tps_ = 0
        self.last_right_tps_ = 0
        self.last_cmd_time_ = self.get_clock().now()

        self.arduino_ = None

        self.vel_sub_ = self.create_subscription(TwistStamped, "gogo_control/cmd_vel", self.vel_callback, 10)
        self.connect_timer_ = self.create_timer(1.0, self.connect_arduino)
        self.send_timer_ = self.create_timer(1.0 / self.SEND_RATE_HZ, self.send_command)
    
    def connect_arduino(self):
        if self.arduino_ and self.arduino_.is_open:
            return
        
        try:
            self.get_logger().info(f"Trying to open Arduino on {self.arduino_port_}")
            self.arduino_ = serial.Serial(self.arduino_port_, self.baudrate_, timeout=0.1)
            time.sleep(2.0)
            self.get_logger().info("Arduino connected")
        except SerialException as e:
            self.get_logger().warn(f"Arduino not available: {e}")
            self.arduino_ = None

    
    def vel_callback(self, msg):
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

        self.last_left_tps_ = left_tps
        self.last_right_tps_ = right_tps
        self.last_cmd_time_ = self.get_clock().now()

        self.get_logger().debug(f"Received cmd: {left_tps} {right_tps}")
    
    def send_command(self):
        if not self.arduino_ or not self.arduino_.is_open:
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time_).nanoseconds * 1e-9

        if elapsed > self.CMD_TIMEOUT_SEC:
            left = 0
            right = 0
            self.get_logger().warn("watchdog expired - stopping motors")
        else:
            left = self.last_left_tps_
            right = self.last_right_tps_
        
        serial_cmd = f"{self.COMMAND_LABEL}: {left} {right}\n"

        try:
            self.arduino_.write(serial_cmd.encode())
        except:
            self.get_logger().error("Lost connection to Arduino")
            self.arduino_.close()
            self.arduino_ = None
    
    def destroy_node(self):
        if self.arduino_ and self.arduino_.is_open:
            try:
                stop_cmd = f"{self.COMMAND_LABEL}: 0 0\n"
                self.arduino_.write(stop_cmd.encode())
                self.arduino_.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
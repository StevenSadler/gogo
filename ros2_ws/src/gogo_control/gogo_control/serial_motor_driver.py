#!usr/bin/dev python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from math import pi
from gogo_control.serial_connection_handler import SerialConnectionHandler
from gogo_control.command_watchdog import CommandWatchdog


class SerialMotorDriver(Node):
    def __init__(self):
        super().__init__("serial_motor_driver")

        self.COMMAND_LABEL = 'C'
        self.MAX_TPS = 5000

        self.CMD_TIMEOUT_SEC = 0.5
        self.RECONNECT_PERIOD_SEC = 2.0
        self.WATCHDOG_POLL_SEC = 0.1

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


        self.watchdog_ = CommandWatchdog(self.CMD_TIMEOUT_SEC)
        self.serial_connection_ = SerialConnectionHandler(
            port = self.arduino_port_,
            baudrate = self.baudrate_,
            reconnect_period_sec = self.RECONNECT_PERIOD_SEC,
            logger = self.get_logger(),
        )

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_timer(self.WATCHDOG_POLL_SEC, self.watchdog_poll_callback)
        self.create_timer(self.RECONNECT_PERIOD_SEC, self.serial_connection_.periodic_reconnect)
    

    def cmd_vel_callback(self, msg: Twist):
        self.watchdog_.kick()
        self.left_tps_, self.right_tps_ = self.generate_tps(msg)
        cmd = self.generate_move_cmd(self.left_tps_, self.right_tps_)
        self.serial_connection_.write(cmd)

    def watchdog_poll_callback(self):
        if self.watchdog_.is_timed_out():
            # Send stop command if cmd_vel stopped
            cmd = self.generate_stop_cmd()
            self.serial_connection_.write(cmd)

    def destroy_node(self):
        self.serial_connection_.close()
        super().destroy_node()


    def generate_tps(self, msg: Twist):
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

        return (left_tps, right_tps)
    
    def generate_move_cmd(self, left, right):
        return f"{self.COMMAND_LABEL}: {left} {right}\n"
    
    def generate_stop_cmd(self):
        return self.generate_move_cmd(0, 0)
    
    


def main():
    rclpy.init()
    node = SerialMotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

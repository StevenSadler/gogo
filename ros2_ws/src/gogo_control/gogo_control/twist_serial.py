#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from gogo_interfaces.msg import MotorCommand, ModeSelect, EncoderFeedback
from gogo_control.hardware.watchdog import Watchdog
from gogo_control.hardware.serial_manager import SerialManager
from gogo_control.kinematics.diff_drive import DiffDriveKinematics
from math import pi

class TwistSerial(Node):
    def __init__(self):
        super().__init__("twist_serial")

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
        # use this for encoders, imu, lidar
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.current_mode = "MODE_IDLE"

        # ROS subscriptions and publishers
        self.sub = self.create_subscription(Twist, "cmd_vel", self.twist_callback, cmd_vel_qos)
        self.mode_sub = self.create_subscription(ModeSelect, "mode_select", self.mode_callback, mode_select_qos)
        self.pub = self.create_publisher(MotorCommand, "motor_command", cmd_vel_qos)
        self.enc_pub = self.create_publisher(EncoderFeedback, "encoders", sensor_qos)

        # Parameters
        self.declare_parameter("min_tps", 0)
        self.declare_parameter("max_tps", 2000)
        self.declare_parameter("wheel_radius", 0.038)
        self.declare_parameter("wheel_separation", 0.278)
        self.declare_parameter("encoder_cpr", 4741.34)
        self.declare_parameter("arduino_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 38400)
        self.declare_parameter("enable_serial", True)

        self.min_tps = self.get_parameter("min_tps").get_parameter_value().integer_value
        self.max_tps = self.get_parameter("max_tps").get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.encoder_cpr = self.get_parameter("encoder_cpr").get_parameter_value().double_value
        self.arduino_port = self.get_parameter("arduino_port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.enable_serial = self.get_parameter("enable_serial").get_parameter_value().bool_value

        self.kinematics = DiffDriveKinematics(
            wheel_separation=self.wheel_separation,
            wheel_radius=self.wheel_radius,
            encoder_cpr=self.encoder_cpr,
            max_tps=self.max_tps,
        )

        self.ticks_per_meter = self.encoder_cpr / (2 * pi * self.wheel_radius)

        self.get_logger().warn(
            f"PARAMS: min_tps={self.min_tps}, max_tps={self.max_tps}, ticks_per_meter={self.ticks_per_meter:.1f}"
        )

        # Expected upstream publish period
        self.publish_period_sec = 0.08  # ~12.5 Hz
        self.cmd_timeout_sec = 0.3

        # Watchdog
        self.watchdog = Watchdog(self.cmd_timeout_sec, self.watchdog_expired_callback)
        self.watchdog_expired = False
        self.create_timer(0.05, self.watchdog_action)

        # Serial manager
        self.serial_manager = SerialManager(
            port=self.arduino_port,
            baudrate=self.baudrate,
            logger=self.get_logger(),
            encoder_callback=self.encoder_callback,
        )
        # Timers for reading and reconnecting
        self.create_timer(0.02, self.serial_manager.read_available_bytes)
        self.create_timer(0.5, self.serial_manager.periodic_reconnect)

        # Logging throttling
        self.last_tx_log_time = 0.0
        self.tx_log_period_sec = 0.1

        self.get_logger().info(
            f"TwistSerial node initialized: watchdog {self.cmd_timeout_sec*1000:.1f} ms")

    # --------------------------
    # ROS CALLBACKS
    # --------------------------
    def mode_callback(self, msg: ModeSelect):
        mode = msg.mode
        self.current_mode = msg.mode
        self.get_logger().info(f"Received mode: {mode}")

        self.send_to_serial(mode)

    def twist_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Reset watchdog timer
        self.watchdog.kick()

        # Convert Twist to wheel TPS
        left_tps, right_tps = self.kinematics.twist_to_tps(v, w)

        # Publish for ROS visibility/debug
        cmd = MotorCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.left_tps = left_tps
        cmd.right_tps = right_tps
        self.pub.publish(cmd)

        # TODO: look into if this is necessary or just to force safety in IDLE and TEST
        if self.current_mode != "MODE_DRIVE":
            self.get_logger().warn(f"Skipping send: current_mode='{self.current_mode}'")
            return

        self.send_motor_cmd_to_serial(left_tps, right_tps)

    # --------------------------
    # HELPER FUNCTIONS
    # --------------------------
    def send_to_serial(self, cmd: str):
        self.serial_manager.send_command(cmd)
    
    def send_motor_cmd_to_serial(self, left: int, right: int):
        self.serial_manager.send_motor_command(left, right)

    def encoder_callback(self, left, right, timestamp):
        msg = EncoderFeedback()
        msg.left_ticks = left
        msg.right_ticks = right
        msg.timestamp_ms = timestamp

        self.enc_pub.publish(msg)

    # --------------------------
    # WATCHDOG & CLEANUP
    # --------------------------
    def watchdog_expired_callback(self):
        # now = self.get_clock().now().to_msg()
        # self.get_logger().warn(f"[WATCHDOG] EXPIRED at {now.sec}.{now.nanosec}")
        # self.send_motor_cmd_to_serial(0, 0)

        # Timer thread - do NOT touch serial here
        self.watchdog_expired = True
    
    def watchdog_action(self):
        if not self.watchdog_expired:
            return

        self.watchdog_expired = False

        now = self.get_clock().now().to_msg()
        self.get_logger().warn(
            f"[WATCHDOG] EXPIRED at {now.sec}.{now.nanosec} — stopping motors"
        )

        self.send_motor_cmd_to_serial(0, 0)

    def destroy_node(self):
        self.watchdog.cancel()
        if self.serialConn:
            self.serialConn.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = TwistSerial()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from gogo_interfaces.msg import MotorCommand, ModeSelect, EncoderFeedback
from gogo_control.twist_serial_connection_handler import TwistSerialConnectionHandler
from gogo_control.serial_message_parser import SerialMessageParser
from threading import Thread, Lock, Timer
from queue import Queue
from math import pi
import time


class Watchdog:
    """Threaded watchdog that calls a callback if not kicked in time."""

    def __init__(self, timeout_sec: float, expired_callback):
        self.timeout_sec = timeout_sec
        self.expired_callback = expired_callback
        self._lock = Lock()
        self._timer = None
        self._active = True
        self.kick()  # start immediately

    def _timer_callback(self):
        with self._lock:
            if self._active:
                self.expired_callback()

    def kick(self):
        """Reset the watchdog timer."""
        with self._lock:
            if self._timer:
                self._timer.cancel()
            if self._active:
                self._timer = Timer(self.timeout_sec, self._timer_callback)
                self._timer.start()

    def cancel(self):
        """Stop the watchdog completely."""
        with self._lock:
            self._active = False
            if self._timer:
                self._timer.cancel()
                self._timer = None


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

        # Serial connection
        if self.enable_serial:
            self.serialConn = TwistSerialConnectionHandler(
                port=self.arduino_port,
                baudrate=self.baudrate,
                reconnect_period_sec=1.0,
                logger=self.get_logger(),
                frame_callback=self.handle_serial_frame,
            )
            self.create_timer(0.02, self.serial_read_timer_callback)  # 50Hz to match
            self.create_timer(0.5, self.serial_reconnect_timer_callback)
        else:
            self.serialConn = None

        # Serial message parser
        self.serial_parser = SerialMessageParser(self.get_logger(), self.encoder_callback)
        

        # Logging throttling
        self.last_tx_log_time = 0.0
        self.tx_log_period_sec = 0.1

        self.get_logger().info(
            f"TwistSerial node initialized: watchdog {self.cmd_timeout_sec*1000:.1f} ms, "
            f"serial={'enabled' if self.enable_serial else 'disabled'}"
        )
    
    def send_to_serial(self, cmd: str):
        # Send command string to Arduino
        if self.enable_serial and self.serialConn and self.serialConn.serial:
            self.serialConn.write_cmd(cmd)
    
    def send_motor_cmd_to_serial(self, left: int, right: int):
        self.send_to_serial(f"{left},{right}")


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
        self.get_logger().debug(f"twist_callback HIT - mode='{self.current_mode}' raw twist: v={v:.3f}, w={w:.3f}")

        # Reset watchdog timer
        self.watchdog.kick()

        # Convert Twist to wheel TPS
        left_tps, right_tps = self.twist_to_tps(msg)

        self.get_logger().debug(
            f"TPS: L={left_tps} R={right_tps} v={v:.3f} m/s, w={w:.3f} rad/s)"
        )

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
        
        self.get_logger().debug(f"Sending to Arduino: L={left_tps}, R={right_tps}")

        self.send_motor_cmd_to_serial(left_tps, right_tps)

    # --------------------------
    # HELPER FUNCTIONS
    # --------------------------
    def twist_to_tps(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics - wheel velocities (m/s)
        half_wL = w * self.wheel_separation / 2.0
        v_left = v - half_wL
        v_right = v + half_wL

        # Convert to raw tps as floats
        left_tps = v_left * self.ticks_per_meter
        right_tps = v_right * self.ticks_per_meter

        # Arc preserving proportional clamp
        max_mag = max(abs(left_tps), abs(right_tps))
        if max_mag > self.max_tps:
            scale = self.max_tps / max_mag
            left_tps *= scale
            right_tps *= scale
            self.get_logger().debug(
                f"TSP scaled: scale={scale:.3f} -> L={left_tps:.1f}, R={right_tps:.1f}"
            )

        self.get_logger().debug(
            f"twist_to_tps: v_left={v_left:.4f} m/s, v_right={v_right:.4f} m/s -> "
            f"L={left_tps:.1f}, R={right_tps:.1f}"
        )

        return int(left_tps), int(right_tps)
    
    def handle_serial_frame(self, payload:bytes):
        self.serial_parser.handle_payload(payload)
    
    def serial_read_timer_callback(self):
        if self.enable_serial and self.serialConn:
            self.serialConn.read_available_bytes()
    
    def serial_reconnect_timer_callback(self):
        if self.enable_serial and self.serialConn:
            self.serialConn.periodic_reconnect()
    
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

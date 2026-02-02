#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from gogo_interfaces.msg import MotorCommand
from gogo_experiment.twist_serial_connection_handler import TwistSerialConnectionHandler
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
                # Only call expired_callback if not kicked since last timer started
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

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Parameters
        self.declare_parameter("min_tps", 400)
        self.declare_parameter("max_tps", 1000)
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

        # ROS subscriptions and publishers
        self.sub = self.create_subscription(Twist, "cmd_vel", self.twist_callback, qos)
        self.pub = self.create_publisher(MotorCommand, "motor_command", qos)

        # Watchdog
        self.watchdog = Watchdog(self.cmd_timeout_sec, self.watchdog_expired_callback)

        # Serial queue and thread
        self.serial_queue = Queue()
        if self.enable_serial:
            self.serial = TwistSerialConnectionHandler(
                port=self.arduino_port,
                baudrate=self.baudrate,
                reconnect_period_sec=1.0,
                logger=self.get_logger()
            )
        else:
            self.serial = None

        self._stop_serial_thread = Thread(target=self.serial_worker, daemon=True)
        self._stop_serial_thread.start()

        # Logging throttling
        self.last_tx_log_time = 0.0
        self.tx_log_period_sec = 0.1

        self.get_logger().info(
            f"TwistSerial node initialized: watchdog {self.cmd_timeout_sec*1000:.1f} ms, "
            f"serial={'enabled' if self.enable_serial else 'disabled'}"
        )

    def twist_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        self.get_logger().debug(f"twist_callback HIT - raw twist: v={v:.3f}, w={w:.3f}")
        
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

        # Queue serial payload
        self.queue_serial(left_tps, right_tps)

    def twist_to_tps(self, msg: Twist):
        # Convert Twist (m/s, rad/s) to left/right motor TPS
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics - wheel velocities (m/s)
        half_wL = w * self.wheel_separation / 2.0
        v_left = v - half_wL
        v_right = v + half_wL

        left_tps = self.velocity_to_tps(v_left)
        right_tps = self.velocity_to_tps(v_right)
        self.get_logger().debug(
            f"twist_to_tps: v_left={v_left:.4f} m/s, v_right={v_right:.4f} m/s -> "
            f"L={left_tps}, R={right_tps}"
        )
        return left_tps, right_tps
    
    def velocity_to_tps(self, v_mps: float) -> int:
        # Map linear velocity to motor ticks per second with min/max TPS
        raw_tps = v_mps * self.ticks_per_meter

        if abs(raw_tps) < self.min_tps:
            tps = 0
            reason = f"abs(raw_tps) < min_tps ({raw_tps:.1f} < {self.min_tps})"
        else:
            tps = max(-self.max_tps, min(raw_tps, self.max_tps))
            reason = f"clamped to max_tps/-max_tps if needed ({tps:.1f})"
        
        self.get_logger().debug(
            f"velocity_to_tps: v_mps={v_mps:.4f} -> raw_tps={raw_tps:.1f} -> TPS={tps} ({reason})"
        )
        return int(tps)

    def queue_serial(self, left: int, right: int):
        if self.enable_serial:
            payload = f"{left},{right}\n"
            self.serial_queue.put(payload)
            now = time.time()
            if now - self.last_tx_log_time >= self.tx_log_period_sec:
                self.get_logger().info(f"TX -> {payload.strip()}")
                self.last_tx_log_time = now

    def serial_worker(self):
        while True:
            try:
                payload = self.serial_queue.get(timeout=0.05)
                if self.serial and self.serial.connected:
                    self.serial.write(payload)
            except Exception:
                continue

    def watchdog_expired_callback(self):
        now = self.get_clock().now().to_msg()
        self.get_logger().warn(f"[WATCHDOG] EXPIRED at {now.sec}.{now.nanosec}")
        self.queue_serial(0, 0)

    def destroy_node(self):
        self.watchdog.cancel()
        if self.serial:
            self.serial.close()
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

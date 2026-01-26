#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gogo_interfaces.msg import IntStamped
from gogo_experiment.int_serial_connection_handler import IntSerialConnectionHandler
from threading import Thread, Event, Lock, Timer
from queue import Queue
import time


class Watchdog:
    """Threaded watchdog that calls a callback if not kicked in time."""
    def __init__(self, timeout_sec: float, callback):
        self.timeout_sec = timeout_sec
        self.callback = callback
        self._timer = None
        self._lock = Lock()
        self._expired_event = Event()
        self.kick()  # start the watchdog

    def _expired(self):
        self._expired_event.set()
        self.callback()

    def kick(self):
        with self._lock:
            if self._timer:
                self._timer.cancel()
            self._timer = Timer(self.timeout_sec, self._expired)
            self._timer.start()
            self._expired_event.clear()

    def cancel(self):
        with self._lock:
            if self._timer:
                self._timer.cancel()
            self._expired_event.clear()


class IntSerial(Node):
    def __init__(self):
        super().__init__("int_serial")

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        )

        # Parameters
        self.declare_parameter("arduino_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 38400)
        self.declare_parameter("enable_serial", True)
        self.arduino_port = self.get_parameter("arduino_port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.enable_serial = self.get_parameter("enable_serial").get_parameter_value().bool_value

        # Expected upstream publish period
        self.publish_period_sec = 0.02  # 50 Hz
        self.cmd_timeout_sec = self.publish_period_sec * 2.5

        # Subscription
        self.sub = self.create_subscription(IntStamped, "int_topic", self.int_stamped_callback, qos)

        # Threaded watchdog
        self.watchdog = Watchdog(self.cmd_timeout_sec, self.watchdog_expired_callback)

        # Serial queue and thread
        self.serial_queue = Queue()
        if self.enable_serial:
            self.serial = IntSerialConnectionHandler(
                port=self.arduino_port,
                baudrate=self.baudrate,
                reconnect_period_sec=1.0,
                logger=self.get_logger()
            )
        else:
            self.serial = None

        self._stop_serial_thread = Event()
        self.serial_thread = Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()

        # Logging throttling
        self.last_tx_log_time = 0.0
        self.tx_log_period_sec = 0.1  # log every 100 ms

        self.get_logger().info(
            f"IntSerial node initialized: watchdog {self.cmd_timeout_sec*1000:.1f} ms, "
            f"serial={'enabled' if self.enable_serial else 'disabled'}"
        )

    def int_stamped_callback(self, msg: IntStamped):
        # Kick watchdog
        self.watchdog.kick()

        # Convert ROS message to Arduino left/right (placeholder)
        left, right = msg.int_value, msg.int_value

        # Queue serial payload
        self.queue_serial(left, right)

        # Log latency
        now = self.get_clock().now().to_msg()
        latency_sec = (now.sec - msg.header.stamp.sec) + (now.nanosec - msg.header.stamp.nanosec)/1e9
        self.get_logger().info(
            f"Received counter={msg.counter}, int_value={msg.int_value}, latency={latency_sec*1000:.3f} ms"
        )

    def watchdog_expired_callback(self):
        self.get_logger().warn("Watchdog expired: missed messages — taking safety action")
        # Send safety command 0,0 immediately
        self.queue_serial(0, 0)

    def queue_serial(self, left: int, right: int):
        """Queue a serial message to be sent by background thread."""
        if self.enable_serial:
            payload = f"{left},{right}\n"
            self.serial_queue.put(payload)

            # Throttle logging
            now = time.time()
            if now - self.last_tx_log_time >= self.tx_log_period_sec:
                self.get_logger().info(f"TX -> {payload.strip()}")
                self.last_tx_log_time = now

    def _serial_worker(self):
        """Background thread that sends queued serial messages."""
        while not self._stop_serial_thread.is_set():
            try:
                payload = self.serial_queue.get(timeout=0.05)
                if self.serial and self.serial.connected:
                    self.serial.write(payload)
            except Exception:
                continue  # timeout, check stop event

    def destroy_node(self):
        # Stop watchdog and serial thread
        self.watchdog.cancel()
        self._stop_serial_thread.set()
        self.serial_thread.join()
        if self.serial:
            self.serial.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = IntSerial()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

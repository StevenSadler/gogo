#!usr/bin/dev python3

import serial
import time


class SerialConnectionHandler:
    def __init__(
        self,
        port: str,
        baudrate: int,
        reconnect_period_sec: float,
        logger=None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.reconnect_period_sec = reconnect_period_sec
        self.logger = logger

        self.serial = None
        self.connected = False
        self.last_reconnect_attempt = 0.0

        self._connect()

    def _log(self, level: str, msg: str):
        if self.logger:
            getattr(self.logger, level)(msg)

    def _connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.connected = True
            self._log("info", f"Connected to serial port {self.port}")
        except serial.SerialException as e:
            self.connected = False
            self._log("warn", f"Serial connection failed: {e}")

    def write(self, data: str):
        if not self.connected:
            return

        try:
            self.serial.write(data.encode("utf-8"))
        except serial.SerialException as e:
            self.connected = False
            self._log("error", f"Serial write failed: {e}")

    def periodic_reconnect(self):
        """Call periodically from a ROS timer."""
        if self.connected:
            return

        now = time.time()
        if now - self.last_reconnect_attempt < self.reconnect_period_sec:
            return

        self.last_reconnect_attempt = now
        self._log("info", "Attempting serial reconnect...")
        self._connect()

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self._log("info", "Serial port closed")
        self.connected = False

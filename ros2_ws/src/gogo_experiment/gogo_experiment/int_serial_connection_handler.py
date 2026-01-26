#!usr/bin/env python3

import serial
import time


class IntSerialConnectionHandler:
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
        
    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
    
    def _log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)
    
    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)

    def _connect(self):
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1,
                write_timeout=0,
            )
            self.connected = True
            self._log_info(f"Connected to serial port {self.port}")
        except serial.SerialException as e:
            self.connected = False
            self.serial = None
            self._log_warn(f"Serial connection failed: {e}")

    def write(self, data: str):
        if not self.connected or not self.serial:
            return

        try:
            self.serial.write(data.encode("utf-8"))
        except (serial.SerialException, serial.SerialTimeoutException) as e:
            self.connected = False
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
            self._log_error(f"Serial write failed: {e}")

    def periodic_reconnect(self):
        """Call periodically from a ROS timer."""
        if self.connected:
            return

        now = time.monotonic()
        if now - self.last_reconnect_attempt < self.reconnect_period_sec:
            return

        self.last_reconnect_attempt = now
        self._log_info("Attempting serial reconnect...")
        self._connect()

    def close(self):
        if self.serial:
            try:
                self.serial.close()
                self._log_info("Serial port closed")
            except Exception:
                pass
        self.serial = None
        self.connected = False

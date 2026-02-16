# gogo_control/hardware/serial_manager.py
from gogo_control.hardware.twist_serial_connection_handler import TwistSerialConnectionHandler
from gogo_control.hardware.serial_message_parser import SerialMessageParser


class SerialManager:
    """
    High-level firmware serial interface.
    Encapsulates connection, parsing, and telemetry.
    """

    def __init__(self, port, baudrate, logger, encoder_callback=None,
                reconnect_period_sec=1.0, enable_serial=True):
        self.enable_serial = enable_serial
        self.encoder_callback = encoder_callback
        self.logger = logger

        if not self.enable_serial:
            self.logger.info("SerialManager: serial disabled, running in dry mode")
            self.conn = None
            self.parser = None
            self.telemetry = None
            return

        # Normal setup (connection, parser, telemetry)
        self.parser = SerialMessageParser(logger, self._internal_encoder_callback)
        self.conn = TwistSerialConnectionHandler(
            port=port,
            baudrate=baudrate,
            reconnect_period_sec=reconnect_period_sec,
            logger=logger,
            frame_callback=self._handle_serial_frame,
        )


    # --------------------------
    # Public API
    # --------------------------
    def send_command(self, cmd: str):
        """
        Send arbitrary string command to firmware (e.g., motor commands or mode changes).
        """
        if self.conn and self.conn.serial:
            self.conn.write_cmd(cmd)

    def send_motor_command(self, left: int, right: int):
        """
        Convenience wrapper for motor TPS commands.
        """
        self.send_command(f"{left},{right}")

    def register_encoder_callback(self, callback):
        """
        Allow external code (e.g., TwistSerial node) to receive encoder updates.
        """
        self.encoder_callback = callback

    def read_available_bytes(self):
        """
        Trigger reading from the serial port.
        """
        if self.conn:
            self.conn.read_available_bytes()

    def periodic_reconnect(self):
        """
        Call periodically to attempt reconnect if disconnected.
        """
        if self.conn:
            self.conn.periodic_reconnect()

    # --------------------------
    # Internal helpers
    # --------------------------
    def _handle_serial_frame(self, payload: bytes):
        """
        Called by connection handler on each complete frame.
        Pass to parser.
        """
        self.parser.handle_payload(payload)

    def _internal_encoder_callback(self, left, right, timestamp):
        """
        Called by SerialMessageParser when encoder frame arrives.
        Passes to external callback if registered.
        """
        if self.encoder_callback:
            self.encoder_callback(left, right, timestamp)

    def close(self):
        if self.conn:
            self.conn.close()

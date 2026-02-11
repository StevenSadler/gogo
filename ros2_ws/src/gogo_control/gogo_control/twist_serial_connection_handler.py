#!usr/bin/env python3

import serial
import time

STX = 0xAA
ETX = 0x55

class TwistSerialConnectionHandler:
    

    def __init__(
        self,
        port: str,
        baudrate: int,
        reconnect_period_sec: float,
        logger=None,
        frame_callback=None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.reconnect_period_sec = reconnect_period_sec
        self.logger = logger
        self.frame_callback = frame_callback

        self.serial = None
        self.connected = False
        self.last_reconnect_attempt = 0.0
        self._rx_buffer = bytearray()

        self._connect()

    # --------------------------
    # LOGGING HELPERS
    # --------------------------
    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
    
    def _log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)
    
    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
    
    # --------------------------
    # CONNECTION
    # --------------------------
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
    
    # --------------------------
    # WRITING
    # --------------------------
    def _write(self, data: bytes):
        if not self.connected or not self.serial:
            return
        
        if not isinstance(data, bytes):
            raise TypeError(f"Expected bytes, got {type(data).__name__}")

        try:
            # Use write timeout to prevent blocking forever
            self.serial.write_timeout = 0  # non-blocking
            self.serial.write(data)
        except (serial.SerialException, serial.SerialTimeoutException) as e:
            self.connected = False
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
            self._log_error(f"Serial write failed: {e}")
    
    def encode_frame(self, payload: str) -> bytes:
        """
        Convert a payload string into a serial frame with checksum.
        Frame format:
            [START_BYTE][LENGTH][PAYLOAD_BYTES][CHECKSUM][END_BYTE]
        CHECKSUM: XOR of all payload bytes
        """

        # Convert string payload to bytes
        payload_bytes = payload.encode("ascii")

        length = len(payload_bytes)
        if length > 31:
            raise ValueError("Payload too long for Arduino")
        checksum = self._calculate_xor_checksum(payload_bytes)

        # Build frame: start | length | payload | checksum | end
        frame = bytes([STX, length, *payload_bytes, checksum, ETX])
        return frame
    
    def write_cmd(self, cmd: str):
        """Send a string command automatically framed"""
        frame = self.encode_frame(cmd)
        self._write(frame)

    # --------------------------
    # READING / BUFFERING
    # --------------------------
    def read_available_bytes(self):
        """
        Call frequently (e.g., in a ROS timer or loop).
        Reads all available bytes from serial and feeds them to the buffer.
        Calls frame_callback for any valid frames.
        """
        if not self.serial or not self.connected:
            return

        while self.serial.in_waiting:
            b = self.serial.read(1)
            if b:
                self._feed_byte(b)

    def _feed_byte(self, b: bytes):
        """Append a single byte to the rx buffer and attempt to extract frame if ETX."""
        self._rx_buffer += b
        if b == bytes([ETX]):
            # self._try_extract_frame()
            self._process_buffer()

    # --------------------------
    # Frame processing helpers
    # --------------------------
    def _calculate_xor_checksum(self, payload: bytes):
        checksum = 0
        for b in payload:
            checksum ^= b
        return checksum
    
    def _process_buffer(self):
        """
        Process rx_buffer, extracting and handling complete frames.
        """
        while self._rx_buffer:
            start_idx = self._find_frame_start()
            if start_idx is None:
                # No start-of-frame found; stop processing
                break

            if not self._has_complete_frame(start_idx):
                # Not enough bytes yet for a full frame
                break

            if not self._validate_checksum_and_etx(start_idx):
                # Bad checksum or ETX; discard first byte and continue
                self._rx_buffer.pop(0)
                continue

            payload, frame_length = self._extract_frame(start_idx)
            if self.frame_callback:
                self.frame_callback(payload)

            # Remove the processed frame from the buffer
            self._rx_buffer = self._rx_buffer[start_idx + frame_length:]
    
    def _find_frame_start(self):
        """
        Returns index of start-of-frame (STX) in rx_buffer, or None if not found.
        """
        for i, byte in enumerate(self._rx_buffer):
            if byte == STX:
                return i
        return None

    def _has_complete_frame(self, start_idx):
        """
        Returns True if rx_buffer contains all bytes for the frame starting at start_idx.
        """
        if len(self._rx_buffer) <= start_idx + 1:
            return False  # can't read length byte yet
        length_byte = self._rx_buffer[start_idx + 1]
        frame_end_idx = start_idx + 2 + length_byte + 1  # 2: STX+LEN, +1: ETX
        return len(self._rx_buffer) >= frame_end_idx

    def _validate_checksum_and_etx(self, start_idx):
        """
        Returns True if checksum matches and ETX is correct; False otherwise.
        """
        length_byte = self._rx_buffer[start_idx + 1]
        payload_start = start_idx + 2
        payload_end = payload_start + length_byte
        payload = self._rx_buffer[payload_start:payload_end]
        checksum = self._rx_buffer[payload_end]

        # Calculate checksum
        if self._calculate_xor_checksum(payload) != checksum:
            return False
        
        # Check ETX
        etx_idx = payload_end + 1
        if self._rx_buffer[etx_idx] != ETX:
            return False
        
        return True

    def _extract_frame(self, start_idx):
        """
        Extracts payload and frame length
        """
        length_byte = self._rx_buffer[start_idx + 1]
        payload_start = start_idx + 2
        payload_end = payload_start + length_byte
        payload = self._rx_buffer[payload_start:payload_end]
        frame_length = 2 + length_byte + 2
        return payload, frame_length
    





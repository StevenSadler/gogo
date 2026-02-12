#!usr/bin/env python3

import serial
import time

STX = 0xAA
ETX = 0x55
STX_SIZE = 1
LEN_SIZE = 1
CHECKSUM_SIZE = 1
ETX_SIZE = 1
MAX_PAYLOAD_LENGTH = 31

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
        if length > MAX_PAYLOAD_LENGTH:
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
    # FRAME PROCESSING
    # --------------------------
    def _calculate_xor_checksum(self, payload: bytes):
        checksum = 0
        for b in payload:
            checksum ^= b
        return checksum
    
    def _process_buffer(self):
        """
        Process rx_buffer, extracting and handling complete frames.
        If need to wait for more bytes, break
        If need to clear corrupted frame, continue

        A complete frame includes these bytes:
        [STX][LEN][PAYLOAD...][CHECKSUM][ETX]
          1    1    N           1         1
        """
        while True:
            start_idx = self._find_frame_start()
            if start_idx is None:
                # No STX found, clear garbage and stop
                self._rx_buffer.clear()
                break
            elif start_idx > 0:
                # Remove garbage before STX
                self._rx_buffer = self._rx_buffer[start_idx:]
                start_idx = 0
            
            # Check if we have length byte
            if not self._has_length(start_idx):
                break
            
            payload_length = self._payload_length(start_idx)

            # Check for invalid length
            if payload_length > MAX_PAYLOAD_LENGTH:
                self._rx_buffer.pop(start_idx)
                continue

            frame_length = self._frame_length(payload_length)

            # Check if full frame is present
            if len(self._rx_buffer) < start_idx + frame_length:
                break

            payload_start = start_idx + STX_SIZE + LEN_SIZE
            payload_end = payload_start + payload_length
            payload = self._rx_buffer[payload_start:payload_end]
            checksum = self._rx_buffer[payload_end]
            etx = self._rx_buffer[payload_end + CHECKSUM_SIZE]

            # Validate checksum and ETX
            if self._calculate_xor_checksum(payload) != checksum or etx != ETX:
                # Corrupted frame -> remove first byte
                self._rx_buffer.pop(0)
                continue
            
            # Valid frame -> call callback
            if self.frame_callback:
                self.frame_callback(payload)

            # Remove the processed frame
            self._rx_buffer = self._rx_buffer[start_idx + frame_length:]
    
    def _find_frame_start(self):
        for i, byte in enumerate(self._rx_buffer):
            if byte == STX:
                return i
        return None
    
    def _has_length(self, start_idx):
        remaining = len(self._rx_buffer) - start_idx
        return remaining >= STX_SIZE + LEN_SIZE
    
    def _payload_length(self, start_idx):
        return self._rx_buffer[start_idx + STX_SIZE]
    
    def _frame_length(self, payload_length):
        return STX_SIZE + LEN_SIZE + payload_length + CHECKSUM_SIZE + ETX_SIZE
    





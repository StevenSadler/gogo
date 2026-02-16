#!/usr/bin/env python3
from gogo_control.hardware.structured_telemetry import FrameID, SubID, LogSeverity, decode_structured_payload

class SerialMessageParser:
    """
    Parses incoming serial payloads (legacy ASCII or structured) and
    logs them via the provided logger.
    """

    def __init__(self, logger, encoder_callback):
        self.logger = logger
        self.encoder_callback = encoder_callback
        # cache structured frame IDs for efficiency
        self.structured_ids = {fid.value for fid in FrameID}

    def handle_payload(self, payload: bytes):
        if not payload:
            return

        first_byte = payload[0]

        if first_byte in self.structured_ids:
            self._handle_structured(payload)
        else:
            self._handle_legacy(payload)

    # --------------------------
    # Structured message handling
    # --------------------------
    def _handle_structured(self, payload: bytes):
        try:
            data = decode_structured_payload(payload)
        except Exception as e:
            self.logger.warn(f"Failed to decode structured payload: {e}")
            return

        frame_id = data.get("frame_id")
        try:
            frame_enum = FrameID(frame_id)
        except ValueError:
            self.logger.warn(f"Unknown structured frame: {frame_id}")
            return

        if frame_enum == FrameID.HEARTBEAT:
            left = data.get("left_tps")
            right = data.get("right_tps")
            self.logger.info(f"[FW HB] L:{left} R:{right}")

        elif frame_enum == FrameID.ENCODER_FEED:
            left = data.get("left_ticks")
            right = data.get("right_ticks")
            timestamp = data.get("timestamp_ms")
            self.encoder_callback(left, right, timestamp)

        elif frame_enum == FrameID.ERROR:
            code = data.get("error_code")
            msg = data.get("message", "")

            try:
                error_enum = SubID(code)
            except ValueError:
                error_enum = None


            if error_enum == SubID.WATCHDOG_EXPIRED:
                self.logger.error(f"[FW ERROR Watchdog expired] {msg}")
            elif error_enum == SubID.UNKNOWN_COMMAND:
                self.logger.error(f"[FW ERROR Unknown command] {msg}")
            elif error_enum == SubID.MOTOR_FAULT:
                self.logger.error(f"[FW ERROR Motor fault] {msg}")
            else:
                self.logger.error(f"parser error ERROR code {code}")
                # self.logger.warn(f"[FW ERROR {code}] event: {msg}")

        elif frame_enum == FrameID.MODE_ACK:
            mode = data.get("mode")
            mode_names = {
                0: "IDLE",
                1: "TEST",
                2: "DRIVE"
            }
            name = mode_names.get(mode, f"UNKNOWN({mode})")
            self.logger.info(f"[FW MODE_ACK] Mode={name}")

        elif frame_enum == FrameID.STATUS_EVENT:
            event = data.get("event")
            if event == SubID.MOTOR_CLAMP_APPLIED:
                self.logger.warn("[FW STATUS] Motor clamp applied")
            elif event == SubID.ROBOCLAW_CONNECTED:
                self.logger.warn("[FW STATUS] Roboclaw connected")
            elif event == SubID.ROBOCLAW_LOST:
                self.logger.warn("[FW STATUS] Roboclaw lost")
            else:
                self.logger.warn(f"[FW STATUS] event: {event}")
        
        elif frame_enum == FrameID.LOG:
            severity = data.get("severity")
            msg = data.get("message", "")

            if severity == SubID.INFO_GENERAL:
                self.logger.info(f"[FW] {msg}")
            elif severity == SubID.WARN_GENERAL:
                self.logger.warn(f"[FW] {msg}")
            elif severity == SubID.ERROR_GENERAL:
                self.logger.error(f"[FW] {msg}")
            else:
                self.logger.error(f"parser error LOG severity {severity}")
                # self.logger.info(f"[FW] {msg}")  # safe fallback


    # --------------------------
    # Legacy ASCII message handling
    # --------------------------
    def _handle_legacy(self, payload: bytes):
        try:
            msg = payload.decode("ascii")
        except UnicodeDecodeError:
            self.logger.warn("Received undecodable legacy frame")
            return

        if "WATCHDOG" in msg or "WARNING" in msg:
            self.logger.warn(f"[FW LEGACY] {msg}")
        elif "Heartbeat" in msg:
            self.logger.debug(f"[FW LEGACY] {msg}")
        else:
            self.logger.info(f"[FW LEGACY] {msg}")

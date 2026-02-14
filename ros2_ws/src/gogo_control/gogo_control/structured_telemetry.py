from enum import IntEnum


# ==============================
# FRAME IDS
# ==============================
class FrameID(IntEnum):
    HEARTBEAT = 0x01
    ENCODER_FEED = 0x02
    ERROR = 0x03
    MODE_ACK = 0x04
    STATUS_EVENT = 0x05
    LOG = 0x06


# ==============================
# SUB IDS
# ==============================
class SubID(IntEnum):
    # ERROR subIDs
    WATCHDOG_EXPIRED    = 0x01
    UNKNOWN_COMMAND     = 0x02
    MOTOR_FAULT         = 0x03

    # STATUS_EVENT subIDs
    MOTOR_CLAMP_APPLIED = 0x10
    ROBOCLAW_CONNECTED  = 0x11
    ROBOCLAW_LOST       = 0x12

    # LOG subIDs
    INFO_GENERAL        = 0x20
    WARN_GENERAL        = 0x21
    ERROR_GENERAL       = 0x22



# ==============================
# LOG SEVERITY
# ==============================
class LogSeverity(IntEnum):
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3


# ==============================
# DECODER
# ==============================
def decode_structured_payload(payload: bytes):
    """
    Expects payload without STX/LEN/CHECKSUM/ETX.
    First byte = frame_id.
    """

    if not payload:
        return None

    frame_id = payload[0]
    data = payload[1:]

    result = {
        "frame_id": frame_id
    }

    try:
        frame_enum = FrameID(frame_id)
    except ValueError:
        result["type"] = "UNKNOWN"
        result["raw"] = data
        return result

    result["type"] = frame_enum.name

    if frame_enum == FrameID.HEARTBEAT:
        if len(data) >= 4:
            result["left_tps"] = int.from_bytes(data[0:2], "little", signed=True)
            result["right_tps"] = int.from_bytes(data[2:4], "little", signed=True)

    elif frame_enum == FrameID.ENCODER_FEED:
        if len(data) >= 8:
            result["left_ticks"] = int.from_bytes(data[0:4], "little", signed=True)
            result["right_ticks"] = int.from_bytes(data[4:8], "little", signed=True)

    elif frame_enum == FrameID.ERROR:
        if len(data) >= 1:
            result["error_code"] = data[0]
            result["message"] = data[1:].decode("ascii", errors="ignore")

    elif frame_enum == FrameID.MODE_ACK:
        if len(data) >= 1:
            result["mode"] = data[0]

    elif frame_enum == FrameID.STATUS_EVENT:
        if len(data) >= 1:
            result["event"] = data[0]

    elif frame_enum == FrameID.LOG:
        if len(data) >= 1:
            result["severity"] = data[0]
            result["message"] = data[1:].decode("ascii", errors="ignore")

    return result

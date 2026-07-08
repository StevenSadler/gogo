#pragma once

#include <Arduino.h>
#include "FramedSerialPrinter.h"
#include "ControlMode.h"

// Forward declare ControlMode so the header compiles
enum class ControlMode : uint8_t;

// ==============================
// FRAME IDS - PRIMARY MESSAGE TYPES
// ==============================
enum class FrameID : uint8_t {
    HEARTBEAT      = 0x01,
    ENCODER_FEED   = 0x02,
    ERROR          = 0x03,
    MODE_ACK       = 0x04,
    STATUS_EVENT   = 0x05,
    LOG            = 0x06,
    IDENTITY       = 0x07
};

// ==============================
// SUB IDS - GLOBALLY UNIQUE
// ==============================
enum class SubID : uint8_t {
    // ERROR subIDs
    WATCHDOG_EXPIRED    = 0x01,
    UNKNOWN_COMMAND     = 0x02,
    MOTOR_FAULT         = 0x03,

    // STATUS_EVENT subIDs
    MOTOR_CLAMP_APPLIED = 0x10,
    ROBOCLAW_CONNECTED  = 0x11,
    ROBOCLAW_LOST       = 0x12,

    // LOG subIDs
    INFO_GENERAL        = 0x20,
    WARN_GENERAL        = 0x21,
    ERROR_GENERAL       = 0x22
};

// ==============================
// LOG SEVERITY
// ==============================
enum class LogSeverity : uint8_t {
    DEBUG = 0,
    INFO  = 1,
    WARN  = 2,
    ERROR = 3
};

// ==============================
// STRUCTURED TELEMETRY SENDER
// ==============================
class StructuredTelemetry {
public:
    static constexpr size_t MAX_PAYLOAD = FramedSerialPrinter::MAX_PAYLOAD;
    
    explicit StructuredTelemetry(HardwareSerial& serial)
        : printer(serial) {}

    // ------------------------------
    // HEARTBEAT
    // ------------------------------
    void sendHeartbeat(int16_t left_tps, int16_t right_tps) {
        begin(FrameID::HEARTBEAT);
        writeInt16(left_tps);
        writeInt16(right_tps);
        commit();
    }

    // ------------------------------
    // ENCODER FEED
    // ------------------------------
    void sendEncoder(int32_t left_ticks, int32_t right_ticks, uint32_t timestamp_ms) {
        begin(FrameID::ENCODER_FEED);
        writeInt32(left_ticks);
        writeInt32(right_ticks);
        writeUInt32(timestamp_ms);
        commit();
    }

    // ------------------------------
    // ERROR
    // ------------------------------
    void sendError(SubID code, const char* msg) {
        begin(FrameID::ERROR);
        printer.write(static_cast<uint8_t>(code));
        writeString(msg);
        commit();
    }

    // ------------------------------
    // MODE ACK
    // ------------------------------
    void sendModeAck(ControlMode mode) {
        begin(FrameID::MODE_ACK);
        printer.write(static_cast<uint8_t>(mode));
        commit();
    }

    // ------------------------------
    // STATUS EVENT
    // ------------------------------
    void sendStatusEvent(SubID event) {
        begin(FrameID::STATUS_EVENT);
        printer.write(static_cast<uint8_t>(event));
        commit();
    }

    // ------------------------------
    // LOG
    // ------------------------------
    void sendLog(SubID severity, const char* msg) {
        begin(FrameID::LOG);
        printer.write(static_cast<uint8_t>(severity));
        writeString(msg);
        commit();
    }

    // ------------------------------
    // IDENTITY
    // ------------------------------
    void sendIdentity(uint32_t build_hash, uint32_t contract_hash) {
        begin(FrameID::IDENTITY);
        writeUInt32(build_hash);
        writeUInt32(contract_hash);
        commit();
    }

private:
    FramedSerialPrinter printer;

    void begin(FrameID id) {
        printer.write(static_cast<uint8_t>(id));
    }

    void commit() {
        printer.commit();
    }

    void writeInt16(int16_t value) {
        printer.write((uint8_t)(value & 0xFF));
        printer.write((uint8_t)((value >> 8) & 0xFF));
    }

    void writeInt32(int32_t value) {
        printer.write((uint8_t)(value & 0xFF));
        printer.write((uint8_t)((value >> 8) & 0xFF));
        printer.write((uint8_t)((value >> 16) & 0xFF));
        printer.write((uint8_t)((value >> 24) & 0xFF));
    }

    void writeUInt32(uint32_t value) {
        printer.write((uint8_t)(value & 0xFF));
        printer.write((uint8_t)((value >> 8) & 0xFF));
        printer.write((uint8_t)((value >> 16) & 0xFF));
        printer.write((uint8_t)((value >> 24) & 0xFF));
    }

    void writeString(const char* msg) {
        if (!msg) return;

        for (size_t i = 0; msg[i] != '\0' && i < MAX_PAYLOAD; i++) {
            printer.write(static_cast<uint8_t>(msg[i]));
        }
    }
};

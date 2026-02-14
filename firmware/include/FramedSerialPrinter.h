#pragma once

#include <Arduino.h>
#include "ISafePrinter.h"

class FramedSerialPrinter : public ISafePrinter {
public:
    explicit FramedSerialPrinter(HardwareSerial& s)
        : serial(s), length(0) {}

    // Make base overloads visible
    using ISafePrinter::commit;

    static constexpr uint8_t MAX_PAYLOAD = 31;

    // -----------------------------
    // Core write: buffer one byte
    // -----------------------------
    void write(uint8_t byte) override {
        if (length >= MAX_PAYLOAD) {
            // Buffer full — silently drop extra data
            return;
        }

        buffer[length++] = byte;
    }

    // -----------------------------
    // Commit: send framed packet
    // -----------------------------
    void commit() override {
        if (length == 0) {
            return;  // nothing to send
        }

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < length; ++i) {
            checksum ^= buffer[i];
        }

        // Frame format:
        // [STX][LEN][PAYLOAD...][CHECKSUM][ETX]

        sendByte(STX);
        sendByte(length);

        for (uint8_t i = 0; i < length; ++i) {
            sendByte(buffer[i]);
        }

        sendByte(checksum);
        sendByte(ETX);

        // Reset buffer after sending
        length = 0;
    }

private:
    HardwareSerial& serial;

    static constexpr uint8_t STX = 0xAA;
    static constexpr uint8_t ETX = 0x55;

    uint8_t buffer[MAX_PAYLOAD];
    uint8_t length;

    // Non-blocking send (100ms max wait like SerialSafePrinter)
    void sendByte(uint8_t byte) {
        unsigned long start = millis();
        while (serial.availableForWrite() == 0) {
            if (millis() - start > 100) {
                return;  // fail silently
            }
        }
        serial.write(byte);
    }
};

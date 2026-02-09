#pragma once
#include <Arduino.h>

class SerialManager {
public:
    SerialManager() : bufIndex(0) {}

    // Handle serial input; calls provided callback with payload string
    void handleSerial(void (*callback)(const char* cmd)) {
        while (Serial.available() > 0) {
            uint8_t byteIn = Serial.read();

            switch (state) {
                case WAIT_STX:
                    if (byteIn == STX) {
                        bufIndex = 0;
                        state = READ_LEN;
                    }
                    break;

                case READ_LEN:
                    payloadLen = byteIn;
                    if (payloadLen > SERIAL_BUF_SIZE - 1) {
                        state = WAIT_STX; // too long, discard
                    } else {
                        state = READ_PAYLOAD;
                    }
                    break;

                case READ_PAYLOAD:
                    serialBuf[bufIndex++] = byteIn;
                    if (bufIndex >= payloadLen) {
                        state = READ_CHKSUM;
                    }
                    break;

                case READ_CHKSUM:
                    checksum = byteIn;
                    if (computeChecksum(serialBuf, payloadLen) != checksum) {
                        state = WAIT_STX; // bad checksum, discard
                    } else {
                        state = WAIT_ETX;
                    }
                    break;

                case WAIT_ETX:
                    if (byteIn == ETX) {
                        serialBuf[payloadLen] = '\0'; // null terminate
                        callback(serialBuf);           // pass payload string
                    }
                    state = WAIT_STX; // ready for next frame
                    break;
            }
        }
    }

private:
    static constexpr uint8_t STX = 0xAA;
    static constexpr uint8_t ETX = 0x55;
    static constexpr size_t SERIAL_BUF_SIZE = 32;

    char serialBuf[SERIAL_BUF_SIZE];
    size_t bufIndex;
    uint8_t payloadLen;
    uint8_t checksum;

    enum State {
        WAIT_STX,
        READ_LEN,
        READ_PAYLOAD,
        READ_CHKSUM,
        WAIT_ETX
    } state = WAIT_STX;

    // Simple checksum: XOR checksum of payload bytes
    static uint8_t computeChecksum(const char* buf, size_t len) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; i++) {
            checksum ^= buf[i];
        }
        return checksum;
    }
};

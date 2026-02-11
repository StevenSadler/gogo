#pragma once
#include <Arduino.h>
#include "ISafePrinter.h"

class SerialSafePrinter : public ISafePrinter {
public:
    explicit SerialSafePrinter(HardwareSerial& s) : serial(s) {}

    // Make base commit overloads visible
    using ISafePrinter::commit;
    
    // Core write: one byte at a time
    void write(uint8_t byte) override {
        unsigned long start = millis();
        while (serial.availableForWrite() == 0) {
            if (millis() - start > 100) {
                return; // fail silently, non-blocking
            }
        }
        serial.write(byte);
    }

    void commit() override {
        // No-op: SerialSafePrinter sends bytes immediately
    }

private:
    HardwareSerial& serial;
};

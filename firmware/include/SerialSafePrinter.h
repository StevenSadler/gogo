#pragma once
#include <Arduino.h>
#include "ISafePrinter.h"

class SerialSafePrinter : public ISafePrinter {
public:
    explicit SerialSafePrinter(HardwareSerial& s) : serial(s) {}

    // this function is called by print and println
    void write(uint8_t byte) override {
        unsigned long start = millis();
        while (serial.availableForWrite() == 0) {
            if (millis() - start > 100) {
                return; // fail silently, non-blocking
            }
        }
        serial.write(byte);
    }

private:
    HardwareSerial& serial;
};

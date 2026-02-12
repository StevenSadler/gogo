#pragma once
#include <stdint.h>
#include <Arduino.h> // for String

class ISafePrinter {
public:
    virtual ~ISafePrinter() = default;

    // Core primitive: one byte at a time
    virtual void write(uint8_t byte) = 0;

    // -------- Convenience (collect pieces) --------
    virtual void print(const char* s) {
        while (*s) write(static_cast<uint8_t>(*s++));
    }

    virtual void print(const String& s) {
        print(s.c_str());
    }

    virtual void print(int value) {
        char buf[12]; // enough for int32
        itoa(value, buf, 10);
        print(buf);
    }

    virtual void print(float value, int precision = 2) {
        char buf[16];
        dtostrf(value, 0, precision, buf);
        print(buf);
    }

    // -------- Commit --------
    // Base class provides overloads with optional extra data
    void commit(const char* extra) {
        if (*extra) print(extra);
        commit(); // call subclass implementation
    }

    void commit(const String& extra) {
        commit(extra.c_str());
    }

    void commit(int value) {
        print(value);
        commit();
    }

    void commit(float value, int precision = 2) {
        print(value, precision);
        commit();
    }

    // Parameterless commit — must be implemented by subclass
    virtual void commit() = 0;
};

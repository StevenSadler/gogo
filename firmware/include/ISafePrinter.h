#pragma once
#include <stdint.h>

class ISafePrinter {
public:
    virtual ~ISafePrinter() = default;

    // Core primitive: one byte at a time
    virtual void write(uint8_t byte) = 0;

    // -------- Convenience (defined ONCE) --------
    void print(const char* s) {
        while (*s) {
            write(static_cast<uint8_t>(*s++));
        }
    }

    void println(const char* s) {
        print(s);
        write('\n');
    }

    void print(int value) {
        char buf[12]; // enough for int32
        itoa(value, buf, 10);
        print(buf);
    }

    void println(int value) {
        print(value);
        write('\n');
    }
};

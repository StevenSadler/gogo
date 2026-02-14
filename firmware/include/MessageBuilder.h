#pragma once

#include <Arduino.h>
#include "FramedSerialPrinter.h"
#include "StructuredTelemetry.h"
#include <stdarg.h>
#include <stdio.h>

class MessageBuilder {
public:
    // Maximum payload allowed in FramedSerialPrinter
    // FramedSerialPrinter MAX_PAYLOAD = 31
    // Reserve 1 byte for FrameID (and optional second byte if needed)
    static constexpr size_t MAX_PAYLOAD = FramedSerialPrinter::MAX_PAYLOAD;

    // Build a formatted message for a given frame, safely truncated
    // Returns a pointer to a static buffer
    static const char* build(FrameID frameID, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);

        // Reserve 1 byte for FrameID (optional: add another for secondary code if needed)
        size_t max_msg_len = MAX_PAYLOAD - 1;  

        // Format into static buffer safely
        vsnprintf(buffer, max_msg_len, fmt, args);

        va_end(args);
        return buffer;
    }

private:
    // Single static buffer for simplicity (shared across calls)
    static char buffer[MAX_PAYLOAD];
};

// Definition of static buffer
char MessageBuilder::buffer[MessageBuilder::MAX_PAYLOAD] = {0};

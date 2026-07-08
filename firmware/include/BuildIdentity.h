#pragma once
#include <Arduino.h>
#include "generated/build_metadata.h"
#include "StructuredTelemetry.h"

struct BuildIdentity {
    uint32_t buildHash;
    uint32_t contractHash;
    StructuredTelemetry& telemetry;

    BuildIdentity(StructuredTelemetry& telemetry)
        : buildHash(hexToU32(BUILD_HASH))
        , contractHash(hexToU32(CONTRACT_HASH))
        , telemetry(telemetry) {}

    void send() const {
        telemetry.sendIdentity(buildHash, contractHash);
    }

    static uint32_t hexToU32(const char* s) {
        uint32_t result = 0;

        for (int i = 0; i < 8 && s[i] != '\0'; i++) {
            char c = s[i];

            result <<= 4; // shift 4 bits per hex digit

            if (c >= '0' && c <= '9') {
                result |= (c - '0');
            } else if (c >= 'a' && c <= 'f') {
                result |= (c - 'a' + 10);
            } else if (c >= 'A' && c <= 'F') {
                result |= (c - 'A' + 10);
            } else {
                // optional: treat invalid char as 0 nibble
                // or ignore entirely (current behavior keeps stable output)
            }
        }

        return result;
    }
};

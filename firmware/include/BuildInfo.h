#pragma once
#include <Arduino.h>
#include "generated/build_hash.h"

struct BuildInfo {
    const char* buildHash;
    const char* buildTimestampUTC;
    const char* libsHash;

    BuildInfo()
        : buildHash(BUILD_HASH)
        , buildTimestampUTC(BUILD_TIMESTAMP_UTC)
        , libsHash(LIBS_HASH) {}

    void report() const {
        Serial.println();
        Serial.println(F("===== Firmware Build Info ====="));
        Serial.print(F("Build hash: "));
        Serial.println(buildHash);
        Serial.print(F("Build time: "));
        Serial.println(buildTimestampUTC);
        Serial.print(F("Libs hash : "));
        Serial.println(libsHash);
        Serial.println(F("================================"));
    }
};

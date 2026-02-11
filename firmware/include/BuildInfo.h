#pragma once
#include <Arduino.h>
#include "generated/build_hash.h"
#include "ISafePrinter.h"

struct BuildInfo {
    const char* buildHash;
    const char* buildTimestampUTC;
    const char* libsHash;
    ISafePrinter& printer;

    BuildInfo(ISafePrinter& safePrinter)
        : buildHash(BUILD_HASH)
        , buildTimestampUTC(BUILD_TIMESTAMP_UTC)
        , libsHash(LIBS_HASH)
        , printer(safePrinter) {}

    void report() const {
        printer.commit();
        printer.commit("===== Firmware Build Info =====");
        printer.print("Build hash: ");
        printer.commit(buildHash);
        printer.print("Build time: ");
        printer.commit(buildTimestampUTC);
        printer.print("Libs hash : ");
        printer.commit(libsHash);
        printer.commit("================================");
    }
};

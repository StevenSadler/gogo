#pragma once
#include <Arduino.h>
#include "generated/build_hash.h"
#include "StructuredTelemetry.h"
#include "MessageBuilder.h"

struct BuildInfo {
    const char* buildHash;
    const char* buildTimestampUTC;
    const char* libsHash;
    const char* contractHash;
    StructuredTelemetry& telemetry;

    BuildInfo(StructuredTelemetry& telemetry)
        : buildHash(BUILD_HASH)
        , buildTimestampUTC(BUILD_TIMESTAMP_UTC)
        , libsHash(LIBS_HASH)
        , contractHash(CONTRACT_HASH)
        , telemetry(telemetry) {}

    void report() const {
        telemetry.sendLog(SubID::INFO_GENERAL, 
            MessageBuilder::build(FrameID::LOG, "===== Firmware Build Info ====="));
        telemetry.sendLog(SubID::INFO_GENERAL, 
            MessageBuilder::build(FrameID::LOG, "B hash: %s", buildHash));
        telemetry.sendLog(SubID::INFO_GENERAL, 
            MessageBuilder::build(FrameID::LOG, "B time: %s", buildTimestampUTC));
        telemetry.sendLog(SubID::INFO_GENERAL, 
            MessageBuilder::build(FrameID::LOG, "L hash: %s", libsHash));
        telemetry.sendLog(SubID::INFO_GENERAL, 
            MessageBuilder::build(FrameID::LOG, "C hash: %s", contractHash));
        telemetry.sendLog(SubID::INFO_GENERAL, 
            MessageBuilder::build(FrameID::LOG, "================================"));
    }
};

#pragma once
#include <stdint.h>

enum class ControlMode : uint8_t {
    IDLE_MODE,
    TEST_MODE,
    DRIVE_MODE
};

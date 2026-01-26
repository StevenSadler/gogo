#include <Arduino.h>
#include "BuildInfo.h"
#include "SerialManager.h"
#include "MotorController.h"

// ----------------------
// CONFIG
// ----------------------
constexpr unsigned long LOOP_PERIOD_MS = 20;  // 50 Hz loop

// ----------------------
// GLOBAL OBJECTS
// ----------------------
SerialManager serialManager;
MotorController motors(-500, 500, 20);
BuildInfo buildInfo;

// Used to run loop at fixed frequency
static unsigned long lastLoopMs = 0; // static preserves value between loop() calls

// ----------------------
// SERIAL COMMAND HANDLER
// ----------------------
void handleCommand(const char* cmd) {
    // Expecting motor command "left,right"
    int left, right;
    if (sscanf(cmd, "%d,%d", &left, &right) == 2) {
        motors.setTarget(left, right);
    }
}

// ----------------------
// SETUP
// ----------------------
void setup() {
    serialManager.begin(38400);
    buildInfo.report();
}

// ----------------------
// LOOP
// ----------------------
void loop() {
    unsigned long now = millis();
    unsigned long elapsed = now - lastLoopMs;

    // Run at fixed period
    if (elapsed >= LOOP_PERIOD_MS) {
        lastLoopMs += LOOP_PERIOD_MS;  // increment by fixed period, not "now"

        serialManager.handleSerial(handleCommand);
        motors.update(now);
    }
}

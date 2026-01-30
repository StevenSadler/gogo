#include <Arduino.h>
#include "MotorController.h"

// ----------------------
// CONFIG
// ----------------------
constexpr unsigned long LOOP_PERIOD_MS = 20; // 50 Hz loop
constexpr unsigned long TEST_DURATION_MS = 5000; // 5-second test

MotorController motors(-500, 500, 20); // min, max, max accel per loop

static unsigned long lastLoopMs = 0;
static unsigned long testStartMs = 0;
static bool testRunning = true;

void setup() {
    Serial.begin(38400);
    delay(100); // USB stabilization
    Serial.println(F("Firmware alive"));

    Serial.println(F("Probing Roboclaw..."));
    motors.probe(); // safe communication test

    Serial.println(F("Starting 5-second motor test..."));
    testStartMs = millis();

    // Safe initial targets
    motors.setTarget(100, 100);
}

void loop() {
    unsigned long now = millis();
    unsigned long elapsed = now - lastLoopMs;

    if (elapsed >= LOOP_PERIOD_MS) {
        lastLoopMs += LOOP_PERIOD_MS;

        motors.update(now);

        // 5-second ramp test
        if (testRunning) {
            unsigned long testElapsed = now - testStartMs;

            if (testElapsed < 1250) {
                motors.setTarget(100, 100);
            } else if (testElapsed < 2500) {
                motors.setTarget(400, 400);
            } else if (testElapsed < 3750) {
                motors.setTarget(700, 700); // clamped internally
            } else if (testElapsed < 5000) {
                motors.setTarget(400, 400);
            } else {
                motors.setTarget(0, 0);
                testRunning = false;
                Serial.println(F("Test complete, motors stopped."));
            }
        }

        // Debug output
        Serial.print(F("Target L: ")); Serial.print(motors.leftTarget);
        Serial.print(F("  R: ")); Serial.print(motors.rightTarget);
        Serial.print(F("  Current L: ")); Serial.print(motors.leftCurrent);
        Serial.print(F("  R: ")); Serial.println(motors.rightCurrent);
    }
}

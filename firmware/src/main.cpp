#include <Arduino.h>
#include "BuildInfo.h"
#include "MotorController.h"
#include "TestHarness.h"

// ----------------------
// MODE
// ----------------------
enum class ControlMode {
    IDLE_MODE,
    TEST_MODE,
    SERIAL_MODE
};

ControlMode mode = ControlMode::TEST_MODE;  // start in TEST for now

// ----------------------
// CONFIG
// ----------------------
constexpr unsigned long LOOP_PERIOD_MS = 20;   // 50 Hz

MotorController motors(-1000, 1000); // min, max
TestHarness testHarness(motors);
BuildInfo buildInfo;

static unsigned long lastLoopMs = 0;

// ----------------------
// MODE ENTRY FUNCTIONS
// ----------------------
void enterIdle() {
    mode = ControlMode::IDLE_MODE;
    motors.setTarget(0, 0);   // immediately stop motors
    Serial.println(F("Entered IDLE_MODE"));
}

void enterTest() {
    mode = ControlMode::TEST_MODE;
    motors.setTarget(0, 0);   // immediately stop motors
    testHarness.start();      // reset harness timing
    Serial.println(F("Entered TEST_MODE"));
}

void enterSerial() {
    mode = ControlMode::SERIAL_MODE;
    // Motors need to stop, for now do nothing until serial control is implemented
    //motors.setTarget(0, 0);   // start safely stopped
    Serial.println(F("Entered SERIAL_MODE"));
}

void setup() {
    Serial.begin(38400);
    buildInfo.report();
    delay(100);
    Serial.println(F("Firmware alive"));

    motors.begin();
    Serial.println(F("Probing Roboclaw..."));
    motors.probe();

    // choose initial mode
    enterTest();
}

void loop() {
    unsigned long now = millis();

    if (now - lastLoopMs >= LOOP_PERIOD_MS) {
        lastLoopMs += LOOP_PERIOD_MS;

        switch (mode) {
            case ControlMode::IDLE_MODE:
                // Intentionally empty for now
                break;

            case ControlMode::TEST_MODE:
                testHarness.update(now);
                break;

            case ControlMode::SERIAL_MODE:
                // Intentionally empty (serial control not wired yet)
                // later call motors.update(now) or call it in future serial control code
                break;
        }

        motors.update(now);
    }
}

#include <Arduino.h>
#include "BuildInfo.h"
#include "MotorController.h"

// ----------------------
// CONFIG
// ----------------------
constexpr unsigned long LOOP_PERIOD_MS = 20;   // 50 Hz
constexpr unsigned long HOLD_MS        = 2000; // hold each speed
constexpr unsigned long STOP_MS        = 1000; // stop between speeds

MotorController motors(-1000, 1000); // min, max
BuildInfo buildInfo;

static unsigned long lastLoopMs   = 0;
static unsigned long segmentStart = 0;
static size_t stepIndex           = 0;
static bool inStopSegment         = true;

// Speeds to test (ticks/sec)
const int speedSteps[] = {400, 600, 800, 1000};
constexpr size_t NUM_STEPS = sizeof(speedSteps) / sizeof(speedSteps[0]);

void setup() {
    Serial.begin(38400);
    buildInfo.report();
    delay(100);
    Serial.println(F("Firmware alive"));

    motors.begin();

    Serial.println(F("Probing Roboclaw..."));
    motors.probe();

    Serial.println(F("Starting stepped speed test with stops"));

    // Start stopped
    motors.setTarget(0, 0);
    segmentStart = millis();
}

void loop() {
    unsigned long now = millis();

    if (now - lastLoopMs >= LOOP_PERIOD_MS) {
        lastLoopMs += LOOP_PERIOD_MS;

        motors.update(now);

        unsigned long segmentElapsed = now - segmentStart;

        // ----------------------
        // STOP SEGMENT
        // ----------------------
        if (inStopSegment) {
            if (segmentElapsed >= STOP_MS) {
                if (stepIndex < NUM_STEPS) {
                    motors.setTarget(speedSteps[stepIndex],
                                     speedSteps[stepIndex]);

                    Serial.print(F("Holding speed: "));
                    Serial.println(speedSteps[stepIndex]);

                    inStopSegment = false;
                    segmentStart  = now;
                } else {
                    // End of test: force motors fully stopped
                    motors.setTarget(0, 0);
                    motors.leftCurrent  = 0;
                    motors.rightCurrent = 0;
                    motors.update(now);

                    Serial.println(F("Test complete, motors stopped."));
                    while (true) delay(1000);
                }
            }
        }
        // ----------------------
        // SPEED HOLD SEGMENT
        // ----------------------
        else {
            if (segmentElapsed >= HOLD_MS) {
                motors.setTarget(0, 0);

                Serial.println(F("Stopping between speeds"));

                inStopSegment = true;
                segmentStart  = now;
                stepIndex++;
            }
        }

        // ----------------------
        // DEBUG OUTPUT
        // ----------------------
        Serial.print(F("t=")); Serial.print(now);
        Serial.print(F("  tgt=")); Serial.print(motors.leftTarget);
        Serial.print(F("  cur=")); Serial.println(motors.leftCurrent);
    }
}

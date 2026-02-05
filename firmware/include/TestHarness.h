#pragma once
#include <Arduino.h>
#include "MotorController.h"

// ----------------------
// TEST HARNESS
// ----------------------
class TestHarness {
public:
    TestHarness(MotorController& mc)
        : motors(mc)
        , SPEED_STEPS{400, 600, 800, 1000}  // initialize array here
        , NUM_STEPS(sizeof(SPEED_STEPS) / sizeof(SPEED_STEPS[0]))
        , running(false)
        , stepIndex(0)
        , inStopSegment(true)
    {}

    // Call when entering TEST mode
    void start() {
        running = true;
        stepIndex = 0;
        inStopSegment = true;
        segmentStart = millis();

        // Ensure motors start from zero
        motors.setTarget(0, 0);

        Serial.println(F("TestHarness started"));
    }

    // Call every loop with current millis()
    void update(unsigned long now) {
        if (!running) return;

        unsigned long segmentElapsed = now - segmentStart;

        // ----------------------
        // STOP SEGMENT
        // ----------------------
        if (inStopSegment) {
            if (segmentElapsed >= STOP_MS) {
                if (stepIndex < NUM_STEPS) {
                    motors.setTarget(SPEED_STEPS[stepIndex],
                                     SPEED_STEPS[stepIndex]);

                    Serial.print(F("Holding speed: "));
                    Serial.println(SPEED_STEPS[stepIndex]);

                    inStopSegment = false;
                    segmentStart  = now;
                } else {
                    // End of test: force motors fully stopped
                    motors.setTarget(0, 0);

                    Serial.println(F("Test complete, motors stopped."));
                    running = false;
                }
            } else {
                motors.setTarget(0, 0);
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
            } else {
                motors.setTarget(SPEED_STEPS[stepIndex],
                                 SPEED_STEPS[stepIndex]);
            }
        }
    }

private:
    MotorController& motors;
    const int SPEED_STEPS[4];
    const size_t NUM_STEPS;

    // state
    bool running;
    size_t stepIndex;
    bool inStopSegment;
    unsigned long segmentStart;

    // ----------------------
    // CONFIG
    // ----------------------
    static constexpr unsigned long HOLD_MS = 2000;   // hold each speed
    static constexpr unsigned long STOP_MS = 1000;   // stop between speeds
};

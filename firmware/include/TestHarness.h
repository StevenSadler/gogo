#pragma once
#include <Arduino.h>
#include "MotorController.h"
#include "ISafePrinter.h"

// ----------------------
// TEST HARNESS
// ----------------------
class TestHarness {
public:
    TestHarness(MotorController& mc, ISafePrinter& safePrinter)
        : motors(mc), printer(safePrinter)
    {}

    // Call when entering TEST mode
    void start() {
        stepIndex = 0;
        running = true;
        inStopSegment = true;
        segmentStart = millis();

        // Ensure motors start from zero
        motors.setTarget(0, 0);

        printer.println("TestHarness started");
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
                    motors.setTarget(leftSpeeds[stepIndex],
                                     rightSpeeds[stepIndex]);

                    printer.print("Holding speed: ");
                    printer.print(leftSpeeds[stepIndex]);
                    printer.print(", ");
                    printer.println(rightSpeeds[stepIndex]);

                    inStopSegment = false;
                    segmentStart  = now;
                } else {
                    // End of test: force motors fully stopped
                    motors.setTarget(0, 0);

                    printer.println("Test complete, motors stopped.");
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

                printer.println("Stopping between speeds");

                inStopSegment = true;
                segmentStart  = now;
                stepIndex++;
            } else {
                motors.setTarget(leftSpeeds[stepIndex],
                                 rightSpeeds[stepIndex]);
            }
        }
    }

private:
    MotorController& motors;
    ISafePrinter& printer;

    // state
    bool running;
    size_t stepIndex;
    bool inStopSegment;
    unsigned long segmentStart;

    static constexpr unsigned long HOLD_MS = 2000;   // hold each speed
    static constexpr unsigned long STOP_MS = 1000;   // stop between speeds

    static constexpr size_t NUM_STEPS = 4;
    const int leftSpeeds[NUM_STEPS]  = {400, 600, 800, 1000};
    const int rightSpeeds[NUM_STEPS] = {400, 600, 800, 1000};
};

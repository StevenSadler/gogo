#pragma once
#include <Arduino.h>

class MotorController {
public:
    MotorController(int minCmd, int maxCmd, int maxAccelPerLoop)
        : CMD_MIN(minCmd), CMD_MAX(maxCmd), MAX_ACCEL(maxAccelPerLoop),
          leftTarget(0), rightTarget(0), leftCurrent(0), rightCurrent(0),
          lastCmdTime(0) {}

    void setTarget(int left, int right) {
        leftTarget = clamp(left, CMD_MIN, CMD_MAX);
        rightTarget = clamp(right, CMD_MIN, CMD_MAX);
        lastCmdTime = millis();
    }

    void update(unsigned long now) {
        // Timeout: stop motors
        if (now - lastCmdTime > CMD_TIMEOUT_MS) {
            leftTarget = 0;
            rightTarget = 0;
        }

        // Ramp speeds toward targets
        leftCurrent = ramp(leftCurrent, leftTarget);
        rightCurrent = ramp(rightCurrent, rightTarget);

        // TODO: replace with real motor output
        // setMotorSpeedLeft(leftCurrent);
        // setMotorSpeedRight(rightCurrent);

        // LED indicator (optional)
        analogWrite(LED_BUILTIN, map(leftCurrent, CMD_MIN, CMD_MAX, 0, 255));
    }

private:
    const int CMD_MIN;
    const int CMD_MAX;
    const int MAX_ACCEL;
    static constexpr unsigned long CMD_TIMEOUT_MS = 100;

    int leftTarget, rightTarget;
    int leftCurrent, rightCurrent;
    unsigned long lastCmdTime;

    int clamp(int val, int minVal, int maxVal) const {
        if (val < minVal) return minVal;
        if (val > maxVal) return maxVal;
        return val;
    }

    int ramp(int current, int target) const {
        int delta = target - current;
        if (delta > MAX_ACCEL) delta = MAX_ACCEL;
        if (delta < -MAX_ACCEL) delta = -MAX_ACCEL;
        return current + delta;
    }
};

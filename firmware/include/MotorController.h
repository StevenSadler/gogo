#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Basicmicro.h>
#include "StructuredTelemetry.h"

struct EncoderCounts {
    int32_t leftTicks;
    int32_t rightTicks;
    unsigned long timestamp_ms;
};

class MotorController {
public:
    // ----------------------
    // CONSTRUCTOR
    // ----------------------
    MotorController(int minCmd, int maxCmd, StructuredTelemetry& telemetry)
        : CMD_MIN(minCmd), CMD_MAX(maxCmd),
          leftTarget(0), rightTarget(0),
          leftCurrent(0), rightCurrent(0),
          lastLeftTarget(0), lastRightTarget(0),
          clampActive(false),
          roboclaw(&roboclawSerial, 10000), // SoftwareSerial + 10ms timeout
          telemetry(telemetry)
    {}

    // ----------------------
    // BEGIN
    // ----------------------
    void begin() {
        roboclawSerial.begin(38400);
        delay(20);
    }

    // ----------------------
    // SET TARGET SPEEDS
    // ----------------------
    void setTarget(int left, int right) {
        int maxMag = max(abs(left), abs(right));
        if (maxMag > CMD_MAX) {
            // Scale both targets proportionally
            float scale = float(CMD_MAX) / float(maxMag);
            left = int(left * scale);
            right = int(right * scale);

            // Send status event only once until valid targets are received
            if (!clampActive) {
                telemetry.sendStatusEvent(SubID::MOTOR_CLAMP_APPLIED);
                telemetry.sendLog(SubID::WARN_GENERAL,
                    MessageBuilder::build(FrameID::LOG, "Motor clamp applied: max=%d", CMD_MAX));
                clampActive = true;
            }
        }
        else {
            clampActive = false;
        }

        leftTarget = left;
        rightTarget = right;

        // Only log if targets changed
        if (lastLeftTarget != leftTarget || lastRightTarget != rightTarget) {
            telemetry.sendLog(SubID::INFO_GENERAL, 
                MessageBuilder::build(FrameID::LOG, "setTarget: %d, %d",
                leftTarget, rightTarget));
            
            lastLeftTarget = leftTarget;
            lastRightTarget = rightTarget;
        }
    }

    // ----------------------
    // RAMP MOTOR OUTPUTS
    // ----------------------
    void update(unsigned long now) {
        int deltaLeft = leftTarget - leftCurrent;
        int deltaRight = rightTarget - rightCurrent;

        // Scale both deltas proportionally to preserve arc
        int maxDelta = max(abs(deltaLeft), abs(deltaRight));
        float scale = 1.0f;
        if (maxDelta > MAX_ACCEL) {
            scale = float(MAX_ACCEL) / float(maxDelta);
        }

        leftCurrent += int(deltaLeft * scale);
        rightCurrent += int(deltaRight * scale);

        // Final clamp to ensure we do not exceed max safe speed
        // This is a safeguard for floating point rounding errors
        leftCurrent = constrain(leftCurrent, CMD_MIN, CMD_MAX);
        rightCurrent = constrain(rightCurrent, CMD_MIN, CMD_MAX);

        // Send ramped speeds to Roboclaw
        roboclaw.SpeedM1(ROBOCLAW_ADDRESS, leftCurrent);
        roboclaw.SpeedM2(ROBOCLAW_ADDRESS, rightCurrent);

        static unsigned long lastHeartbeatMs = 0;
        if (now - lastHeartbeatMs >= 5000) {
            telemetry.sendHeartbeat(leftCurrent, rightCurrent);
            lastHeartbeatMs = now;
        }
    }

    // ----------------------
    // READ ENCODERS
    // ----------------------
    void readEncoders(EncoderCounts& counts) {
        counts.timestamp_ms = millis();
        counts.leftTicks = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS);
        counts.rightTicks = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS);
    }

    // ----------------------
    // SAFE ROBOCLAW PROBE
    // ----------------------
    bool probe() {
        uint32_t tick, state, enc1, enc2, speed1, speed2, ispeed1, ispeed2;
        uint16_t temp1, temp2, mainBatt, logicBatt;
        int16_t pwm1, pwm2, cur1, cur2;
        uint16_t speedError1, speedError2, posError1, posError2;

        bool ok = roboclaw.GetStatus(ROBOCLAW_ADDRESS,
                                     tick, state,
                                     temp1, temp2,
                                     mainBatt, logicBatt,
                                     pwm1, pwm2,
                                     cur1, cur2,
                                     enc1, enc2,
                                     speed1, speed2,
                                     ispeed1, ispeed2,
                                     speedError1, speedError2,
                                     posError1, posError2);
        if (ok) {
            telemetry.sendStatusEvent(SubID::ROBOCLAW_CONNECTED);
        } 
        else {
            telemetry.sendError(SubID::MOTOR_FAULT,
                MessageBuilder::build(FrameID::ERROR, "Roboclaw not responding"));
        }
        return ok;
    }

private:
    const int CMD_MIN;
    const int CMD_MAX;
    const uint8_t ROBOCLAW_ADDRESS = 0x80;

    int leftTarget;
    int rightTarget;
    int leftCurrent;
    int rightCurrent;
    int lastLeftTarget;
    int lastRightTarget;

    bool clampActive;

    static constexpr int MAX_ACCEL = 150;
    static constexpr int MIN_STEADY_SPEED = 0;

    SoftwareSerial roboclawSerial{10, 11};
    Basicmicro roboclaw;

    StructuredTelemetry& telemetry;
};

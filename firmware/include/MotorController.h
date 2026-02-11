#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Basicmicro.h>
#include "ISafePrinter.h"

class MotorController {
public:
    // ----------------------
    // CONSTRUCTOR
    // ----------------------
    MotorController(int minCmd, int maxCmd, ISafePrinter& safePrinter)
        : CMD_MIN(minCmd), CMD_MAX(maxCmd),
          leftTarget(0), rightTarget(0),
          leftCurrent(0), rightCurrent(0),
          lastLeftTarget(0), lastRightTarget(0),
          roboclaw(&roboclawSerial, 10000), // SoftwareSerial + 10ms timeout
          printer(safePrinter)
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
        leftTarget = constrain(left, CMD_MIN, CMD_MAX);
        rightTarget = constrain(right, CMD_MIN, CMD_MAX);

        if (-MIN_STEADY_SPEED < leftTarget && leftTarget < MIN_STEADY_SPEED){
            leftTarget = 0;
        }
        if (-MIN_STEADY_SPEED < rightTarget && rightTarget < MIN_STEADY_SPEED){
            rightTarget = 0;
        }

        if (lastLeftTarget != leftTarget || lastRightTarget != rightTarget) {
            printer.print("[MotorController] setTarget: leftTarget=");
            printer.print(leftTarget);
            printer.print(" rightTarget=");
            printer.commit(rightTarget);
        }
    }

    // ----------------------
    // RAMP MOTOR OUTPUTS
    // ----------------------
    void update(unsigned long now) {
        int deltaLeft = leftTarget - leftCurrent;
        int deltaRight = rightTarget - rightCurrent;

        int maxDelta = max(abs(deltaLeft), abs(deltaRight));
        float scale = 1.0f;
        if (maxDelta > MAX_ACCEL) {
            scale = float(MAX_ACCEL) / float(maxDelta);
        }

        leftCurrent += int(deltaLeft * scale);
        rightCurrent += int(deltaRight * scale);

        int leftClamped = constrain(leftCurrent, CMD_MIN, CMD_MAX);
        int rightClamped = constrain(rightCurrent, CMD_MIN, CMD_MAX);

        if (leftClamped != leftCurrent || rightClamped != rightCurrent) {
            printer.print("[MotorController] WARNING: Final clamp applied -> L:");
            printer.print(leftCurrent);
            printer.print("->");
            printer.print(leftClamped);
            printer.print(" R:");
            printer.print(rightCurrent);
            printer.print("->");
            printer.commit(rightClamped);
        }

        leftCurrent = leftClamped;
        rightCurrent = rightClamped;

        // Send ramped speeds to Roboclaw
        uint8_t address = 0x80;
        roboclaw.SpeedM1(address, leftCurrent);
        roboclaw.SpeedM2(address, rightCurrent);

        if (lastLeftTarget != leftTarget || lastRightTarget != rightTarget) {
            printer.print("[MotorController] Target changed -> L:");
            printer.print(leftTarget);
            printer.print(" R:");
            printer.commit(rightTarget);

            lastLeftTarget = leftTarget;
            lastRightTarget = rightTarget;
        }

        static unsigned long lastHeartbeatMs = 0;
        if (now - lastHeartbeatMs >= 1000) {
            printer.print("[MotorController] Heartbeat -> L:");
            printer.print(leftCurrent);
            printer.print(" R:");
            printer.commit(rightCurrent);
            lastHeartbeatMs = now;
        }
    }

    // ----------------------
    // SAFE ROBOCLAW PROBE
    // ----------------------
    bool probe() {
        uint8_t address = 0x80; // default Roboclaw address
        uint32_t tick, state, enc1, enc2, speed1, speed2, ispeed1, ispeed2;
        uint16_t temp1, temp2, mainBatt, logicBatt;
        int16_t pwm1, pwm2, cur1, cur2;
        uint16_t speedError1, speedError2, posError1, posError2;

        bool ok = roboclaw.GetStatus(address,
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
        if (ok) printer.commit("Roboclaw responded");
        else printer.commit("No response from Roboclaw");
        return ok;
    }

private:
    const int CMD_MIN;
    const int CMD_MAX;

    int leftTarget;
    int rightTarget;
    int leftCurrent;
    int rightCurrent;
    int lastLeftTarget;
    int lastRightTarget;

    static constexpr int MAX_ACCEL = 150;
    static constexpr int MIN_STEADY_SPEED = 0;

    SoftwareSerial roboclawSerial{10, 11};
    Basicmicro roboclaw;

    ISafePrinter& printer; // <-- use this for all printing
};

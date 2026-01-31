#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Basicmicro.h>

class MotorController {
public:
    // ----------------------
    // CONFIG
    // ----------------------
    const int CMD_MIN;
    const int CMD_MAX;
    const int MAX_ACCEL;

    // ----------------------
    // STATE
    // ----------------------
    int leftTarget;
    int rightTarget;
    int leftCurrent;
    int rightCurrent;

    unsigned long lastCmdTime;

    // Roboclaw mapping:
    // M1 = left wheel
    // M2 = right wheel


    // ----------------------
    // CONSTRUCTOR
    // ----------------------
    MotorController(int minCmd, int maxCmd, int maxAccelPerLoop)
        : CMD_MIN(minCmd), CMD_MAX(maxCmd), MAX_ACCEL(maxAccelPerLoop),
          leftTarget(0), rightTarget(0),
          leftCurrent(0), rightCurrent(0),
          lastCmdTime(0),
          roboclaw(&roboclawSerial, 10000)  // SoftwareSerial + 10ms timeout
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
    }

    // ----------------------
    // RAMP MOTOR OUTPUTS (locally, no Roboclaw yet)
    // ----------------------
    void update(unsigned long now) {
        // One-time start boost if motor is at 0 but target is nonzero
        const int START_BOOST = 70;  // adjust if needed

        // Left motor
        if (leftCurrent == 0 && leftTarget != 0) {
            leftCurrent = (leftTarget > 0) ? START_BOOST : -START_BOOST;
        } else if (leftCurrent < leftTarget) {
            leftCurrent += MAX_ACCEL;
            if (leftCurrent > leftTarget) leftCurrent = leftTarget;
        } else if (leftCurrent > leftTarget) {
            leftCurrent -= MAX_ACCEL;
            if (leftCurrent < leftTarget) leftCurrent = leftTarget;
        }

        // Right motor
        if (rightCurrent == 0 && rightTarget != 0) {
            rightCurrent = (rightTarget > 0) ? START_BOOST : -START_BOOST;
        } else if (rightCurrent < rightTarget) {
            rightCurrent += MAX_ACCEL;
            if (rightCurrent > rightTarget) rightCurrent = rightTarget;
        } else if (rightCurrent > rightTarget) {
            rightCurrent -= MAX_ACCEL;
            if (rightCurrent < rightTarget) rightCurrent = rightTarget;
        }

        // Send ramped speeds to Roboclaw
        uint8_t address = 0x80;
        roboclaw.SpeedM1(address, leftCurrent);
        roboclaw.SpeedM2(address, rightCurrent);
    }

    // ----------------------
    // SAFE ROBOT CLAW PROBE
    // ----------------------
    bool probe() {
        uint8_t address = 0x80; // default Roboclaw address
        // We'll only call GetStatus with dummy variables to test communication
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
        if (ok) Serial.println(F("Roboclaw responded"));
        else Serial.println(F("No response from Roboclaw"));
        return ok;
    }

private:
    SoftwareSerial roboclawSerial{10, 11}; // S2=10, S1=11
    Basicmicro roboclaw;
};

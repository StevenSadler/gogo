#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Basicmicro.h>

class MotorController {
public:
    // Roboclaw mapping:
    // M1 = left wheel
    // M2 = right wheel


    // ----------------------
    // CONSTRUCTOR
    // ----------------------
    MotorController(int minCmd, int maxCmd)
        : CMD_MIN(minCmd), CMD_MAX(maxCmd),
          leftTarget(0), rightTarget(0),
          leftCurrent(0), rightCurrent(0),
          lastLeftTarget(0), lastRightTarget(0),
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
        // constrain to min/max commands
        leftTarget = constrain(left, CMD_MIN, CMD_MAX);
        rightTarget = constrain(right, CMD_MIN, CMD_MAX);

        // snap to zero if inside deadband
        if (-MIN_STEADY_SPEED < leftTarget && leftTarget < MIN_STEADY_SPEED){
            leftTarget = 0;
        }
        if (-MIN_STEADY_SPEED < rightTarget && rightTarget < MIN_STEADY_SPEED){
            rightTarget = 0;
        }

        if (lastLeftTarget != leftTarget || lastRightTarget != rightTarget) {
            Serial.print(F("[MotorController] setTarget: leftTarget="));
            Serial.print(leftTarget);
            Serial.print(F(" rightTarget="));
            Serial.println(rightTarget);
        }
    }

    // ----------------------
    // RAMP MOTOR OUTPUTS (locally, no Roboclaw yet)
    // ----------------------
    void update(unsigned long now) {
        int deltaLeft = leftTarget - leftCurrent;
        int deltaRight = rightTarget - rightCurrent;

        // Scale proportionally if either delta exceeds MAX_ACCEL
        int maxDelta = max(abs(deltaLeft), abs(deltaRight));
        float scale = 1.0f;
        if (maxDelta > MAX_ACCEL) {
            scale = float(MAX_ACCEL) / float(maxDelta);
        }

        leftCurrent += int(deltaLeft * scale);
        rightCurrent += int(deltaRight * scale);

        // ----------------------
        // Final failsafe: enforce motor limits
        // Upstream code (XboxTwist/TwistSerial) should respect targets, but MotorController
        // guarantees the robot never exceeds safe min/max speeds. This is the authoritative clamp.
        // ----------------------
        int leftClamped = constrain(leftCurrent, CMD_MIN, CMD_MAX);
        int rightClamped = constrain(rightCurrent, CMD_MIN, CMD_MAX);

        // Log if clamping actually occurred
        if (leftClamped != leftCurrent || rightClamped != rightCurrent) {
            Serial.print(F("[MotorController] WARNING: Final clamp applied -> "));
            Serial.print(F("deltaL:"));
            Serial.print(deltaLeft);
            Serial.print(F("deltaR:"));
            Serial.print(deltaRight);
            Serial.print(F(" L:"));
            Serial.print(leftCurrent);
            Serial.print(F("->"));
            Serial.print(leftClamped);
            Serial.print(F(" R:"));
            Serial.print(rightCurrent);
            Serial.print(F("->"));
            Serial.println(rightClamped);
        }

        leftCurrent = leftClamped;
        rightCurrent = rightClamped;

        // Send ramped speeds to Roboclaw
        uint8_t address = 0x80;
        roboclaw.SpeedM1(address, leftCurrent);
        roboclaw.SpeedM2(address, rightCurrent);

        // Debug print if targets changed
        if (lastLeftTarget != leftTarget || lastRightTarget != rightTarget) {
            Serial.print(F("[MotorController] Target changed -> L:"));
            Serial.print(leftTarget);
            Serial.print(F(" R:"));
            Serial.println(rightTarget);

            lastLeftTarget = leftTarget;
            lastRightTarget = rightTarget;
        }

        // Heartbeat every second
        static unsigned long lastHeartbeatMs = 0;
        if (now - lastHeartbeatMs >= 1000) {
            Serial.print(F("[MotorController] Heartbeat -> L:"));
            Serial.print(leftCurrent);
            Serial.print(F(" R:"));
            Serial.println(rightCurrent);
            lastHeartbeatMs = now;
        }

    }

    // ----------------------
    // SAFE ROBOCLAW PROBE
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
    const int CMD_MIN;
    const int CMD_MAX;

    int leftTarget;
    int rightTarget;
    int leftCurrent;
    int rightCurrent;
    int lastLeftTarget;
    int lastRightTarget;
    
    static constexpr int MAX_ACCEL = 150;        // max tps change per loop
    static constexpr int MIN_STEADY_SPEED = 0;   // 400; // min tps to avoid motor stutter


    SoftwareSerial roboclawSerial{10, 11}; // S2=10, S1=11
    Basicmicro roboclaw;

};

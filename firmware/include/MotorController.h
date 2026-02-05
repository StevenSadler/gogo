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

        lastTargetMs = millis();
        watchdogExpired = false;

        Serial.print(F("[MotorController] setTarget: leftTarget="));
        Serial.print(leftTarget);
        Serial.print(F(" rightTarget="));
        Serial.println(rightTarget);

    }

    // ----------------------
    // RAMP MOTOR OUTPUTS (locally, no Roboclaw yet)
    // ----------------------
    void update(unsigned long now) {
        // Check watchdog
        if (now - lastTargetMs > WATCHDOG_TIMEOUT_MS) {
            if (!watchdogExpired) {
                Serial.println(F("[MotorController] WATCHDOG EXPIRED -> forcing stop"));
                watchdogExpired = true;
            }
            leftTarget = 0;
            rightTarget = 0;
        }

        leftCurrent = rampMotor(leftCurrent, leftTarget);
        rightCurrent = rampMotor(rightCurrent, rightTarget);

        // Send ramped speeds to Roboclaw
        uint8_t address = 0x80;
        roboclaw.SpeedM1(address, leftCurrent);
        roboclaw.SpeedM2(address, rightCurrent);

        // Serial.print(F("[MotorController] update: leftCurrent="));
        // Serial.print(leftCurrent);
        // Serial.print(F(" rightCurrent="));
        // Serial.println(rightCurrent);

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
    const int CMD_MIN;
    const int CMD_MAX;
    int leftTarget;
    int rightTarget;
    int leftCurrent;
    int rightCurrent;
    unsigned long lastCmdTime;
    
    static constexpr int MAX_ACCEL = 25;         // max tps change per loop
    static constexpr int MIN_STEADY_SPEED = 400; // min tps to avoid motor stutter
    

    SoftwareSerial roboclawSerial{10, 11}; // S2=10, S1=11
    Basicmicro roboclaw;

    // Watchdog
    static constexpr unsigned long WATCHDOG_TIMEOUT_MS = 200;
    unsigned long lastTargetMs = 0;
    bool watchdogExpired = false;

    int rampMotor(int current, int target) {
        // FORWARD ACCELERATION (0 <= current, current < target)
        // either stopped or moving forward, needing forward acceleration
        if (0 <= current && current < target) {
            current += MAX_ACCEL;
            if (target < current) current = target;                 // clamp to target
            if (current < MIN_STEADY_SPEED) current = MIN_STEADY_SPEED; // enforce min steady
        }
        // REVERSE ACCELERATION (current <= 0, target < current)
        // either stopped or moving backward, needing reverse acceleration
        else if (target < current && current <= 0) {
            current -= MAX_ACCEL;
            if (current < target) current = target;                 // clamp to target
            if (-MIN_STEADY_SPEED < current) current = -MIN_STEADY_SPEED; // enforce min steady
        }
        // FORWARD DECELERATION (0 < current, target < current)
        // strictly moving forward, needing forward deceleration
        else if (0 < current && target < current)  {
            current -= MAX_ACCEL;
            if (current < target) current = target;                // clamp to target
            // no deadband applied here
        }
        // REVERSE DECELERATION (current < 0, current < target)
        else if (current < 0 && current < target) {
            current += MAX_ACCEL;
            if (target < current) current = target;               // clamp to target
            // no deadband applied here
        }
        // else current == target → do nothing

        return current;
    }

};

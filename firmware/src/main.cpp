#include <Arduino.h>
#include "generated/contract.h"
#include "BuildIdentity.h"
#include "BuildInfo.h"
#include "MotorController.h"
#include "SerialManager.h"
#include "TestHarness.h"
#include "StructuredTelemetry.h"
#include "MessageBuilder.h"
#include "ControlMode.h"

ControlMode mode;

// ----------------------
// CONFIG
// ----------------------
constexpr unsigned long WATCHDOG_TIMEOUT_MS = 300;   // 0.3 s

SerialManager serialMgr;
StructuredTelemetry telemetry(Serial);
MotorController motors(telemetry);
TestHarness testHarness(motors, telemetry);
BuildInfo buildInfo(telemetry);
BuildIdentity buildIdentity(telemetry);

bool watchdogExpiryNoted = false; // did watchdog just expire
unsigned long lastControlMs = 0;
unsigned long lastOdomMs = 0;
unsigned long lastCommandMs = 0;  // last time a motor command was sent

// ----------------------
// MODE ENTRY FUNCTIONS
// ----------------------
void enterIdle() {
    mode = ControlMode::IDLE_MODE;
    motors.setTarget(0, 0);   // immediately stop motors
    telemetry.sendModeAck(mode);
}

void enterTest() {
    mode = ControlMode::TEST_MODE;
    motors.setTarget(0, 0);   // immediately stop motors
    testHarness.start();      // reset harness timing
    telemetry.sendModeAck(mode);
}

void enterDrive() {
    mode = ControlMode::DRIVE_MODE;
    motors.setTarget(0, 0);   // immediately stop motors
    telemetry.sendModeAck(mode);
}

void enterAuto() {
    // for now, duplicate DRIVE mode behavior
    mode = ControlMode::AUTO_MODE;
    motors.setTarget(0, 0);   // immediately stop motors
    telemetry.sendModeAck(mode);
}

const char* modeToString(ControlMode m) {
    switch (m) {
        case ControlMode::IDLE_MODE:
            return "MODE_IDLE";
        case ControlMode::TEST_MODE:
            return "MODE_TEST";
        case ControlMode::DRIVE_MODE:
            return "MODE_DRIVE";
        case ControlMode::AUTO_MODE:
            return "MODE_AUTO";
        default:
            return "UNKNOWN";
    }
}

// ----------------------
// WATCHDOG FUNCTIONS
// ----------------------
bool watchdogExpired() {
    return (millis() - lastCommandMs) > WATCHDOG_TIMEOUT_MS;
}

void kickWatchdog() {
    lastCommandMs = millis();
}

void handleSerialCommand(const char* cmd) {
    // Mode commands first
    if (strcmp(cmd, "MODE_IDLE") == 0) {
        enterIdle();
        return;
    }
    if (strcmp(cmd, "MODE_TEST") == 0) {
        enterTest();
        return;
    }
    if (strcmp(cmd, "MODE_DRIVE") == 0) {
        enterDrive();
        return;
    }
    if (strcmp(cmd, "MODE_AUTO") == 0) {
        enterAuto();
        return;
    }

    // Motor commands: format "left_tps,right_tps"
    int left = 0, right = 0;
    if (sscanf(cmd, "%d,%d", &left, &right) == 2) {
        motors.setTarget(left, right);
        kickWatchdog();
        return;
    }

    // Unknown command
    telemetry.sendError(SubID::UNKNOWN_COMMAND, 
        MessageBuilder::build(FrameID::ERROR, cmd));
}

void setup() {
    Serial.begin(38400);
    buildInfo.report();
    delay(100);
    telemetry.sendLog(SubID::INFO_GENERAL, 
        MessageBuilder::build(FrameID::LOG, "Firmware alive"));

    motors.begin();
    telemetry.sendLog(SubID::INFO_GENERAL, 
        MessageBuilder::build(FrameID::LOG, "Probing Roboclaw..."));

    motors.probe();

    buildIdentity.send();

    // choose initial mode
    enterIdle();
    kickWatchdog();
}

void loop() {
    unsigned long now = millis();

    // ----------------------
    // MOTOR CONTROL LOOP
    // ----------------------
    if (now - lastControlMs >= CONTROL_PERIOD_MS) {
        lastControlMs += CONTROL_PERIOD_MS;

        // Handle incoming serial commands
        serialMgr.handleSerial(handleSerialCommand);

        // Update control depending on mode
        switch (mode) {
            case ControlMode::IDLE_MODE:
                // Already stopped, no motor updates needed
                break;

            case ControlMode::TEST_MODE:
                kickWatchdog();            // simulate messages arriving
                testHarness.update(now);   // run test commands
                break;

            case ControlMode::DRIVE_MODE:
                // Motor commands handled in handleSerialCommand
                break;
            
            case ControlMode::AUTO_MODE:
                // Motor commands handled in handleSerialCommand
                break;
        }

        // Watchdog check (applies to TEST, DRIVE, and AUTO only)
        if ((mode == ControlMode::TEST_MODE || mode == ControlMode::DRIVE_MODE || mode == ControlMode::AUTO_MODE) && watchdogExpired()) {
            if (!watchdogExpiryNoted) {
                motors.setTarget(0,0);
                telemetry.sendError(SubID::WATCHDOG_EXPIRED, nullptr);
                watchdogExpiryNoted = true;
            }
        } else {
            watchdogExpiryNoted = false;
        }

        // Update motor outputs (ramping and send to Roboclaw)
        motors.update(now);
    }

    // ----------------------
    // ODOM LOOP
    // ----------------------
    if (now - lastOdomMs >= ODOM_PERIOD_MS) {
        lastOdomMs += ODOM_PERIOD_MS;

        EncoderCounts counts;
        motors.readEncoders(counts);
        telemetry.sendEncoder(counts.leftTicks, counts.rightTicks, counts.timestamp_ms);
    }
}

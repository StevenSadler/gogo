#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>

#define RX_PIN 10
#define TX_PIN 11

#define CMD_TIMEOUT_MS 500

// ---------------- RoboClaw ----------------
#define ROBOCLAW_ADDRESS 0x80
#define ROBOCLAW_BAUD    115200

SoftwareSerial roboclawSerial(RX_PIN, TX_PIN);
RoboClaw roboclaw(&roboclawSerial, 10000);

unsigned long last_cmd_time = 0;
bool e_stop = true;

// -----------------------------------------
void setup()
{
  // USB serial (PC / RPi)
  Serial.begin(115200);
  while (!Serial) { }

  // RoboClaw serial
  roboclawSerial.begin(ROBOCLAW_BAUD);
  roboclaw.begin(ROBOCLAW_BAUD);

  delay(100);

  last_cmd_time = millis();

  Serial.println("Simple USB -> RoboClaw bridge ready");
}

// -----------------------------------------
void loop()
{
  if (Serial.available())
  {
    // Read one line (blocking, simple)
    String line = Serial.readStringUntil('\n');

    long left_tps = 0;
    long right_tps = 0;

    // Expected: "C: <left> <right>"
    if (sscanf(line.c_str(), "C: %ld %ld", &left_tps, &right_tps) == 2)
    {
      roboclaw.SpeedM1(ROBOCLAW_ADDRESS, left_tps);
      roboclaw.SpeedM2(ROBOCLAW_ADDRESS, right_tps);

      last_cmd_time = millis();
      e_stop = false;

      // Optional debug
      Serial.print("Set ");
      Serial.print(left_tps);
      Serial.print(" ");
      Serial.println(right_tps);
    }
  }

  unsigned long now = millis();
  if (!e_stop && (now - last_cmd_time > CMD_TIMEOUT_MS))
  {
    roboclaw.SpeedM1(ROBOCLAW_ADDRESS, 0);
    roboclaw.SpeedM2(ROBOCLAW_ADDRESS, 0);
    e_stop = true;

    Serial.println("WATCHDOG: motors stopped");
  }
}

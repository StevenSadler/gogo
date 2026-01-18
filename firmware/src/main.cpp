#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>

#define RX_PIN 10
#define TX_PIN 11

// ---------------- RoboClaw ----------------
#define ROBOCLAW_ADDRESS 0x80
#define ROBOCLAW_BAUD    115200

SoftwareSerial roboclawSerial(RX_PIN, TX_PIN);
RoboClaw roboclaw(&roboclawSerial, 10000);

// -----------------------------------------
void setup()
{
  // USB serial (PC / RPi)
  Serial.begin(ROBOCLAW_BAUD);
  while (!Serial) { }

  // RoboClaw serial
  roboclawSerial.begin(ROBOCLAW_BAUD);
  roboclaw.begin(ROBOCLAW_BAUD);

  Serial.println("USB -> RoboClaw bridge ready");
}

// -----------------------------------------
void loop()
{
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');

    long left_tps = 0;
    long right_tps = 0;

    // Expected: "C: <left> <right>"
    if (sscanf(line.c_str(), "C: %ld %ld", &left_tps, &right_tps) == 2)
    {
      roboclaw.SpeedM1(ROBOCLAW_ADDRESS, left_tps);
      roboclaw.SpeedM2(ROBOCLAW_ADDRESS, right_tps);

      // Optional debug
      Serial.print("Set ");
      Serial.print(left_tps);
      Serial.print(" ");
      Serial.println(right_tps);
    }
  }
}

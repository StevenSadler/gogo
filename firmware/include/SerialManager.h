#pragma once
#include <Arduino.h>
#include <ctype.h> // for isspace()

class SerialManager {
public:
    SerialManager() : bufIndex(0) {}

    void begin(unsigned long baud) {
        Serial.begin(baud);
        delay(100); // give time for USB connection
    }

    // Handle serial input; calls provided callback with trimmed command
    void handleSerial(void (*callback)(const char* cmd)) {
        while (Serial.available() > 0) {
            char c = Serial.read();

            // Line ending: parse buffer
            if (c == '\n' || c == '\r') {
                if (bufIndex > 0) {
                    serialBuf[bufIndex] = '\0';
                    char* cmd = trimWhitespace(serialBuf);
                    callback(cmd);
                    bufIndex = 0;
                }
                continue;
            }

            // Append character if space permits
            if (bufIndex < SERIAL_BUF_SIZE - 1) {
                serialBuf[bufIndex++] = c;
            } else {
                bufIndex = 0; // overflow
            }
        }
    }

private:
    static constexpr size_t SERIAL_BUF_SIZE = 32;
    char serialBuf[SERIAL_BUF_SIZE];
    size_t bufIndex;

    // Trim leading and trailing whitespace in-place
    static char* trimWhitespace(char* str) {
        if (!str) return str;

        // Leading
        while (*str && isspace((unsigned char)*str)) str++;

        if (*str == 0) return str;

        // Trailing
        char* end = str;
        while (*end) end++;
        end--;
        while (end > str && isspace((unsigned char)*end)) {
            *end = '\0';
            end--;
        }
        return str;
    }
};

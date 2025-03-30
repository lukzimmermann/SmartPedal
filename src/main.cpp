#include <Arduino.h>

#include "servo.h"
#define DIRECTION_PIN 32
#define PULSE_PIN 33
#define COUNT_PIN 25
#define ANALOG_PIN 26

Servo servo;

const uint16_t MAX_POSITION = 8000;
unsigned long lastPrint = 0;
unsigned long lastUpdate = 0;
bool state = false;
long position = 0;

void setup() {
    pinMode(ANALOG_PIN, INPUT);
    Serial.begin(115200);
    servo.begin(DIRECTION_PIN, PULSE_PIN, COUNT_PIN);
    servo.setPID(0.1, 0.0100000, 0.0100000);
}

void loop() {
    if (lastPrint + 100 < millis()) {
        // Serial.println(analogRead(ANALOG_PIN));
        //  Serial.print(millis());
        //  Serial.print("; ");
        //  Serial.println(position);
        lastPrint = millis();
    }

    if (lastUpdate + 10 < millis()) {
        lastUpdate = millis();

        uint16_t targetPosition =
            map(analogRead(ANALOG_PIN), 0, 4095, 0, MAX_POSITION);

        // if (state) {
        //     targetPosition = 0;
        // } else {
        //     targetPosition = MAX_POSITION;
        // }
        position = servo.update(targetPosition);
    }
}

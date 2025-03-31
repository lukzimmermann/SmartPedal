#include <Arduino.h>
#include <math.h>

#include "HX711.h"
#include "servo.h"

#define DIRECTION_PIN 32
#define PULSE_PIN 33
#define COUNT_PIN 25
#define ANALOG_PIN 26

const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

Servo servo;
HX711 scale;

const uint16_t MAX_POSITION = 16000;
unsigned long lastPrint = 0;
unsigned long lastUpdate = 0;
bool state = false;
long position = 0;
unsigned long absCounter = 0;
uint16_t absFactor = 0;
bool absState = false;

long force = 0;

void setup() {
    pinMode(ANALOG_PIN, INPUT);
    Serial.begin(115200);
    servo.begin(DIRECTION_PIN, PULSE_PIN, COUNT_PIN);
    servo.setPID(0.0125, 0.0065, 0.0001);

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale();
    delay(1000);
    scale.tare();
}

void loop() {
    if (lastPrint + 100 < millis()) {
        // Serial.println(force);
        //   Serial.print("; ");
        //   Serial.println(position);
        lastPrint = millis();
    }

    if (scale.is_ready()) {
        force = scale.get_units(1);
    }

    if (lastUpdate + 1 < millis()) {
        lastUpdate = millis();

        absCounter += 1;

        if (absCounter > 10) {
            absCounter = 0;
            absState = !absState;
        }

        uint16_t targetPosition =
            map(analogRead(ANALOG_PIN), 0, 4095, 0, MAX_POSITION);

        long maxForce = -100000;  // min 10'000
        if (force > 0) {
            force = 0;
        }
        if (force < maxForce) {
            force = maxForce;
        }

        if (targetPosition < 1) {
            targetPosition = map(force, 0, maxForce, 0, MAX_POSITION);
        }
        uint16_t absEffect = 0;
        if (targetPosition > 12000) {
            absEffect = map(targetPosition, 12000, 15000, 0, 500);
            if (absState) {
                absEffect = absEffect * -1;
            }
        }

        targetPosition += absEffect;
        position = servo.update(targetPosition);
    }
}

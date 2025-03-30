#include "servo.h"

#include <math.h>

uint8_t ledcChannel = 0;
const uint16_t MAX_FREQUENCY = 3000;  // 44800;
bool Servo::direction = true;
volatile long Servo::pulseCount = 8000;
unsigned long Servo::lastPulseTime = 0;
unsigned long Servo::currentFrequency = 0;

Servo::Servo() : pid(&input, &output, &setPoint, kp, ki, kd, DIRECT) {}

void Servo::begin(int directionPin, int pulsePin, int interruptPin) {
    this->directionPin = directionPin;
    ledcSetup(ledcChannel, 20000, 2);
    ledcAttachPin(pulsePin, ledcChannel);

    pinMode(interruptPin, INPUT_PULLUP);
    pinMode(directionPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(interruptPin), Servo::countPulse,
                    RISING);

    digitalWrite(directionPin, direction);
    pid.SetMode(AUTOMATIC);
}

void Servo::setPID(double kp, double ki, double kd) {
    pid.SetTunings(kp, ki, kd);
}

long Servo::update(long targetPosition) {
    setPoint = targetPosition;
    input = pulseCount;
    delay(50);

    pid.Compute();

    // int16_t rawFrequency =
    //     map(output * 10000, 0, 2550000, -1 * MAX_FREQUENCY, MAX_FREQUENCY);

    double rawFrequency = (output - 0.0) *
                              (MAX_FREQUENCY - -1.0 * MAX_FREQUENCY) /
                              (255.0 - 0.0) +
                          -1.0 * MAX_FREQUENCY;

    if (rawFrequency < 0) {
        direction = false;
    } else {
        direction = true;
    }

    if (pulseCount > 8000) {
        pulseCount = 8000;
        // direction = false;
    }

    if (pulseCount < 0) {
        // direction = true;
        pulseCount = 0;
    }

    digitalWrite(directionPin, direction);

    uint32_t newFreq = abs(rawFrequency);
    // uint32_t newFreq = targetPosition;

    uint8_t resolution = 10;
    if (newFreq < 3000) {
        resolution = 12;
    }
    if (newFreq < 100) {
        ledcWrite(ledcChannel, 0);
    } else {
        ledcSetup(ledcChannel, newFreq, resolution);
        ledcWrite(ledcChannel, pow(2, resolution) / 2);
    }

    if (true) {
        Serial.print("target: ");
        Serial.print(targetPosition);
        Serial.print("; current: ");
        Serial.print(pulseCount);
        Serial.print("; freq: ");
        Serial.print(rawFrequency);
        Serial.print("; out: ");
        Serial.print(output);
        Serial.print("; dir ");
        Serial.print(direction);
        Serial.println("; ");
    }
    return pulseCount;
}

void IRAM_ATTR Servo::countPulse() {
    if (direction) {
        pulseCount++;
    } else {
        pulseCount--;
    }
}
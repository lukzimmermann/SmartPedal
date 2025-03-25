#include <Arduino.h>
#include <PID_v1.h>

#include "HX711.h"

const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;

const int directionPin = 32;
const int stepPin = 33;
const int analogPin = 26;
bool pulseState = false;
int pulseFrequency = 500;
unsigned long lastPulseTime = 0;
unsigned long position = 5;
bool direction = true;

double setpoint, input, output;
double Kp = 0.08, Ki = 0.00005, Kd = 0.0001;  // 0.2 0.005 0.01
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

unsigned long lastPrint = 0;
long pidOutput = 0;

long force = 0;

void setup() {
    Serial.begin(115200);

    pinMode(directionPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(analogPin, INPUT);

    digitalWrite(directionPin, direction);
    myPID.SetMode(AUTOMATIC);

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale();
    Serial.println("Tare... remove any weights from the scale.");
    delay(1000);
    scale.tare();
}

void loop() {
    myPID.Compute();

    input = position;
    uint16_t analogValue = analogRead(analogPin);
    if (analogValue < 1000) {
        setpoint = map(force * -1, 0, 40000, 1600, 8000);
    } else {
        setpoint = map(analogValue, 0, 4095, 1600, 8000);
    }
    pidOutput = map(output * 10, 0, 2550, -40000, 40000);
    // pulseFrequency = abs(pidOutput);

    if (lastPrint + 100 < millis()) {
        Serial.print(analogValue);
        Serial.print("\t");
        Serial.print(setpoint);
        Serial.print("\t");
        Serial.print(position);
        Serial.print("\t");
        Serial.print(output);
        Serial.print("\t");
        Serial.print(pidOutput);
        Serial.print("\t");
        Serial.print(direction);
        Serial.print("\t");
        Serial.print(force);
        Serial.println("");
        lastPrint = millis();
    }

    if (lastPulseTime + 1000000.0 / pulseFrequency < micros()) {
        if (pulseState) {
            digitalWrite(stepPin, pulseState);
            pulseState = !pulseState;
            if (direction) {
                position += 1;
            } else {
                position -= 1;
            }
        } else {
            digitalWrite(stepPin, pulseState);
            pulseState = !pulseState;
        }

        if (pidOutput > 0) {
            direction = true;
        } else {
            direction = false;
        }

        if (position > 8000) {
            position = 8000;
            direction = false;
        }

        if (position < 1) {
            direction = true;
            position = 0;
        }

        digitalWrite(directionPin, direction);
        lastPulseTime = micros();
    }

    // Serial.println(pulseState);
    if (scale.is_ready()) {
        force = scale.get_units(1);
    }
}
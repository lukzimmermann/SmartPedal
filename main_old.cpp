#include <Arduino.h>

#define OUTPUT_PIN 33
#define COUNT_PIN 25

int freq = 20000;
const int ledcChannel = 0;
const int resolution = 2;

volatile unsigned long pulseCount = 0;

unsigned long lastPrint = 0;

void IRAM_ATTR countPulse() { pulseCount++; }

void setFrequency(uint32_t newFreq) {
    if (newFreq == 0) {
        ledcWrite(ledcChannel, 0);
    } else {
        ledcSetup(ledcChannel, newFreq, resolution);
        ledcWrite(ledcChannel, 2);
    }
}

void setup() {
    Serial.begin(115200);
    ledcSetup(ledcChannel, freq, resolution);
    ledcAttachPin(OUTPUT_PIN, ledcChannel);

    pinMode(COUNT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(COUNT_PIN), countPulse, RISING);

    setFrequency(uint32_t(20000));
}

void loop() {
    if (lastPrint + 1000 < millis()) {
        Serial.print(millis());
        Serial.print("; ");
        Serial.println(pulseCount);

        lastPrint = millis();
    }
}

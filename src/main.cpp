#include <Arduino.h>
#define OUTPUT_PIN 33

int freq = 1000;
const int ledcChannel = 0;
const int resolution = 2;

void setFrequency(uint32_t newFreq) {
    if (newFreq == 0) {
        ledcWrite(ledcChannel, 0);
    } else {
        ledcSetup(ledcChannel, newFreq, resolution);
        ledcWrite(ledcChannel, 2);
    }
}

void setup() {
    ledcSetup(ledcChannel, freq, resolution);
    ledcAttachPin(OUTPUT_PIN, ledcChannel);
}

void loop() {
    setFrequency(uint32_t(20000));
    delay(1000);
}

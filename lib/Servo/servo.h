#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <PID_v1.h>

class Servo {
   public:
    Servo();
    void begin(int directionPin, int pulsePin, int interruptPin);
    void setPID(double kp, double ki, double kd);
    long update(long targetPosition);

    static void IRAM_ATTR countPulse();

   private:
    volatile static long pulseCount;
    static unsigned long currentFrequency;
    static unsigned long lastPulseTime;
    uint8_t ledcChannel;
    PID pid;
    int directionPin;
    static bool direction;
    double kp, ki, kd;
    double setPoint;
    double input;
    double output;
};

#endif
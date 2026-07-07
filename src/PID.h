#ifndef P4P_SRC_PID_H_
#define P4P_SRC_PID_H_

#include "Arduino.h"

class PID {
public:
    PID(float kp, float ki, float kd, float windupLimit, float saturationHigh, float saturationLow);
    float update(float target, float current, float dt);
    float getOutput();
    void setSaturationLimits(float saturationHigh, float saturationLow);

private:
    float kp, ki, kd;
    float windupLimit;
    float saturationHigh, saturationLow;
    float integral, derivative;
    float lastError;
    float output;
};

#endif  // P4P_SRC_PID_H_